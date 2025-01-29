"""Moonraker component for managing TFT display communication.

This module provides an adapter for interfacing with TFT displays,
handling serial communication, command processing, and status reporting.
"""

from __future__ import annotations
import os
import time
import logging
import asyncio
import re
from typing import (
    TYPE_CHECKING,
    Any,
    Tuple,
    Optional,
    Dict,
    List,
    Callable,
    Coroutine,
    Union,
)
try:
    import serial
    from jinja2 import Template
except ImportError:
    logging.error("Missing required dependencies: pyserial and/or jinja2")
    raise

from ..utils import ServerError  # Changed to absolute import

# Annotation imports
if TYPE_CHECKING:
    from .power import PrinterPower
    from ..confighelper import ConfigHelper
    from .klippy_apis import KlippyAPI as APIComp
    from .file_manager.file_manager import FileManager as FMComp
    FlexCallback = Callable[..., Optional[Coroutine]]

PRINT_STATUS_TEMPLATE = (
    "//action:notification "
    "Layer Left {{ (print_stats.info.current_layer or 0) }}/{{ (print_stats.info.total_layer or 0) }}\n"
    "//action:notification "
    "Data Left {{ (virtual_sdcard.file_position or 0) }}/{{ (virtual_sdcard.file_size or 0) }}"
)

TEMPERATURE_TEMPLATE = (
    "T:{{ extruder.temperature | int }} /{{ extruder.target | int }} "
    "B:{{ heater_bed.temperature | int }} /{{ heater_bed.target | int }} "
    "@:0 B@:0"
)

PROBE_OFFSET_TEMPLATE = (
    "M851 X{{ config.bltouch.x_offset | float - gcode_move.homing_origin[0] }} "
    "Y{{ config.bltouch.y_offset | float - gcode_move.homing_origin[1] }} "
    "Z{{ config.bltouch.z_offset | float - gcode_move.homing_origin[2] }}"
)

REPORT_SETTINGS_TEMPLATE = (
    "M203 X{{ toolhead.max_velocity }} Y{{ toolhead.max_velocity }} "
    "Z{{ config.printer.max_z_velocity }} E{{ extruder.max_extrude_only_velocity }}\n"
    "M201 X{{ toolhead.max_accel }} Y{{ toolhead.max_accel }} "
    "Z{{ config.printer.max_z_accel }} E{{ extruder.max_extrude_only_accel }}\n"
    "M206 X{{ gcode_move.homing_origin[0] }} "
         "Y{{ gcode_move.homing_origin[1] }} "
         "Z{{ gcode_move.homing_origin[2] }}\n"
    f"{PROBE_OFFSET_TEMPLATE}\n"
    "M420 S1 Z{{ config.bed_mesh.fade_end }}\n"
    "M106 S{{ fan.speed * 255 | int }}\n"
    "work:{min:{x:0,y:0,z:0},"
          "max:{x:{{ config.stepper_x.position_max }},"
               "y:{{ config.stepper_y.position_max }},"
               "z:{{ config.stepper_z.position_max }}}}"
)

FIRMWARE_INFO_TEMPLATE = (
    "FIRMWARE_NAME:{{ firmware_name }} "
    "SOURCE_CODE_URL:https://github.com/Klipper3d/klipper "
    "PROTOCOL_VERSION:1.0 "
    "MACHINE_TYPE:{{ machine_name }}\n"
    "Cap:EEPROM:1\n"
    "Cap:AUTOREPORT_TEMP:1\n"
    "Cap:AUTOREPORT_POS:1\n"
    "Cap:AUTOLEVEL:1\n"
    "Cap:Z_PROBE:1\n"
    "Cap:LEVELING_DATA:0\n"
    "Cap:SOFTWARE_POWER:0\n"
    "Cap:TOGGLE_LIGHTS:0\n"
    "Cap:CASE_LIGHT_BRIGHTNESS:0\n"
    "Cap:EMERGENCY_PARSER:1\n"
    "Cap:PROMPT_SUPPORT:0\n"
    "Cap:SDCARD:1\n"
    "Cap:MULTI_VOLUME:0\n"
    "Cap:AUTOREPORT_SD_STATUS:1\n"
    "Cap:LONG_FILENAME:1\n"
    "Cap:BABYSTEPPING:1\n"
    "Cap:BUILD_PERCENT:1\n"
    "Cap:CHAMBER_TEMPERATURE:0"
)

SOFTWARE_ENDSTOPS_TEMPLATE = (
    "Soft endstops: {{ state }}"
)

PROBE_TEST_TEMPLATE = (
    "Last query: {{ probe.last_query }}\n"
    "Last Z result: {{ probe.last_z_result }}"
)

POSITION_TEMPLATE = (
    "X:{{ gcode_move.position[0] | round(2) }} "
    "Y:{{ gcode_move.position[1] | round(2) }} "
    "Z:{{ gcode_move.position[2] | round(2) }} "
    "E:{{ gcode_move.position[3] | round(2) }}"
)

FEED_RATE_TEMPLATE = (
    "FR:{{ gcode_move.speed_factor * 100 | int }}%"
)
FLOW_RATE_TEMPLATE = (
    "E0 Flow:{{ gcode_move.extrude_factor * 100 | int }}%"
)

FILE_LIST_TEMPLATE = (
    "Begin file list\n"
    "{% for file, size in files %}{{ file }} {{ size }}\n{% endfor %}"
    "End file list\nok"
)

PROBE_ACCURACY_TEMPLATE = (
    "Mean: {{ avg_val }} Min: {{ min_val }} Max: {{ max_val }} Range: {{ range_val }}\n"
    "Standard Deviation: {{ stddev_val }}\n"
    "ok"
)

class SerialConnection:
    """Manages the serial connection to the TFT."""

    def __init__(self, config: ConfigHelper, tft: TFTAdapter) -> None:
        """Initialize the serial connection.

        Args:
            config: Configuration helper instance
            tft: TFT adapter instance
        """
        self.event_loop = config.get_server().get_event_loop()
        self.tft = tft
        self.port: str = config.get("serial")
        self.baud = config.getint("baud", 57600)
        self.serial: Optional[serial.Serial] = None
        self.file_descriptor: Optional[int] = None
        self.connected: bool = False
        self.attempting_connect: bool = True
        self.partial_input: bytes = b""

    def disconnect(self, reconnect: bool = False) -> None:
        """Disconnect the serial connection."""
        if self.connected:
            if self.file_descriptor is not None:
                self.event_loop.remove_reader(self.file_descriptor)
                self.file_descriptor = None
            self.connected = False
            if self.serial is not None:
                self.serial.close()
            self.serial = None
            self.partial_input = b""
            logging.info("TFT Disconnected")
        if reconnect and not self.attempting_connect:
            self.attempting_connect = True
            self.event_loop.delay_callback(1., self.connect)

    async def connect(self) -> None:
        """Attempt to establish a serial connection."""
        self.attempting_connect = True
        start_time = connect_time = time.time()
        while not self.connected:
            if connect_time > start_time + 30.:
                logging.info("Unable to connect, aborting")
                break
            logging.info("Attempting to connect to: %s", self.port)
            try:
                self.serial = serial.Serial(
                    self.port, self.baud, timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException):
                logging.exception("Unable to open port: %s", self.port)
                await asyncio.sleep(2.)
                connect_time += time.time()
                continue
            self.file_descriptor = self.serial.fileno()
            os.set_blocking(self.file_descriptor, False)
            self.event_loop.add_reader(self.file_descriptor, self.tft._handle_incoming)
            self.connected = True
            logging.info("TFT Connected")
        self.attempting_connect = False

    def _send_to_tft(self, message=None) -> None:
        """Write a response to the serial connection."""
        formatted_msg = message.replace("\n", "\\n")
        logging.info("write: %s", formatted_msg)
        byte_resp = (message + "\n").encode("utf-8")
        self.serial.write(byte_resp)

    def echo(self, message: str) -> None:
        """Send echo message to TFT."""
        self._send_to_tft(f"echo:{message}")

    def command(self, command: str) -> None:
        """Send command to TFT."""
        self._send_to_tft(command)

    def error(self, error: str) -> None:
        """Send error message to TFT."""
        self._send_to_tft(f"Error:{error}")

    def action(self, action: str) -> None:
        """Send action command to TFT."""
        self._send_to_tft(f"//action:{action}")

    def notification(self, notification: str) -> None:
        """Send notification to TFT."""
        self.action(f"notification {notification}")

class TFTAdapter:
    """Adapter for managing the TFT display."""

    def __init__(self, config: ConfigHelper) -> None:
        """Initialize the TFT adapter.

        Args:
            config: Configuration helper instance
        """
        self.values: Dict[str, Dict[str, Any]] = {}
        self.config: Dict[str, Any] = {}
        self.firmware_name: Optional[str] = None
        self.is_ready: bool = False
        self.is_busy: bool = False
        self.queue: List[Union[str, Tuple[FlexCallback, Any]]] = []
        self.last_printer_state: str = "O"
        self.machine_name = config.get("machine_name", "Klipper")
        self.filament_sensor: str = f"filament_switch_sensor {config.get('filament_sensor_name')}"

        # Tasks for periodic reporting
        self.temperature_report_task: Optional[asyncio.Task] = None
        self.position_report_task: Optional[asyncio.Task] = None
        self.print_status_report_task: Optional[asyncio.Task] = None

        self._init_components(config)
        self._register_server_events()
        self._init_command_handlers()

    def _init_components(self, config: ConfigHelper) -> None:
        """Initialize component dependencies."""
        self.server = config.get_server()
        self.event_loop = self.server.get_event_loop()
        self.file_manager: FMComp = self.server.lookup_component("file_manager")
        self.klippy_apis: APIComp = self.server.lookup_component("klippy_apis")
        self.ser_conn = SerialConnection(config, self)

    def _register_server_events(self) -> None:
        """Register server event handlers."""
        self.server.register_event_handler(
            "server:klippy_ready", self._process_klippy_ready)
        self.server.register_event_handler(
            "server:klippy_shutdown", self._process_klippy_shutdown)
        self.server.register_event_handler(
            "server:klippy_disconnect", self._process_klippy_disconnect)
        self.server.register_event_handler(
            "server:gcode_response", self.handle_gcode_response)

    def _init_command_handlers(self):
        # Initialize tracked state.
        logging.info("TFT Configured")

        # These commands are directly executued on the server and do not to
        # make a request to Klippy
        self.direct_gcodes: Dict[str, FlexCallback] = {
            "G26": self._send_ok_response, # Mesh Validation Pattern (G26 H240 B70 R99)
            "G29": self._send_ok_response, # Bed leveling (G29)
            "G30": self._probe_at_position, # Single Z-Probe (G30 E1 X28 Y207)
            "M20": self._list_sd_files,
            "M21": self._init_sd_card,
            "M23": self._select_sd_file,
            "M24": self._start_print,
            "M25": self._pause_print,
            "M27": self._set_print_status_autoreport,
            "M33": self._get_long_path,
            "M48": "PROBE_ACCURACY",
            "M81": self._power_off,
            "M82": self._send_ok_response, # E Absolute
            "M92": self._send_ok_response, # Set Axis Steps-per-unit
            "M105": self._report_temperature,
            "M108": "CANCEL_PRINT",        # Break and Continue
            "M114": self._report_position,
            "M115": self._report_firmware_info,
            "M118": self._serial_print,
            "M150": self._set_led,
            "M154": self._set_position_autoreport,
            "M155": self._set_temperature_autoreport,
            "M201": self._set_acceleration,
            "M203": self._set_velocity,
            "M206": self._set_gcode_offset,
            "M211": self._report_software_endstops,
            "M220": self._set_feed_rate,
            "M221": self._set_flow_rate,
            "M280": self._probe_command,
            "M290": self._set_babystep,
            "M303": self._pid_autotune,
            "M401": "BLTOUCH_DEBUG COMMAND=self_test",
            "M420": self._set_bed_leveling,
            "M500": self._z_offset_apply_probe,
            "M501": self._restore_settings,
            "M502": self._restore_settings,
            "M503": self._report_settings,
            "M524": "CANCEL_PRINT",
            "M701": self._load_filament,
            "M702": self._unload_filament,
            "M851": self._set_probe_offset,
            "M876": self._send_ok_response, # Handle Prompt Response
            "T0": self._send_ok_response,   # Select or Report Tool
        }

        self.standard_gcodes: List[str] = [
            "G0", "G1", "G28", "G90", "G91", "G92", "M84", "M104", "M106", "M140"
        ]

    def _handle_incoming(self) -> None:
        """Handle incoming data from the serial connection."""
        if self.ser_conn.file_descriptor is None:
            return
        try:
            data = os.read(self.ser_conn.file_descriptor, 4096)
        except os.error:
            return

        if not data:
            # possibly an error, disconnect
            self.ser_conn.disconnect(reconnect=True)
            logging.info("serial_display: No data received, disconnecting")
            return

        # Remove null bytes, separate into lines
        data = data.strip(b"\x00")
        lines = data.split(b"\n")
        lines[0] = self.ser_conn.partial_input + lines[0]
        self.ser_conn.partial_input = lines.pop()
        for line in lines:
            try:
                decoded_line = line.strip().decode("utf-8", "ignore")
                self.process_line(decoded_line)
            except ServerError:
                logging.exception("GCode Processing Error: %s", decoded_line)
                self.ser_conn.error(f"!! GCode Processing Error: {decoded_line}")
            except Exception:
                logging.exception("Error during gcode processing")

    async def component_init(self) -> None:
        """Initialize the component."""
        await self.ser_conn.connect()

    async def _process_klippy_ready(self) -> None:
        """Handle the event when Klippy is ready."""
        # Request "info" and "configfile" status
        retries = 10
        printer_info: Dict[str, Any] = {}
        cfg_status: Dict[str, Any] = {}
        while retries:
            try:
                printer_info = await self.klippy_apis.get_klippy_info()
                cfg_status = await self.klippy_apis.query_objects({"configfile": None})
            except self.server.error:
                logging.exception("TFT initialization request failed")
                retries -= 1
                if not retries:
                    raise
                await asyncio.sleep(1.)
                continue
            break

        self.firmware_name = "Marlin | Klipper " + printer_info["software_version"]
        self.config: Dict[str, Any] = cfg_status.get("configfile", {}).get("config", {})

        # Make subscription request
        sub_args: Dict[str, Optional[List[str]]] = {
            "gcode_move": None,
            "toolhead": None,
            "virtual_sdcard": None,
            "fan": None,
            "extruder": None,
            "heater_bed": None,
            "display_status": None,
            "print_stats": None,
            "probe": None,
            f"{self.filament_sensor}": None
        }
        try:
            self.values = await self.klippy_apis.query_objects(sub_args)
            self._print_status_change(self.values.get("print_stats", {}).get("state"), None)
            await self.klippy_apis.subscribe_objects(sub_args, self._subcription_updates)
            self.is_ready = True
            self.is_busy = False
        except self.server.error:
            logging.exception("Unable to complete subscription request")

    def _subcription_updates(self, data: Dict[str, Any], _: float) -> None:
        """Update printer state values."""
        for key, values in data.items():
            if key in self.values:
                self.values[key].update(values)
        filament_sensor = data.get(self.filament_sensor)
        state = data.get("print_stats", {}).get("state")
        if state:
            self._print_status_change(state, filament_sensor)
        display_status: Optional[int] = data.get("display_status")
        if data.get("display_status"):
            self._display_status_change(display_status)

    def _display_status_change(self, display_status: Dict[str, Any]) -> None:
        """Process display status changes."""
        message = display_status.get("message")
        if not message:
            return

        if re.match(r"^\d+/\d+ | ET ", message):
            current_layer, total_layer = re.search(r"(^\d+)/(\d+) | ET ", message).groups()
            self._queue_task(
                f"SET_PRINT_STATS_INFO CURRENT_LAYER={current_layer} TOTAL_LAYER={total_layer}")
        elif not message.startswith("ET "):
            self.ser_conn.notification(message)

    def _print_status_change(self,
                             state: Dict[str, Any],
                             filament_sensor: Dict[str, Any]) -> None:
        """Process subscription changes."""
        logging.debug("Current printer state: %s", self.last_printer_state)
        filament_detected = None
        if filament_sensor:
            filament_detected = filament_sensor["filament_detected"]
        if state == "printing":
            if self.last_printer_state == "paused":
                self.ser_conn.action("resume")
                if filament_detected:
                    self.ser_conn.action("prompt_end")
                    self.ser_conn.action("prompt_begin Continue?")
                    self.ser_conn.action("prompt_button Ok")
                    self.ser_conn.action("prompt_show")
            else:
                self.ser_conn.action("print_start")

                filename = self.values["print_stats"]["filename"]
                metadata = self.file_manager.get_file_metadata(filename)
                logging.info(f"metadata: {metadata}")
                estimated_time = metadata.get("estimated_time")
                layer_count = metadata.get("layer_count")
                hours = int(estimated_time // 3600)
                minutes = int((estimated_time % 3600) // 60)
                seconds = int(estimated_time % 60)
                self.ser_conn.notification(f"Time Left {hours:02}h{minutes:02}m{seconds:02}s")

                self._queue_task(
                    f"SET_PRINT_STATS_INFO CURRENT_LAYER=1 TOTAL_LAYER={layer_count}")
                self._report(f"{PRINT_STATUS_TEMPLATE}", **self.values)
        elif state == "paused":
            if filament_detected is False:
                self.ser_conn.action("paused filament_runout")
            else:
                self.ser_conn.action("paused")
        elif state == "cancelled":
            self.ser_conn.action("cancel")
        self.last_printer_state = state

    def _process_klippy_shutdown(self) -> None:
        """Handle the event when Klippy shuts down."""
        power: PrinterPower = self.server.lookup_component("power")
        power.set_device_power("printer", "off")

    def _process_klippy_disconnect(self) -> None:
        """Handle the event when Klippy disconnects."""
        # Tell the TFT that the printer is "off"
        self.ser_conn.command("Reset Software")
        self.is_ready = False
        self.last_printer_state = "O"

    def process_line(self, line: str) -> None:
        """Process an incoming line of G-code."""
        logging.info("read:  %s", line)
        # If we find M112 in the line then skip verification
        if "M112" in line:
            self.event_loop.register_callback(self.klippy_apis.emergency_stop)
            return

        parts = line.split()
        gcode = parts[0].strip()

        if gcode in self.standard_gcodes:
            self._queue_task(line)

        elif gcode in self.direct_gcodes:
            if isinstance(self.direct_gcodes[gcode], str):
                self._queue_task(self.direct_gcodes[gcode])
                return
            params: Dict[str, Any] = {}
            for part in parts[1:]:
                logging.debug("part: %s", part)
                if not re.match(r"^-?\d+(?:\.\d+)?$", part[1:]):
                    if not params.get("arg_string"):
                        params["arg_string"] = part
                    else:
                        params["arg_string"] = f"{params['arg_string']} {part}"
                    continue
                else:
                    arg = part[0].lower()
                    val = int(part[1:]) if re.match(r"^-?\d+$", part[1:]) else float(part[1:])
                    params[f"arg_{arg}"] = val
            logging.debug("params: %s", params)
            func = self.direct_gcodes[gcode]
            self._queue_task((func, params))
            return
        else:
            logging.warning("Unregistered command: %s", line)
            self._queue_task(line)

    def _queue_task(self, task: Union[str, List[str], Tuple[FlexCallback, Any]]) -> None:
        """Queue a task for execution."""
        if isinstance(task, (str, list)):
            self.queue.append(task)
        elif isinstance(task, tuple) and len(task) == 2:
            self.queue.append((task[0], task[1]))
        else:
            self.queue.append(task)
        if self.is_ready and not self.is_busy:
            self.is_busy = True
            self.event_loop.register_callback(self._process_queue)

    async def _process_queue(self) -> None:
        """Process the queued tasks."""
        self.is_busy = True
        while self.queue:
            item = self.queue.pop(0)
            logging.debug("Processing: %s", repr(item))
            if isinstance(item, (str, list)):
                await self._process_script(item)
            else:
                await self._process_command(item)
        self.is_busy = False

    async def _process_script(self, scripts: str) -> None:
        """Process a script task."""
        logging.debug("script: %s", scripts)
        if isinstance(scripts, str):
            scripts = [scripts]
        try:
            for script in scripts:
                if script in ["RESTART", "FIRMWARE_RESTART"]:
                    response = await self.klippy_apis.do_restart(script)
                else:
                    response = await self.klippy_apis.run_gcode(script)
            logging.debug("response script %s: %s" % (scripts, response))
            self.ser_conn.command(response)
        except self.server.error:
            msg = f"Error executing script {script}"
            logging.exception(msg)
            self.ser_conn.error(msg)

    async def _process_command(self, item: Tuple[FlexCallback, Any]) -> None:
        """Process a command task."""
        cmd, args = item
        try:
            ret = cmd(**args)
            if ret is not None:
                await ret
        except (ValueError, TypeError) as e:
            logging.exception("Error processing command: %s", str(e))
        except Exception as e:
            logging.exception("Unexpected error processing command: %s", str(e))

    def handle_gcode_response(self, response: str) -> None:
        """Handle the response from a G-code command."""
        if "// Sending" in response or ("B:" in response and "T0:" in response):
            return
        logging.debug("Received gcode response: %s", response)
        if "Klipper state" in response or response.startswith("!!"):
            if "not hot enough" in response:
                self.ser_conn.error(response[3:])
            else:
                logging.error("Error response: %s", response)
                return
        if response.startswith(("File opened:", "File selected", "ok")):
            self.ser_conn.command(response)
        elif response.startswith(("echo: Adjusted Print Time", "echo: ")):
            if re.match(r"^echo: \d+/\d+ | ET ", response):
                return
        elif response.startswith("// probe at "):
            match = re.search(r"probe at ([\d.-]+),([\d.-]+) is z=([\d.-]+)", response)
            if match:
                x, y, z = match.groups()
                self.ser_conn.command(f"Bed X:{x} Y:{y} Z:{z}")
        else:
            logging.debug("Unhandled response: %s", response)

    def _report(self, template, **data):
        """Send report to tft."""
        self.ser_conn.command(Template(template).render(**data))

    async def _autoreport(self, template, interval, **data):
        """Send periodic reports based on the specified template."""
        while self.ser_conn.connected and interval > 0:
            self._report(template, **data)
            await asyncio.sleep(interval)

    def _set_autoreport_interval(self, task, template, interval, **data) -> None:
        if task:
            task.done()
        if interval > 0:
            task = self.event_loop.create_task(self._autoreport(template, interval, **data))
        self.ser_conn.command("ok")

    def _set_temperature_autoreport(self, arg_s: int) -> None:
        """Set the interval for temperature reports."""
        self._set_autoreport_interval(self.temperature_report_task,
                                      f"ok {TEMPERATURE_TEMPLATE}",
                                      arg_s,
                                      **self.values)

    def _set_position_autoreport(self, arg_s: int) -> None:
        """Set the interval for position reports."""
        self._set_autoreport_interval(self.position_report_task,
                                      POSITION_TEMPLATE,
                                      arg_s,
                                      **self.values)

    def _set_print_status_autoreport(self, arg_s: Optional[int] = 3) -> None:
        """Set the interval for print status reports."""
        self._set_autoreport_interval(self.print_status_report_task,
                                      PRINT_STATUS_TEMPLATE,
                                      arg_s,
                                      **self.values)

    def _clean_filename(self, filename: str) -> str:
        """Clean up the filename by removing unnecessary parts."""
        # Remove quotes and whitespace
        filename.strip(" \"\t\n")
        # Remove drive number
        if filename.startswith("0:/"):
            filename = filename[3:]
        # Remove initial "gcodes" folder.  This is necessary
        # due to the HACK in the tft_M20 gcode.
        if filename.startswith("gcodes/"):
            filename = filename[6:]
        elif filename.startswith("/gcodes/"):
            filename = filename[7:]
        # Start with a "/" so the gcode parser can correctly
        # handle files that begin with digits or special chars
        if filename[0] != "/":
            filename = "/" + filename
        return filename

    def _select_sd_file(self, arg_string: str) -> None:
        """Select an SD file for printing."""
        self.values["print_stats"]["filename"] = self._clean_filename(arg_string)
        self._queue_task(f"M23 {self._clean_filename(arg_string)}")

    def _start_print(self) -> None:
        """Start printing the selected file."""
        selected_file = self.values["print_stats"]["filename"]
        sd_state = self.values.get("print_stats", {}).get("state", "standby")
        if sd_state == "paused":
            self._queue_task("RESUME")
        elif sd_state in ("standby", "cancelled"):
            self._queue_task(f"SDCARD_PRINT_FILE FILENAME=\"{selected_file}\"")
        else:
            self.ser_conn.error("Cannot start printing, printer is not in a stopped state")

    def _pause_print(self, **_: Any) -> None:
        """Pause the current print."""
        # TODO: handle P1
        sd_state = self.values.get("print_stats", {}).get("state", "standby")
        if sd_state == "printing":
            self._queue_task("PAUSE")
        else:
            self.ser_conn.error("Cannot pause, printer is not printing")

    def _print_file(self, args: List[str]) -> None:
        """Print the specified file."""
        filename = self._clean_filename(args[0])
        # Escape existing double quotes in the file name
        filename = filename.replace("\"", "\\\"")
        self._queue_task(f"SDCARD_PRINT_FILE FILENAME=\"{filename}\"")

    def _set_led(self, **args: Dict[int]) -> None:
        """Set the LED color and brightness."""
        # TODO: read led name from config
        red = args.get("arg_r", 0) / 255
        green = args.get("arg_u", 0) / 255
        blue = args.get("arg_b", 0) / 255
        white = args.get("arg_w", 0) / 255
        brightness = args.get("arg_p", 255) / 255
        self._queue_task(f"SET_LED LED=statusled "
                         f"RED={red * brightness:.3f} "
                         f"GREEN={green * brightness:.3f} "
                         f"BLUE={blue * brightness:.3f} "
                         f"WHITE={white * brightness:.3f} "
                         "TRANSMIT=1 SYNC=1")

    def _set_babystep(self, **args: Dict[float]) -> None:
        """Set the babystep offsets."""
        offsets = []
        if "arg_x" in args:
            offsets.append(f"X_ADJUST={args['arg_x']}")
        if "arg_y" in args:
            offsets.append(f"Y_ADJUST={args['arg_y']}")
        if "arg_z" in args:
            offsets.append(f"Z_ADJUST={args['arg_z']}")
        offset_str = " ".join(offsets)
        if "xyz" in self.values.get("toolhead").get("homed_axes"):
            offset_str = f"{offset_str} MOVE=1 MOVE_SPEED=10"
        self._queue_task(f"SET_GCODE_OFFSET {offset_str}")

    def _pid_autotune(self,
                      arg_e: Optional[int] = None,
                      arg_s: Optional[int] = None,
                      arg_u: Optional[int] = None,
                      **args: Dict[float]) -> None:
        """Initiates a process to determine the PID values."""
        heater = "heater_bed" if arg_e == -1 else "extruder"
        cmd = f"PID_CALIBRATE HEATER={heater} TARGET={arg_s}"
        if arg_u == 1:
            cmd = [cmd, "SAVE_CONFIG"]
        self._queue_task(cmd)

    def _set_bed_leveling(self,
                          arg_s: Optional[int] = None,
                          arg_z: Optional[int] = None,
                          **_: Any) -> None:
        """Set the bed leveling state."""
        if arg_s:
            if arg_s == 0:
                self._queue_task("BED_MESH_CLEAR")
            else:
                self._queue_task("BED_MESH_PROFILE LOAD=default")
        elif arg_z:
            # TODO: Not working
            self._queue_task([
                "BED_MESH_PROFILE LOAD=default",
                f"BED_MESH_OFFSET ZFADE={arg_z}"])
        else:
            # TODO: Falta implementar M420 V1 T1
            self.ser_conn.command("ok")

    def _power_off(self) -> None:
        """Power off printer."""
        cmd = [
            "CLEAR_PAUSE",
            "TURN_OFF_HEATERS",
            "M106 S0",   # Turn of extruder-fan
            "G92 E0",    # Reset Extruder
            "M220 S100", # Reset Speed factor override percentage to default (100%)
            "M221 S100", # Reset Extruder flow rate override percentage to default (100%)
            "M84"
        ]
        self._queue_task(cmd)

    def _init_sd_card(self) -> None:
        """Initialize the SD card."""
        self.ser_conn.command("SD card ok\nok")

    def _list_sd_files(self, **_: Any) -> None:
        """List the files on the SD card."""
        response_type = 2
        if response_type != 2:
            logging.info("Cannot process response type %s in M20", response_type)
            return
        path = "/"

        # Strip quotes if they exist
        path = path.strip("\"")

        # Path should come in as "0:/macros, or 0:/<gcode_folder>".  With
        # repetier compatibility enabled, the default folder is root,
        # ie. "0:/"
        if path.startswith("0:/"):
            path = path[2:]
        response: Dict[str, Any] = {"dir": path}
        response["files"] = []

        if path == "/":
            response["dir"] = "/gcodes"
            path = "gcodes"
        elif path.startswith("/gcodes"):
            path = path[1:]

        files = {"files": []}
        flist = self.file_manager.list_dir(path, simple_format=False)
        if flist:
            files["files"] = [(file["filename"], file["size"])
                            for file in flist.get("files", [])]
        self._report(FILE_LIST_TEMPLATE, **files)

    def _get_long_path(self, arg_string: str) -> None:
        """Get the full path of a file."""
        self.ser_conn.command(f"{arg_string}\nok")

    def _report_software_endstops(self) -> None:
        """Report the status of software endstops."""
        filament_sensor=self.values.get(self.filament_sensor, {})
        state = { "state": "On" if filament_sensor.get("enabled", False) else "Off"}
        self._report(f"{SOFTWARE_ENDSTOPS_TEMPLATE}\nok", **state)
        logging.info(f"state: {self.values.get('print_stats', {}).get('state')}")
        self._print_status_change(self.values.get("print_stats", {}).get("state"), None)

    def _report_settings(self, **_: Any) -> None:
        """Report the printer settings."""
        self._report(f"{REPORT_SETTINGS_TEMPLATE}\nok", **(
            self.values | {"config":self.config}))

    def _send_ok_response(self, **_: Any) -> None:
        """Send an "ok" response."""
        self.ser_conn.command("ok")

    def _serial_print(self,
                      arg_p: Optional[int] = None,
                      arg_a: Optional[int] = None,
                      arg_string: Optional[str] = None) -> None:
        """Send serial print message."""
        self.ser_conn.command("ok")
        if arg_p == 0 and arg_string != "action:cancel":
            if arg_a == 1:
                self.ser_conn.command(f"//{arg_string}")
            else:
                self.ser_conn.echo(f"{arg_string}\nok" if arg_string else "ok")

    def _set_acceleration(self,
                          arg_x: Optional[float] = None,
                          arg_y: Optional[float] = None,
                          **_: Any) -> None:
        """Set the acceleration limits."""
        self._queue_task(
            f"SET_VELOCITY_LIMIT ACCEL={arg_x or arg_x} ACCEL_TO_DECEL={(arg_x or arg_x) / 2}"
        )

    def _set_velocity(self,
                      arg_x: Optional[float] = None,
                      arg_y: Optional[float] = None,
                      **_: Any) -> None:
        """Set the velocity limits."""
        self._queue_task(f"SET_VELOCITY_LIMIT VELOCITY={arg_x or arg_y}")

    def _set_gcode_offset(self, **args: Dict[float]) -> None:
        """Set the G-code offsets."""
        offsets = []
        if "arg_x" in args:
            offsets.append(f"X={args['arg_x']}")
        if "arg_y" in args:
            offsets.append(f"Y={args['arg_y']}")
        if "arg_z" in args:
            offsets.append(f"Z={args['arg_z']}")
        offset_str = " ".join(offsets)
        self._queue_task(f"SET_GCODE_OFFSET {offset_str}")

    def _set_probe_offset(self, **args: Dict[float]) -> None:
        """Set the probe offsets."""
        if not args:
            self._report(PROBE_OFFSET_TEMPLATE, **(self.values | {"config":self.config}))
        self.ser_conn.command("ok")

    def _load_filament(self) -> None:
        """Load filament into the extruder."""
        params = {
            "length": 25,
            "extruder": 0,
            "zmove": 0
        }
        self._handle_filament(params)

    def _unload_filament(self) -> None:
        """Unload filament from the extruder."""
        params = {
            "length": -25,
            "extruder": 0,
            "zmove": 0
        }
        self._handle_filament(params)

    def _handle_filament(self, args: Dict[str, Any]) -> None:
        """Handle filament loading and unloading."""
        cmd = [
            "G91",                                                     # Relative Positioning
            f"G92 E{args.get('extruder')}",                            # Reset Extruder
            f"G1 Z{args.get('zmove')} E{args.get('length')} F{3*60}",  # Extrude or Retract
            "G92 E0"                                                   # Reset Extruder
        ]
        self._queue_task(cmd)

    def close(self) -> None:
        """Close the TFT adapter and disconnect."""
        self.ser_conn.disconnect()
        if self.temperature_report_task:
            self.temperature_report_task.cancel()
        if self.position_report_task:
            self.position_report_task.cancel()
        if self.print_status_report_task:
            self.print_status_report_task.cancel()

    def _set_feed_rate(self, arg_s: Optional[int] = None, **_: Any) -> None:
        """Set the feed rate."""
        if arg_s is not None:
            self._queue_task(f"M220 S{arg_s}")
        else:
            self._report(f"{FEED_RATE_TEMPLATE}\nok", **self.values)

    def _set_flow_rate(self, arg_s: Optional[int] = None, **_: Any) -> None:
        """Set the flow rate."""
        if arg_s is not None:
            self._queue_task(f"M221 S{arg_s}")
        else:
            self._report(f"{FLOW_RATE_TEMPLATE}\nok", **self.values)

    def _report_temperature(self) -> None:
        """Report the current temperature."""
        self._report(f"{TEMPERATURE_TEMPLATE}\nok", **self.values)

    def _report_position(self) -> None:
        """Report the current position."""
        self._report(f"{POSITION_TEMPLATE}\nok", **self.values)

    def _report_firmware_info(self) -> None:
        """Report the firmware information."""
        self._report(FIRMWARE_INFO_TEMPLATE, **(
            self.values |
            { "machine_name": self.machine_name } |
            { "firmware_name": self.firmware_name }))

    def _z_offset_apply_probe(self) -> List[str]:
        """Apply the Z offset from the probe."""
        sd_state = self.values.get("print_stats", {}).get("state", "standby")
        if sd_state in ("printing", "paused"):
            self.ser_conn.error("Not saved - Printing")
        else:
            self._queue_task(["Z_OFFSET_APPLY_PROBE", "SAVE_CONFIG"])

    def _restore_settings(self) -> List[str]:
        """Restore settings from file."""
        sd_state = self.values.get("print_stats", {}).get("state", "standby")
        if sd_state in ("printing", "paused"):
            self.ser_conn.error("Not saved - Printing")
        else:
            self._queue_task("RESTART")

    def _probe_command(self, arg_s: int, **_: Any) -> None:
        """Handle probe commands."""
        if arg_s == 120:  # Test
            cmd = "QUERY_PROBE"
        else:
            if self.config.get("bltouch"):
                value = {10: "pin_down", 90: "pin_up", 160: "reset"}.get(arg_s)
                cmd = f"BLTOUCH_DEBUG COMMAND={value}"
            else:
                value = {10: "1", 90: "0", 160: "0"}.get(arg_s)
                cmd = f"SET_PIN PIN=_probe_enable VALUE={value}"
        self._queue_task(cmd)

    def _probe_at_position(self, arg_x: int, arg_y: int, **_: Any) -> None:
        """Handle probe commands."""
        cmd = [
            f"G1 X{arg_x} Y{arg_y}",
            "PROBE",
            "G1 Z10"
        ]
        self._queue_task(cmd)

def load_component(config: ConfigHelper) -> TFTAdapter:
    """Load the TFT adapter component."""
    return TFTAdapter(config)
