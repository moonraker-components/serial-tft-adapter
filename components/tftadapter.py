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
import math
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

from ..utils import ServerError

# Annotation imports
if TYPE_CHECKING:
    from .power import PrinterPower
    from ..confighelper import ConfigHelper
    from .klippy_apis import KlippyAPI
    from .file_manager.file_manager import FileManager
    from .database import MoonrakerDatabase
    FlexCallback = Callable[..., Optional[Coroutine]]

PRINT_STATUS_TEMPLATE = (
    "//action:notification Layer Left "
    "{{ (current_layer or 0) }}/{{ (total_layer or 0) }}\n"
    "//action:notification Data Left "
    "{{ (virtual_sdcard.file_position or 0) }}/{{ (virtual_sdcard.file_size or 0) }}\n"
    "M106 S{{ fan.speed * 255 | int }}"
)

TEMPERATURE_TEMPLATE = (
    "T:{{ extruder.temperature | int }} /{{ extruder.target | int }} "
    "B:{{ heater_bed.temperature | int }} /{{ heater_bed.target | int }} "
    "@:0 B@:0"
)

PROBE_OFFSET_TEMPLATE = (
    "M851 X{{ printer_cfg.bltouch.x_offset | float - gcode_move.homing_origin[0] }} "
    "Y{{ printer_cfg.bltouch.y_offset | float - gcode_move.homing_origin[1] }} "
    "Z{{ printer_cfg.bltouch.z_offset | float - gcode_move.homing_origin[2] }}"
)

REPORT_SETTINGS_TEMPLATE = (
    "M203 X{{ toolhead.max_velocity }} Y{{ toolhead.max_velocity }} "
    "Z{{ printer_cfg.printer.max_z_velocity }} E{{ extruder.max_extrude_only_velocity }}\n"
    "M201 X{{ toolhead.max_accel }} Y{{ toolhead.max_accel }} "
    "Z{{ printer_cfg.printer.max_z_accel }} E{{ extruder.max_extrude_only_accel }}\n"
    "M206 X{{ gcode_move.homing_origin[0] }} "
         "Y{{ gcode_move.homing_origin[1] }} "
         "Z{{ gcode_move.homing_origin[2] }}\n"
    f"{PROBE_OFFSET_TEMPLATE}\n"
    "M420 S1 Z{{ printer_cfg.bed_mesh.fade_end }}\n"
    "M106 S{{ fan.speed * 255 | int }}\n"
    "work:{min:{x:0,y:0,z:0},"
          "max:{x:{{ printer_cfg.stepper_x.position_max }},"
               "y:{{ printer_cfg.stepper_y.position_max }},"
               "z:{{ printer_cfg.stepper_z.position_max }}}}"
)

FIRMWARE_INFO_TEMPLATE = (
    "FIRMWARE_NAME:{{ firmware_name }} "
    "SOURCE_CODE_URL:https://github.com/Klipper3d/klipper "
    "PROTOCOL_VERSION:1.0 "
    "MACHINE_TYPE:{{ machine_name }}\n"
    "Auto Bed Leveling\n"
    "Cap:EXTRUDER_COUNT:1\n"
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

POSITION_TEMPLATE = (
    "X:{{ gcode_move.position[0] | round(2) }} "
    "Y:{{ gcode_move.position[1] | round(2) }} "
    "Z:{{ gcode_move.position[2] | round(2) }} "
    "E:{{ gcode_move.position[3] | round(2) }}"
)

PROBE_ACCURACY_TEMPLATE = (
    "Mean: {{ avg_val }} Min: {{ min_val }} Max: {{ max_val }} Range: {{ range_val }}\n"
    "Standard Deviation: {{ stddev_val }}\n"
    "ok"
)

class SerialConnection:
    """Manages the serial connection to the TFT."""

    def __init__(self, config: ConfigHelper, tft: TFTAdapter) -> None:
        """Initialize the serial connection."""
        self.event_loop = config.get_server().get_event_loop()
        self.tft = tft
        self.port: str = config.get("serial")
        self.baud = config.getint("baud", 57600)
        self.serial: Optional[serial.Serial] = None
        self.file_descriptor: Optional[int] = None
        self.connected = False
        self.attempting_connect = True
        self.partial_input: bytes = b""

    async def connect(self) -> None:
        """Attempt to establish a serial connection."""
        self.attempting_connect = True
        start_time = time.time()
        connect_time = start_time

        while not self.connected:
            if connect_time > start_time + 30:
                logging.info("Unable to connect, aborting")
                break

            logging.info("Attempting to connect to: %s", self.port)
            try:
                self.serial = serial.Serial(
                    self.port, self.baud, timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException):
                logging.exception("Unable to open port: %s", self.port)
                await asyncio.sleep(2.)
                connect_time = time.time()
                continue

            self.file_descriptor = self.serial.fileno()
            os.set_blocking(self.file_descriptor, False)
            self.event_loop.add_reader(
                self.file_descriptor,
                self.tft.handle_incoming
            )
            self.connected = True
            logging.info("TFT Connected")

        self.attempting_connect = False

    def send_to_tft(self, message: str) -> None:
        """Write a response to the serial connection."""
        formatted_msg = message.replace("\n", "\\n")
        logging.info("write: %s", formatted_msg)
        byte_resp = (message + "\n").encode("utf-8")
        self.serial.write(byte_resp)

    def echo(self, message: str) -> None:
        """Send echo message to TFT."""
        self.send_to_tft(f"echo:{message}")

    def command(self, command: str) -> None:
        """Send command to TFT."""
        self.send_to_tft(command)

    def error(self, error: str) -> None:
        """Send error message to TFT."""
        self.send_to_tft(f"Error:{error}")

    def action(self, action: str) -> None:
        """Send action command to TFT."""
        self.send_to_tft(f"//action:{action}")

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
        # Initialize all attributes in __init__
        self.server = config.get_server()
        self.event_loop = self.server.get_event_loop()
        self.file_manager: FileManager = self.server.lookup_component("file_manager")
        self.klippy_apis: KlippyAPI = self.server.lookup_component("klippy_apis")

        # Basic state
        self.object_status: Dict[str, Dict[str, Any]] = {}
        self.printer_cfg: Dict[str, Any] = {}
        self.is_ready: bool = False
        self.is_busy: bool = False
        self.queue: List[Union[str, Tuple[FlexCallback, Any]]] = []
        self.last_printer_state: str = None

        db: MoonrakerDatabase = self.server.lookup_component("database")
        sync_provider = db.get_provider_wrapper()
        mainsail_info: Dict[str, Any]
        mainsail_info = sync_provider.get_item("mainsail", "general", {})

        # Configuration values
        self.printer_info: Dict[str, Any] = {
            "machine_name": mainsail_info.get("printername", "Klipper"),
            "led_config": None,
            "filament_sensor": None
        }

        # Report tasks
        self.temperature_report_task: Optional[asyncio.Task] = None
        self.position_report_task: Optional[asyncio.Task] = None
        self.print_status_report_task: Optional[asyncio.Task] = None

        # Serial connection
        self.ser_conn = SerialConnection(config, self)

        # Initialize command handlers
        self.direct_gcodes: Dict[str, FlexCallback] = {
            "G26": self._mesh_validation,  # Mesh Validation Pattern
            "G29": ["BED_MESH_CALIBRATE PROFILE=default", "SAVE_CONFIG"],
            "G30": self._probe_at_position,
            "M20": self._list_sd_files,
            "M21": self._init_sd_card,
            "M23": self._select_sd_file,
            "M24": self._start_print,
            "M25": self._pause_print,
            "M27": self._set_print_status_autoreport,
            "M33": self._get_long_path,
            "M48": "PROBE_ACCURACY",
            "M81": self._power_off,
            "M82": self._send_ok_response,  # E Absolute
            "M92": self._send_ok_response,  # Set Axis Steps-per-unit
            "M105": self._report_temperature,
            "M108": "CANCEL_PRINT",         # Break and Continue
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
            "M876": self._send_ok_response,  # Handle Prompt Response
            "T0": self._send_ok_response,    # Select or Report Tool
        }

        self.standard_gcodes: List[str] = ["G0", "G1", "G28", "G90", "G91", "G92",
                                           "M84", "M104", "M106", "M140"]

        self._register_server_events()

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

    def handle_incoming(self) -> None:
        """Handle incoming data from the serial connection."""
        if self.ser_conn.file_descriptor is None:
            return
        try:
            data = os.read(self.ser_conn.file_descriptor, 4096)
        except os.error:
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
        cfg_status: Dict[str, Any] = {}
        while retries:
            try:
                objects = await self.klippy_apis.get_object_list(default=[])
                # Log the full object name for neopixel and filament_switch_sensor
                for object in objects:
                    if "neopixel" in object:
                        self.printer_info["led_config"] = object.split()[1]
                    if "filament_switch_sensor" in object:
                        self.printer_info["filament_sensor"] = object
                klippy_info = await self.klippy_apis.get_klippy_info()
                logging.info("Klippy Info: %s", klippy_info)
                self.printer_info.update(
                    {"firmware_name": f"Marlin | Klipper {klippy_info.get('software_version')}"})
                cfg_status = await self.klippy_apis.query_objects({"configfile": None})
            except self.server.error:
                logging.exception("TFT initialization request failed")
                retries -= 1
                if not retries:
                    raise
                await asyncio.sleep(1.)
                continue
            break
        self.printer_cfg: Dict[str, Any] = cfg_status.get("configfile", {}).get("config", {})

        # Make subscription request
        sub_args: Dict[str, Optional[List[str]]] = {
            "gcode_move": None,
            "toolhead": None,
            "virtual_sdcard": None,
            "fan": None,
            "extruder": None,
            "bed_mesh": None,
            "heater_bed": None,
            "display_status": None,
            "print_stats": None,
            "probe": None
        }
        filament_sensor = self.printer_info.get('filament_sensor')
        if filament_sensor:
            sub_args[filament_sensor] = None
        try:
            self.object_status = await self.klippy_apis.query_objects(sub_args)
            await self.klippy_apis.subscribe_objects(sub_args, self._subcription_updates)
            self.is_ready = True
            self.is_busy = False
        except self.server.error:
            logging.exception("Unable to complete subscription request")

    def _subcription_updates(self, data_update: Dict[str, Any], _: float) -> None:
        """Update printer states values."""
        for key, values in data_update.items():
            if key in self.object_status:
                self.object_status[key].update(values)
        if data_update.get("bed_mesh") is not None:
            self._bed_mesh_change()
        printer_state = data_update.get("print_stats", {}).get("state")
        if printer_state is not None:
            self._print_status_change(printer_state,
                                      self.object_status.get(self.printer_info.get("filament_sensor"), None))
        display_status = data_update.get("display_status")
        if data_update.get("display_status") is not None:
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

    def _bed_mesh_change(self) -> None:
        """Process bed mesh changes."""
        bed_mesh = self.object_status.get("bed_mesh")
        bed_leveling = "ON" if bed_mesh.get("profile_name") != "" else "OFF"
        self.ser_conn.echo(f"Bed Leveling {bed_leveling}")

    def _print_status_change(self,
                             printer_state: Dict[str, Any],
                             filament_state: Dict[str, Any]) -> None:
        """Process print status changes."""
        logging.info("Previous printer state: %s", self.last_printer_state)
        filament_detected = None
        if filament_state:
            filament_detected = filament_state["filament_detected"]
        logging.info("filament_state: %s", filament_state)
        logging.info("filament_detected: %s", filament_detected)
        if printer_state == "printing":
            if self.last_printer_state == "paused":
                self.ser_conn.action("resume")
                if filament_detected:
                    self.ser_conn.action("prompt_end")
                    self.ser_conn.action("prompt_begin Continue?")
                    self.ser_conn.action("prompt_button Ok")
                    self.ser_conn.action("prompt_show")
            else:
                self.ser_conn.action("print_start")
                filename = self.object_status["print_stats"]["filename"]
                metadata = self.file_manager.get_file_metadata(filename)
                estimated_time = metadata.get("estimated_time")
                if estimated_time:
                    hours = int(estimated_time // 3600)
                    minutes = int((estimated_time % 3600) // 60)
                    seconds = int(estimated_time % 60)
                    self.ser_conn.notification(f"Time Left {hours:02}h{minutes:02}m{seconds:02}s")
        elif printer_state == "paused":
            self.ser_conn.action("paused" if filament_detected else "paused filament_runout")
        if printer_state in ("cancelled", "complete", "standby"):
            self.ser_conn.action("cancel" if printer_state == "cancelled" else "print_end")
            if self.print_status_report_task:
                self.print_status_report_task.cancel()
        self.last_printer_state = printer_state

    def _process_klippy_shutdown(self) -> None:
        """Handle the event when Klippy shuts down."""
        power: PrinterPower = self.server.lookup_component("power")
        power.set_device_power("printer", "off")

    def _process_klippy_disconnect(self) -> None:
        """Handle the event when Klippy disconnects."""
        # Tell the TFT that the printer is "off"
        self.ser_conn.command("Reset Software")
        logging.info("Stopping autoreports")
        if self.temperature_report_task:
            self.temperature_report_task.cancel()
        if self.position_report_task:
            self.position_report_task.cancel()
        if self.print_status_report_task:
            self.print_status_report_task.cancel()
        self.is_ready = False
        self.last_printer_state = None

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
            if isinstance(self.direct_gcodes[gcode], (str, list)):
                self._queue_task(self.direct_gcodes[gcode])
                return
            params: Dict[str, Any] = {}
            for part in parts[1:]:
                logging.debug("part: %s", part)
                if not re.match(r"^-?\d+(?:\.\d+)?$", part[1:]):
                    args = params.get("arg_string")
                    params["arg_string"] = part if args is None else f"{args} {part}"
                else:
                    params[f"arg_{part[0].lower()}"] = (int(part[1:])
                                                        if re.match(r"^-?\d+$", part[1:])
                                                        else float(part[1:]))
            logging.debug("params: %s", params)
            func = self.direct_gcodes[gcode]
            if "M150" in gcode:
                logging.info("params: %s", params)
                self.event_loop.register_callback(self._set_led, **params)
                self.ser_conn.command("ok")
            else:
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
            if isinstance(item, (str, list)):
                await self._process_script(item)
            else:
                await self._process_command(item)
        self.is_busy = False

    async def _process_script(self, scripts: str) -> None:
        """Process a script task."""
        if isinstance(scripts, str):
            scripts = [scripts]
        try:
            for script in scripts:
                logging.info("Processing gcode: %s", script)
                if script in ["RESTART", "FIRMWARE_RESTART"]:
                    response = await self.klippy_apis.do_restart(script)
                else:
                    response = await self.klippy_apis.run_gcode(script)
                    logging.debug("response script %s: %s", scripts, response)
            self.ser_conn.command(response)
        except self.server.error:
            msg = f"Error executing script {script}"
            logging.exception(msg)
            # self.ser_conn.error(msg)

    async def _process_command(self, item: Tuple[FlexCallback, Any]) -> None:
        """Process a command task."""
        cmd, args = item
        logging.info("Processing command: %s(args = %s)", cmd.__name__, args)
        try:
            ret = cmd(**args)
            if ret is not None:
                await ret
        except (ValueError, TypeError) as e:
            logging.exception("Error processing command: %s", str(e))
        except (asyncio.CancelledError, asyncio.TimeoutError) as e:
            logging.exception("Async operation error: %s", str(e))
        except OSError as e:
            logging.exception("System error during command execution: %s", str(e))
        except Exception as e:
            logging.exception("Critical error in command execution: %s ", str(e))

    def handle_gcode_response(self, response: str) -> None:
        """Handle the response from a G-code command."""
        if "// Sending" in response or ("B:" in response and "T0:" in response):
            return
        logging.info("Received gcode response: %s", response)
        if "Klipper state" in response or response.startswith("!!"):
            if ("not hot enough" in response or
                "Must home" in response or
                "Move exceeds" in response):
                self.ser_conn.error(response[3:])
            else:
                logging.error("Error response: %s", response)
                return
        if response.startswith(("File opened:", "File selected", "ok")):
            self.ser_conn.command(response)
        elif response.startswith(("echo: Adjusted Print Time", "echo: ")):
            if re.match(r"^echo: \d+/\d+ | ET ", response):
                return
        elif "probe: open" in response:
            self._report("Last query: {{ probe.last_query }}\n"
                         "Last Z result: {{ probe.last_z_result }}\nok", **self.object_status)
        elif "probe accuracy results:" in response:
            parts = response[3:].split(",")
            data = {"max_val": parts[0].split()[-1],
                    "min_val": parts[1].split()[-1],
                    "range_val": parts[2].split()[-1],
                    "avg_val": parts[3].split()[-1],
                    "stddev_val": parts[5].split()[-1]}
            self._report(f"{PROBE_ACCURACY_TEMPLATE}\nok", **data)
        elif response.startswith("// probe at"):
            match = re.search(r"probe at ([\d.-]+),([\d.-]+) is z=([\d.-]+)", response)
            x, y, z = match.groups()
            self.ser_conn.command(f"Bed X:{x} Y:{y} Z:{z}")
        else:
            logging.debug("Unhandled response: %s", response)

    def _report(self, template, **data):
        """Send report to tft."""
        self.ser_conn.command(Template(template).render(**data))

    async def _autoreport(self, template, interval, data_callback):
        """Send periodic reports based on the specified template."""
        while self.ser_conn.connected and interval > 0:
            data = data_callback()
            if data:
                self._report(template, **data)
            await asyncio.sleep(interval)

    def _get_object_status(self):
        return self.object_status

    def _set_autoreport(self, task_attr, template, interval, data_callback=None):
        """Set up an autoreporting task."""
        if data_callback is None:
            data_callback = self._get_object_status

        task = getattr(self, task_attr, None)
        if task:
            task.cancel()

        if interval > 0:
            task = self.event_loop.create_task(self._autoreport(template, interval, data_callback))

        setattr(self, task_attr, task)
        self.ser_conn.command("ok")

    def _set_temperature_autoreport(self, arg_s: int) -> None:
        """Set the interval for temperature reports."""
        self._set_autoreport("temperature_report_task", f"ok {TEMPERATURE_TEMPLATE}", arg_s)

    def _set_position_autoreport(self, arg_s: int) -> None:
        """Set the interval for position reports."""
        self._set_autoreport("position_report_task", POSITION_TEMPLATE, arg_s)

    def _set_print_status_autoreport(self, arg_s: Optional[int] = 3) -> None:
        """Set up automatic reporting of the print status."""
        self._set_autoreport("print_status_report_task", PRINT_STATUS_TEMPLATE, arg_s,
                             self._get_layer_info)

    def _clean_filename(self, filename: str) -> str:
        """Clean up the filename by removing unnecessary parts."""
        logging.info("original filename: %s", filename)
        # Remove quotes and whitespace
        filename.strip(" \"\t\n")
        # if filename.startswith("/"):
        #     filename = filename[1:]
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
        file_to_print = arg_string.replace('[:space:]', ' ')
        file_size = self.file_manager.get_file_metadata(file_to_print).get("size")
        self.object_status["print_stats"]["filename"] = self._clean_filename(file_to_print)
        self.ser_conn.command(f"File opened:{file_to_print} Size:{file_size}\nFile selected\nok")

    def _get_layer_info(self) -> tuple[int, int]:
        """Calculate max layers and current layer in the print."""
        print_stats = self.object_status.get("print_stats", {})
        metadata = self.file_manager.get_file_metadata(print_stats.get("filename", ""))

        total_layer = print_stats.get("info", {}).get("total_layer") or metadata.get("layer_count")
        first_layer_height = metadata.get("first_layer_height")
        layer_height = metadata.get("layer_height")
        object_height = metadata.get("object_height")

        if (total_layer is None and
            all(v is not None for v in [first_layer_height, layer_height, object_height])):
            total_layer = math.ceil((object_height - first_layer_height) / layer_height + 1) or 0

        current_layer = (print_stats.get("info", {}).get("current_layer")
                         if print_stats.get("print_duration", 0) > 0 else 0)
        if current_layer is None and first_layer_height is not None and layer_height is not None:
            current_layer = math.ceil(
                (self.object_status.get("gcode_move", {}).get("gcode_position", [0, 0, 0])[2]
                - first_layer_height) / layer_height + 1) or 0

        return (self.object_status |
                {"current_layer": min(current_layer, total_layer), "total_layer": total_layer})

    def _start_print(self) -> None:
        """Start printing the selected file."""
        selected_file = self.object_status["print_stats"]["filename"]
        sd_state = self.object_status.get("print_stats", {}).get("state", "standby")
        if sd_state == "paused":
            self._queue_task("RESUME")
        elif sd_state in ("standby", "cancelled", "complete"):
            self._queue_task(f"SDCARD_PRINT_FILE FILENAME=\"{selected_file}\"")
        else:
            self.ser_conn.error("Cannot start printing, printer is not in a stopped state")

    def _pause_print(self, **_: Any) -> None:
        """Pause the current print."""
        # TODO: handle P1
        sd_state = self.object_status.get("print_stats", {}).get("state", "standby")
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
        if self.printer_info.get("led_config") is None:
            logging.warning("LED configuration name not set, skipping LED command")
            return
        red = args.get("arg_r", 0) / 255
        green = args.get("arg_u", 0) / 255
        blue = args.get("arg_b", 0) / 255
        white = args.get("arg_w", 0) / 255
        brightness = args.get("arg_p", 255) / 255
        self._queue_task(f"SET_LED LED={self.printer_info.get('led_config')} "
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
        if "xyz" in self.object_status.get("toolhead").get("homed_axes"):
            offset_str = f"{offset_str} MOVE=1 MOVE_SPEED=10"
        self._queue_task(f"SET_GCODE_OFFSET {offset_str}")

    def _pid_autotune(self,
                      **args: Dict[float]) -> None:
        """Initiates a process to determine the PID values."""
        heater = "heater_bed" if args.get("arg_e") == -1 else "extruder"
        cmd = [f"PID_CALIBRATE HEATER={heater} TARGET={args.get('arg_s')}"]
        if args.get("arg_u") == 1:
            cmd.extend("SAVE_CONFIG")
        self._queue_task(cmd)

    def _mesh_validation(self,
                         **args: Dict[int]) -> None:
        """Print Validation Pattern."""
        start = 10
        width, height = self.object_status.get('toolhead').get('axis_maximum')[:2]
        cmd = ["M82", # absolute extrusion mode"
               f"M140 S{args.get('arg_b')}",
               f"M104 S{args.get('arg_h')} T0",
               "G92 E0", # reset extruder
               "G1 F3000 Z1.0", # move z up little to prevent scratching of surface
               f"G1 F5000.0 X{start} Y{height - start} Z0.2", # move to start-line position
               f"M190 S{args.get('arg_b')}",
               f"M109 S{args.get('arg_h')} T0",
               "M107",
               "G1 F1500 E2",
               f"G1 F1500 X{start} Y{start} E10",
               f"G1 F1500 X{width - start} Y{start} E20",
               f"G1 F1500 X{width - start} Y{height - start} E30",
               f"G1 F1500 X{start} Y{height - start} E40",
               f"G1 F5000 X{start} Y{height - start} Z10"]
        self._queue_task(cmd)
        self.process_line("M81")

    def _set_bed_leveling(self,
                          **args: Dict[int]) -> None:
        """Set the bed leveling state."""
        if args.get("arg_s") is not None:
            if args.get("arg_s") == 0:
                self._queue_task("BED_MESH_CLEAR")
            else:
                self._queue_task("BED_MESH_PROFILE LOAD=default")
        elif args.get("arg_z") is not None:
            # TODO: Not working
            self._queue_task([
                "BED_MESH_PROFILE LOAD=default",
                f"BED_MESH_OFFSET ZFADE={args['arg_z']}"])
        elif args.get("arg_v") == 1 and args.get("arg_t") == 1:
            mesh = self.object_status["bed_mesh"]
            points = mesh.get("profiles").get("default").get("points")
            logging.debug("mesh: %s", self.object_status["bed_mesh"])
            self.ser_conn.command("Bilinear Leveling Grid:\n"
                "----- 0------ 1------ 2------ 3------ 4\n"
                f"0 {points[0][0]} {points[0][1]} {points[0][2]} {points[0][3]} {points[0][4]}\n"
                f"1 {points[1][0]} {points[1][1]} {points[1][2]} {points[1][3]} {points[1][4]}\n"
                f"2 {points[2][0]} {points[2][1]} {points[2][2]} {points[2][3]} {points[2][4]}\n"
                f"3 {points[3][0]} {points[3][1]} {points[3][2]} {points[3][3]} {points[3][4]}\n"
                f"4 {points[4][0]} {points[4][1]} {points[4][2]} {points[4][3]} {points[4][4]}\n")
        else:
            # TODO: Falta implementar M420 sin parametros
            self._bed_mesh_change()
            self.ser_conn.command("ok")

    def _power_off(self) -> None:
        """Power off printer."""
        self._queue_task(["CLEAR_PAUSE",
                          "TURN_OFF_HEATERS",
                          "M106 S0",   # Turn of extruder-fan
                          "G92 E0",    # Reset Extruder
                          "M220 S100", # Reset Speed factor override percentage
                          "M221 S100", # Reset Extruder flow rate override percentage
                          "M84"])

    def _init_sd_card(self) -> None:
        """Initialize the SD card."""
        self.ser_conn.command("SD card ok\nok")

    def _list_sd_files(self, **_: Any) -> None:
        """List the files and directories on the SD card recursively."""
        def scan_directory(path: str) -> Dict[str, List[Tuple[str, int, float]]]:
            """Recursively scan the directory for files."""
            try:
                contents = self.file_manager.list_dir(path, simple_format=False)
                if not contents:
                    return {"files": [], "dirs": []}

                files = [
                    (file["filename"], file["size"], file["modified"])
                    for file in contents.get("files", [])
                    if not file["filename"].startswith(".")
                ]
                dirs = [
                    dir["dirname"] for dir in contents.get("dirs", [])
                    if not dir["dirname"].startswith(".")
                ]

                for dir_name in dirs:
                    subpath = os.path.join(path, dir_name)

                return {"files": files, "dirs": dirs}
            except Exception as e:
                logging.exception(f"Error scanning directory {path}: {e}")
                return {"files": [], "dirs": []}

        root_path = "gcodes"  # Default search directory
        file_structure = scan_directory(root_path)

        # Sort files by modified timestamp (descending order)
        sorted_files = sorted(file_structure["files"], key=lambda f: f[2], reverse=True)

        self._report(
            "Begin file list\n"
            "{% for file, size, _ in files %}{{ file | replace(' ','[:space:]') }} {{ size }} {{ file }}\n{% endfor %}"
            "End file list\nok",
            files=sorted_files
        )

    def _get_long_path(self, arg_string: str) -> None:
        """Get the full path of a file."""
        self.ser_conn.command(f"{arg_string}\nok")

    def _report_software_endstops(self) -> None:
        """Report the status of software endstops."""
        filament_state = self.object_status.get(self.printer_info.get("filament_sensor"), None)
        state = {"state": "On" if filament_state and filament_state.get("enabled", False) else "Off"}
        self._report("Soft endstops: {{ state }}\nok", **state)
        self._print_status_change(self.object_status.get("print_stats", {}).get("state"), filament_state)

    def _report_settings(self, **_: Any) -> None:
        """Report the printer settings."""
        self._report(f"{REPORT_SETTINGS_TEMPLATE}\nok", **(
            self.object_status | {"printer_cfg":self.printer_cfg}))

    def _send_ok_response(self, **_: Any) -> None:
        """Send an "ok" response."""
        self.ser_conn.command("ok")

    def _serial_print(self,
                      arg_p: Optional[int] = None,
                      arg_a: Optional[int] = None,
                      arg_string: Optional[str] = None) -> None:
        """Send serial print message."""
        if arg_p == 0 and arg_string != "action:cancel":
            if arg_a == 1:
                self.ser_conn.command(f"//{arg_string}")
            else:
                self.ser_conn.echo(f"{arg_string}\nok" if arg_string else "ok")
        self.ser_conn.command("ok")

    def _set_acceleration(self,
                          arg_x: Optional[float] = None,
                          arg_y: Optional[float] = None,
                          **_: Any) -> None:
        """Set the acceleration limits."""
        self._queue_task(
            f"SET_VELOCITY_LIMIT ACCEL={arg_x or arg_y} ACCEL_TO_DECEL={(arg_x or arg_y) / 2}")

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
            self._report(PROBE_OFFSET_TEMPLATE, **(
                self.object_status | {"printer_cfg": self.printer_cfg}))
        self.ser_conn.command("ok")

    def _load_filament(self) -> None:
        """Load filament into the extruder."""
        self._handle_filament(length = 25)

    def _unload_filament(self) -> None:
        """Unload filament from the extruder."""
        self._handle_filament(length= -25)

    def _handle_filament(self, length: int, extruder: float = 0, zmove: float = 20) -> None:
        """Handle filament loading and unloading."""
        scripts = []
        if not "xyz" in self.object_status.get("toolhead").get("homed_axes"):
            scripts = ["G28"]                     # Home if not homed
        scripts.extend(["G90",                    # Absolute Positioning
                        f"G92 E{extruder}"])      # Reset Extruder
        if self.object_status.get("gcode_move", {}).get("gcode_position", [0, 0, 0])[2] < zmove:
            scripts.append(f"G1 F6000 Z{zmove}")  # Raise head if under zmove
        scripts.extend([f"G1 E{length} F{3*60}",  # Extrude or Retract
                        "G92 E0",                 # Relative Positioning
                        "G92 E0"])                # Reset Extruder
        self._queue_task(scripts)

    def _set_feed_rate(self, arg_s: Optional[int] = None, **_: Any) -> None:
        """Set the feed rate."""
        if arg_s is not None:
            self._queue_task(f"M220 S{arg_s}")
        else:
            self._report(
                "FR:{{ gcode_move.speed_factor * 100 | int }}%\nok", **self.object_status)

    def _set_flow_rate(self, arg_s: Optional[int] = None, **_: Any) -> None:
        """Set the flow rate."""
        if arg_s is not None:
            self._queue_task(f"M221 S{arg_s}")
        else:
            self._report(
                "E0 Flow:{{ gcode_move.extrude_factor * 100 | int }}%\nok", **self.object_status)

    def _report_temperature(self) -> None:
        """Report the current temperature."""
        self._report(f"{TEMPERATURE_TEMPLATE}\nok", **self.object_status)

    def _report_position(self) -> None:
        """Report the current position."""
        self._report(f"{POSITION_TEMPLATE}\nok", **self.object_status)

    def _report_firmware_info(self) -> None:
        """Report the firmware information."""
        self._report(FIRMWARE_INFO_TEMPLATE, **(
            self.object_status |
            {"machine_name": self.printer_info.get("machine_name")} |
            {"firmware_name": self.printer_info.get("firmware_name")}))

    def _z_offset_apply_probe(self) -> List[str]:
        """Apply the Z offset from the probe."""
        sd_state = self.object_status.get("print_stats", {}).get("state", "standby")
        if sd_state in ("printing", "paused"):
            self.ser_conn.error("Not saved - Printing")
        else:
            self._queue_task(["Z_OFFSET_APPLY_PROBE", "SAVE_CONFIG"])

    def _restore_settings(self) -> List[str]:
        """Restore settings from file."""
        sd_state = self.object_status.get("print_stats", {}).get("state", "standby")
        if sd_state in ("printing", "paused"):
            self.ser_conn.error("Not saved - Printing")
        else:
            self._queue_task("RESTART")

    def _probe_command(self, arg_s: int, **_: Any) -> None:
        """Handle probe commands."""
        if arg_s == 120:  # Test
            cmd = "QUERY_PROBE"
        else:
            if self.printer_cfg.get("bltouch") is not None:
                value = {10: "pin_down", 90: "pin_up", 160: "reset"}.get(arg_s)
                cmd = f"BLTOUCH_DEBUG COMMAND={value}"
            else:
                value = {10: "1", 90: "0", 160: "0"}.get(arg_s)
                cmd = f"SET_PIN PIN=_probe_enable VALUE={value}"
        self._queue_task(cmd)

    def _probe_at_position(self, arg_x: int, arg_y: int, **_: Any) -> None:
        """ Single Z-Probe at especific position)."""
        self._queue_task([f"G1 F7000 X{arg_x} Y{arg_y}",
                          "PROBE",
                          "G1 Z10"])

def load_component(config: ConfigHelper) -> TFTAdapter:
    """Load the TFT adapter component."""
    return TFTAdapter(config)
