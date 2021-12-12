# TFT LCD display adapter componen for moonraker
#
# Copyright (C) 2020  Hugo Araya Nash <kabroxiko@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations
import os
import time
import logging
import asyncio
import re
import json
from collections import deque
from jinja2 import Template
from typing import (
    TYPE_CHECKING,
    Deque,
    Any,
    Tuple,
    Optional,
    Dict,
    List,
    Callable,
    Coroutine,
    Union,
)
import serial
from ..utils import ServerError
from ..utils import json_wrapper as jsonw

# Annotation imports
if TYPE_CHECKING:
    from ..confighelper import ConfigHelper
    from .klippy_connection import KlippyConnection
    from .klippy_apis import KlippyAPI as APIComp
    from .file_manager.file_manager import FileManager as FMComp
    FlexCallback = Callable[..., Optional[Coroutine]]

PRINT_STATUS_TEMPLATE = (
    "//action:notification "
    "Layer Left {{ (virtual_sdcard.file_position or 0) }}/{{ (virtual_sdcard.file_size or 0) }}"
)

TEMPERATURE_TEMPLATE = (
    "T:{{ extruder.temperature | round(2) }} /{{ extruder.target | round(2) }} "
    "B:{{ heater_bed.temperature | round(2) }} /{{ heater_bed.target | round(2) }} "
    "@:0 B@:0"
)

PROBE_OFFSET_TEMPLATE = (
    "M851 X{{ bltouch.x_offset | float - gcode_move.homing_origin[0] }} "
    "Y{{ bltouch.y_offset | float - gcode_move.homing_origin[1] }} "
    "Z{{ bltouch.z_offset | float - gcode_move.homing_origin[2] }}"
)

REPORT_SETTINGS_TEMPLATE = (
    "M203 X{{ toolhead.max_velocity }} Y{{ toolhead.max_velocity }} "
    "Z{{ printer.max_z_velocity }} E{{ extruder.max_extrude_only_velocity }}\n"
    "M201 X{{ toolhead.max_accel }} Y{{ toolhead.max_accel }} "
    "Z{{ printer.max_z_accel }} E{{ extruder.max_extrude_only_accel }}\n"
    "M206 X{{ gcode_move.homing_origin[0] }} "
         "Y{{ gcode_move.homing_origin[1] }} "
         "Z{{ gcode_move.homing_origin[2] }}\n"
    f"{PROBE_OFFSET_TEMPLATE}\n"
    "M420 S1 Z{{ bed_mesh.fade_end }}\n"
    "M106 S{{ fan.speed }}"
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

    def __init__(self,
                 config: ConfigHelper,
                 tft: TFTAdapter
                 ) -> None:
        """Initialize the serial connection."""
        self.event_loop = config.get_server().get_event_loop()
        self.tft = tft
        self.port: str = config.get('serial')
        self.baud = config.getint('baud', 57600)
        self.partial_input: bytes = b""
        self.ser: Optional[serial.Serial] = None
        self.fd: Optional[int] = None
        self.connected: bool = False
        self.send_busy: bool = False
        self.send_buffer: bytes = b""
        self.attempting_connect: bool = True

    def disconnect(self, reconnect: bool = False) -> None:
        """Disconnect the serial connection."""
        if self.connected:
            if self.fd is not None:
                self.event_loop.remove_reader(self.fd)
                self.fd = None
            self.connected = False
            if self.ser is not None:
                self.ser.close()
            self.ser = None
            self.partial_input = b""
            self.send_buffer = b""
            self.tft.initialized = False
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
                self.ser = serial.Serial(
                    self.port, self.baud, timeout=0, exclusive=True)
            except (OSError, IOError, serial.SerialException):
                logging.exception("Unable to open port: %s", self.port)
                await asyncio.sleep(2.)
                connect_time += time.time()
                continue
            self.fd = self.ser.fileno()
            fd = self.fd = self.ser.fileno()
            os.set_blocking(fd, False)
            self.event_loop.add_reader(fd, self._handle_incoming)
            self.connected = True
            logging.info("TFT Connected")
        self.attempting_connect = False

    def _handle_incoming(self) -> None:
        """Handle incoming data from the serial connection."""
        # Process incoming data using same method as gcode.py
        if self.fd is None:
            return
        try:
            data = os.read(self.fd, 4096)
        except os.error:
            return

        if not data:
            # possibly an error, disconnect
            self.disconnect(reconnect=True)
            logging.info("serial_display: No data received, disconnecting")
            return

        # Remove null bytes, separate into lines
        data = data.strip(b'\x00')
        lines = data.split(b'\n')
        lines[0] = self.partial_input + lines[0]
        self.partial_input = lines.pop()
        for line in lines:
            try:
                decoded_line = line.strip().decode('utf-8', 'ignore')
                self.tft.process_line(decoded_line)
            except ServerError:
                logging.exception("GCode Processing Error: %s", decoded_line)
                self._send_to_tft(error=f"!! GCode Processing Error: {decoded_line}")
            except Exception:
                logging.exception("Error during gcode processing")

class TFTAdapter:
    """Adapter for managing the TFT display."""

    def __init__(self, config: ConfigHelper) -> None:
        """Initialize the TFT adapter."""
        self.server = config.get_server()
        self.event_loop = self.server.get_event_loop()
        self.file_manager: FMComp = self.server.lookup_component('file_manager')
        self.klippy_apis: APIComp = self.server.lookup_component('klippy_apis')
        self.machine_name = config.get('machine_name', "Klipper")
        self.firmware_name: str = "Klipper"
        self.filament_sensor: str = f"filament_switch_sensor {config.get('filament_sensor_name')}"
        self.last_message: Optional[str] = None
        self.current_file: str = ""
        self.file_metadata: Dict[str, Any] = {}
        self.temperature_report_task: Optional[asyncio.Task] = None
        self.position_report_task: Optional[asyncio.Task] = None
        self.print_status_report_task: Optional[asyncio.Task] = None

        # Initialize tracked state.
        self.printer_state: Dict[str, Dict[str, Any]] = {}
        self.config = {}
        self.extruder_count: int = 0
        self.heaters: List[str] = []
        self.is_ready: bool = False
        self.is_shutdown: bool = False
        self.initialized: bool = False
        self.gq_busy: bool = True
        self.queue: List[Union[str, Tuple[FlexCallback, Any]]] = []
        self.last_printer_state: str = 'O'
        self.last_update_time: float = 0.

        self.ser_conn = SerialConnection(config, self)
        logging.info("TFT Configured")

        # Register server events
        self._register_server_events()

        # These commands are directly executued on the server and do not to
        # make a request to Klippy
        self.direct_gcodes: Dict[str, FlexCallback] = {
            'G26': self._send_ok_response, # Mesh Validation Pattern (G26 H240 B70 R99)
            'G29': self._send_ok_response, # Bed leveling (G29)
            'G30': self._send_ok_response, # Single Z-Probe (G30 E1 X28 Y207)
            'M20': self._list_sd_files,
            'M21': self._init_sd_card,
            'M23': self._select_sd_file,
            'M24': self._start_print,
            'M25': self._pause_print,
            'M27': self._set_print_status_autoreport,
            'M33': self._get_long_path,
            'M48': "PROBE_ACCURACY",
            'M81': self._power_off,
            'M82': self._send_ok_response, # E Absolute
            'M92': self._send_ok_response, # Set Axis Steps-per-unit
            'M105': self._report_temperature,
            'M108': self._send_ok_response, # Break and Continue
            'M114': self._report_position,
            'M115': self._report_firmware_info,
            'M118': self._serial_print,
            'M150': self._set_led,
            'M154': self._set_position_autoreport,
            'M155': self._set_temperature_autoreport,
            'M201': self._set_acceleration,
            'M203': self._set_velocity,
            'M206': self._set_gcode_offset,
            'M211': self._report_software_endstops,
            'M220': self._set_feed_rate,
            'M221': self._set_flow_rate,
            'M280': self._probe_command,
            'M290': self._set_babystep,
            'M303': self._pid_autotune,
            'M420': self._set_bed_leveling,
            'M500': self._z_offset_apply_probe,
            'M501': self._restore_settings,
            'M502': self._restore_settings,
            'M503': self._report_settings,
            'M524': "CANCEL_PRINT",
            'M701': self._load_filament,
            'M702': self._unload_filament,
            'M851': self._set_probe_offset,
            'M876': self._send_ok_response, # Handle Prompt Response
            'T0': self._send_ok_response,   # Select or Report Tool
        }

        self.standard_gcodes: List[str] = [
            'G0', 'G1', 'G28', 'G90', 'G92', 'M84', 'M104', 'M106', 'M140'
        ]

    def _register_server_events(self) -> None:
        """Register server event handlers."""
        self.server.register_event_handler(
            "server:klippy_ready", self._process_klippy_ready
        )
        self.server.register_event_handler(
            "server:klippy_shutdown", self._process_klippy_shutdown
        )
        self.server.register_event_handler(
            "server:klippy_disconnect", self._process_klippy_disconnect
        )
        self.server.register_event_handler(
            "server:gcode_response", self.handle_gcode_response
        )

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
                cfg_status = await self.klippy_apis.query_objects({'configfile': None})
            except self.server.error:
                logging.exception("TFT initialization request failed")
                retries -= 1
                if not retries:
                    raise
                await asyncio.sleep(1.)
                continue
            break

        self.firmware_name = "Marlin | Klipper " + printer_info['software_version']
        self.config: Dict[str, Any] = cfg_status.get('configfile', {}).get('config', {})

        logging.info(
            "TFT Config Received:\n"
            "Firmware Name: %s\n"
            "Printer Config: %s\n", self.firmware_name, self.config)

        # Make subscription request
        sub_args: Dict[str, Optional[List[str]]] = {
            "motion_report": None,
            "gcode_move": None,
            "toolhead": None,
            "virtual_sdcard": None,
            "fan": None,
            "display_status": None,
            "print_stats": None,
            "idle_timeout": None,
            "probe": None,
            f"{self.filament_sensor}": None
        }
        self.extruder_count = 0
        self.heaters = []
        extruders = []
        for cfg in self.config:
            if cfg.startswith("extruder"):
                self.extruder_count += 1
                extruders.append(cfg)
                sub_args[cfg] = None
            elif cfg == "heater_bed":
                self.heaters.append(cfg)
                sub_args[cfg] = None
        extruders.sort()
        self.heaters.extend(extruders)
        try:
            self.printer_state = await self.klippy_apis.query_objects(sub_args)
            self._actions(self.printer_state)
            await self.klippy_apis.subscribe_objects(sub_args, self._subcription_updates)
            self.gq_busy = False
        except self.server.error:
            logging.exception("Unable to complete subscription request")
        self.is_shutdown = False
        self.is_ready = True
    def _send_to_tft(self, message=None, command=None, action=None, error=None) -> None:
        """Write a response to the serial connection."""
        if command:
            msg = f'{command}'
        elif action:
            msg = f'//action:{action}'
        elif error:
            msg = f'Error:{error}'
        else:
            msg = f'{message}'

        formatted_msg = msg.replace('\n', '\\n')
        logging.info("write: %s", formatted_msg)
        byte_resp = (msg + "\n").encode("utf-8")
        self.ser_conn.ser.write(byte_resp)

    def _subcription_updates(self, data: Dict[str, Any], _: float):
        """Update printer state values."""
        for key, values in data.items():
            if key in self.printer_state:
                self.printer_state[key].update(values)
        self._actions(data)

    def _actions(self, data: Dict[str, Any]):
        """Process subscription changes."""
        filament_detected = None
        if data.get(self.filament_sensor):
            logging.info(f"data: {data}")
            filament_detected = data[self.filament_sensor]["filament_detected"]
        print_stats = data.get('print_stats')
        if print_stats and print_stats.get('state'):
            logging.info(f"data: {data}")
            state = print_stats["state"]
            if state == 'printing':
                if self.last_printer_state == 'paused':
                    self._send_to_tft(action="resume")
                    if filament_detected:
                        self._send_to_tft(action="prompt_end")
                        self._send_to_tft(action="prompt_begin Continue?")
                        self._send_to_tft(action="prompt_button Ok")
                        self._send_to_tft(action="prompt_show")
                elif self.last_printer_state != 'printing':
                    self._send_to_tft(action="print_start")
            elif state == 'paused':
                if filament_detected is False:
                    self._send_to_tft(action="paused filament_runout")
                else:
                    self._send_to_tft(action="paused")
            elif state == 'cancelled':
                self._send_to_tft(action="cancel")
            self.last_printer_state = state

    def _process_klippy_shutdown(self) -> None:
        """Handle the event when Klippy shuts down."""
        self.is_shutdown = True

    def _process_klippy_disconnect(self) -> None:
        """Handle the event when Klippy disconnects."""
        # Tell the TFT that the printer is "off"
        self._send_to_tft('Reset Software')
        self.last_printer_state = 'O'
        self.is_ready = False
        self.is_shutdown = self.is_shutdown = False

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
            self.queue_task(line)

        elif gcode in self.direct_gcodes:
            if isinstance(self.direct_gcodes[gcode], str):
                self.queue_task(self.direct_gcodes[gcode])
                return
            params: Dict[str, Any] = {}
            for part in parts[1:]:
                logging.debug("part: %s", part)
                if not re.match(r'^-?\d+(?:\.\d+)?$', part[1:]):
                    if not params.get("arg_string"):
                        params["arg_string"] = part
                    else:
                        params["arg_string"] = f'{params["arg_string"]} {part}'
                    continue
                else:
                    arg = part[0].lower()
                    if re.match(r'^-?\d+$', part[1:]):
                        val = int(part[1:])
                    else:
                        val = float(part[1:])
                    params[f"arg_{arg}"] = val
            logging.debug("params: %s", params)
            func = self.direct_gcodes[gcode]
            self.queue_task((func, params))
            return
        else:
            logging.warning("Unregistered command: %s", line)
            self.queue_task(line)

    def queue_task(self, task: Union[str, List[str], Tuple[FlexCallback, Any]]) -> None:
        """Queue a task for execution."""
        if isinstance(task, str) or isinstance(task, list):
            self.queue.append(task)
        elif isinstance(task, tuple) and len(task) == 2:
            self.queue.append((task[0], task[1]))
        else:
            self.queue.append(task)
        if not self.gq_busy:
            self.gq_busy = True
            self.event_loop.register_callback(self._process_queue)

    async def _process_queue(self) -> None:
        """Process the queued tasks."""
        self.gq_busy = True
        while self.queue:
            item = self.queue.pop(0)
            if isinstance(item, str) or isinstance(item, list):
                await self._process_script(item)
            else:
                await self._process_command(item)
        self.gq_busy = False

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
            self._send_to_tft(response)
        except self.server.error:
            msg = f"Error executing script {script}"
            logging.exception(msg)
            self._send_to_tft(error=msg)

    async def _process_command(self, item: Tuple[FlexCallback, Any]) -> None:
        """Process a command task."""
        cmd, args = item
        try:
            ret = cmd(**args)
            if ret is not None:
                await ret
        except Exception:
            logging.exception("Error processing command")

    def _report(self, template, **data):
        """Send report to tft."""
        self._send_to_tft(Template(template).render(**data))

    async def _autoreport(self, template, interval, **data):
        """Send periodic reports based on the specified template."""
        while self.ser_conn.connected and interval > 0:
            self._report(template, **data)
            await asyncio.sleep(interval)

    def _set_autoreport_interval(self, task, template, interval, **data) -> None:
        if interval > 0:
            if task:
                task.cancel()
            task = self.event_loop.create_task(
                self._autoreport(template, interval, **data)
            )
        else:
            if task:
                task.cancel()
                task = None
        self._send_to_tft("ok")

    def _set_temperature_autoreport(self, arg_s: int) -> None:
        """Set the interval for temperature reports."""
        self._set_autoreport_interval(self.temperature_report_task,
                                      f"ok {TEMPERATURE_TEMPLATE}",
                                      arg_s,
                                      **self.printer_state)

    def _set_position_autoreport(self, arg_s: int) -> None:
        """Set the interval for position reports."""
        self._set_autoreport_interval(self.position_report_task,
                                      POSITION_TEMPLATE,
                                      arg_s,
                                      **self.printer_state)

    def _set_print_status_autoreport(self, arg_s: Optional[int] = 0) -> None:
        """Set the interval for print status reports."""
        self._set_autoreport_interval(self.print_status_report_task,
                                      PRINT_STATUS_TEMPLATE,
                                      arg_s,
                                      **self.printer_state)

    def handle_gcode_response(self, response: str) -> None:
        """Handle the response from a G-code command."""
        # TODO
        # Untreated response: echo: 1/75 | ET 34m
        # Untreated response: echo: Restoring "extruder" temperature to 200.0°C,
        #                           this may take some time
        if "// Sending" in response or ("B:" in response and "T0:" in response):
            return
        logging.debug("response: %s" % response)
        if "Klipper state" in response or response.startswith('!!'):
            if "not hot enough" in response:
                self._send_to_tft(error=response[3:])
            else:
                logging.error("response: %s" % response)
        elif response.startswith('File opened:') or \
             response.startswith('File selected') or \
             response.startswith('ok'):
            self._send_to_tft(response)
        elif response.startswith('echo: Adjusted Print Time'):
            timeleft = response.split('echo: Adjusted Print Time')[-1].strip()
            hours, minutes = timeleft.split('hr')
            minutes = minutes.strip().replace('min', '')
            formatted_timeleft = f"{hours}h{minutes}m00s"
            self._send_to_tft(action=f"notification Time Left {formatted_timeleft}")
        elif response.startswith('//'):
            if "prompt_text" in response or \
               "prompt_begin" in response or \
               "prompt_footer_button" in response or \
               "prompt_show" in response:
                logging.error("response: %s" % response)
            elif "probe: open" in response:
                self._report(f"PROBE_TEST_TEMPLATE\nok", **self.printer_state)
            elif "probe accuracy results:" in response:
                parts = response[3:].split(',')
                data = {
                    "max_val": parts[0].split()[-1],
                    "min_val": parts[1].split()[-1],
                    "range_val": parts[2].split()[-1],
                    "avg_val": parts[3].split()[-1],
                    "stddev_val": parts[5].split()[-1]
                }
                self._report(f"PROBE_ACCURACY_TEMPLATE\nok", **data)
            elif "Unknown command" in response:
                self._send_to_tft(error=response[3:])
            else:
                self._send_to_tft(response[3:])
        else:
            logging.info("Untreated response: %s", response)

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
        self.current_file = self._clean_filename(arg_string)
        self.queue_task(f"M23 {self.current_file}")

    def _start_print(self) -> None:
        """Start printing the selected file."""
        sd_state = self.printer_state.get("print_stats", {}).get("state", "standby")
        if sd_state == "paused":
            self.queue_task("RESUME")
        elif sd_state in ("standby", "cancelled"):
            self.queue_task(f"SDCARD_PRINT_FILE FILENAME=\"{self.current_file}\"")
        else:
            self._send_to_tft(error="Cannot start printing, printer is not in a stopped state")

    def _pause_print(self, arg_p: int) -> None:
        """Pause the current print."""
        # TODO: handle P1
        sd_state = self.printer_state.get("print_stats", {}).get("state", "standby")
        if sd_state == "printing":
            self.queue_task("PAUSE")
        else:
            self._send_to_tft(error="Cannot pause, printer is not printing")

    def _probe_command(self, arg_p: int, arg_s: int) -> None:
        """Handle probe commands."""
        if arg_s == 120:  # Test
            cmd = "QUERY_PROBE"
        else:
            if self.config.get("bltouch"):
                value = {
                    10: "pin_down", 90: "pin_up", 160: "reset"
                }.get(arg_s)
                cmd = f"BLTOUCH_DEBUG COMMAND={value}"
            else:
                value = {
                    10: "1", 90: "0", 160: "0"
                }.get(arg_s)
                cmd = f"SET_PIN PIN=_probe_enable VALUE={value}"
        self.queue_task(cmd)

    def _print_file(self, args: List[str]) -> None:
        """Print the specified file."""
        filename = self._clean_filename(args[0])
        # Escape existing double quotes in the file name
        filename = filename.replace("\"", "\\\"")
        self.queue_task(f"SDCARD_PRINT_FILE FILENAME=\"{filename}\"")

    def _set_led(self, **args: Dict[int]) -> None:
        """Set the LED color and brightness."""
        red = args.get('arg_r', 0) / 255
        green = args.get('arg_u', 0) / 255
        blue = args.get('arg_b', 0) / 255
        white = args.get('arg_w', 0) / 255
        brightness = args.get('arg_p', 255) / 255
        cmd = (
            f"SET_LED LED=statusled "
            f"RED={red * brightness:.3f} "
            f"GREEN={green * brightness:.3f} "
            f"BLUE={blue * brightness:.3f} "
            f"WHITE={white * brightness:.3f} "
            "TRANSMIT=1 SYNC=1"
        )
        self.queue_task(cmd)

    def _set_babystep(self, **args: Dict[float]) -> None:
        """Set the babystep offsets."""
        offsets = []
        if 'arg_x' in args:
            offsets.append(f"X={args['arg_x']}")
        if 'arg_y' in args:
            offsets.append(f"Y={args['arg_y']}")
        if 'arg_z' in args:
            offsets.append(f"Z={args['arg_z']}")
        offset_str = " ".join(offsets)
        self.queue_task(f"SET_GCODE_OFFSET {offset_str}")

    def _pid_autotune(self, **args: Dict[float]) -> None:
        """Initiates a process to determine the PID values."""
        heater = "heater_bed" if args.get("arg_e") == -1 else "extruder"
        cmd = f"PID_CALIBRATE HEATER={heater} TARGET={args.get('arg_s')}"
        if args.get("arg_u") == 1:
            cmd = [cmd, "SAVE_CONFIG"]
        self.queue_task(cmd)

    def _set_bed_leveling(self, **args) -> None:
        """Set the bed leveling state."""
        if args.get('arg_s'):
            if args.get('arg_s') == 0:
                self.queue_task("BED_MESH_CLEAR")
            else:
                self.queue_task("BED_MESH_PROFILE LOAD=default")
        else:
            # TODO: Falta implementar M420 V1 T1 y M420 Zx.xx
            self._send_to_tft("ok")

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
        self.queue_task(cmd)

    def _init_sd_card(self) -> None:
        """Initialize the SD card."""
        self._send_to_tft("SD card ok\nok")

    def _list_sd_files(self, arg_string: Optional[str] = None) -> None:
        """List the files on the SD card."""
        response_type = 2
        if response_type != 2:
            logging.info("Cannot process response type %s in M20", response_type)
            return
        path = "/"

        # Strip quotes if they exist
        path = path.strip('\"')

        # Path should come in as "0:/macros, or 0:/<gcode_folder>".  With
        # repetier compatibility enabled, the default folder is root,
        # ie. "0:/"
        if path.startswith("0:/"):
            path = path[2:]
        response: Dict[str, Any] = {'dir': path}
        response['files'] = []

        if path == "/":
            response['dir'] = "/gcodes"
            path = "gcodes"
        elif path.startswith("/gcodes"):
            path = path[1:]

        flist = self.file_manager.list_dir(path, simple_format=False)
        if flist:
            files = {
                "files" : [(file['filename'], file['size']) for file in flist.get("files")]
            }
        self._report(FILE_LIST_TEMPLATE, **files)

    async def _delete_sd_file(self, arg_string: str = "") -> None:
        """Delete a file from the SD card."""
        # Delete a file.  Clean up the file name and make sure
        # it is relative to the "gcodes" root.
        path = arg_string
        path = path.strip('\"')
        if path.startswith("0:/"):
            path = path[3:]

        if not path.startswith("gcodes/"):
            path = "gcodes/" + path
        await self.file_manager.delete_file(path)

    def _get_long_path(self, arg_string: Optional[str] = None) -> None:
        """Get the full path of a file."""
        filename: Optional[str] = arg_string
        if filename is None:
            self._send_to_tft(error="Missing filename\nok")
            return

        # Clean up the filename
        filename = filename.strip('\"')
        if filename.startswith("0:/"):
            filename = filename[3:]

        if not filename.startswith("gcodes/"):
            filename = "gcodes/" + filename

        self._send_to_tft(f"{filename}\nok")

    def _report_software_endstops(self) -> None:
        """Report the status of software endstops."""
        filament_sensor=self.printer_state.get(self.filament_sensor, {})
        state = { "state": "On" if filament_sensor.get("enabled", False) else "Off"}
        self._report(f"{SOFTWARE_ENDSTOPS_TEMPLATE}\nok", **state)

    def _report_settings(self, arg_s: Optional[str] = None) -> None:
        """Report the printer settings."""
        self._report(f"{REPORT_SETTINGS_TEMPLATE}\nok", **(self.printer_state | self.config))

    def _send_ok_response(self, **args: Dict[float]) -> None:
        """Send an 'ok' response."""
        self._send_to_tft("ok")

    def _serial_print(self,
                      arg_p: Optional[int] = None,
                      arg_a: Optional[int] = None,
                      arg_string: Optional[str] = None) -> None:
        """Send serial print message."""
        self._send_to_tft("ok")
        if arg_p == 0 and arg_string != "action:cancel":
            if arg_a == 1:
                self._send_to_tft(f"//{arg_string}")
            else:
                self._send_to_tft(f"echo:{arg_string}\nok" if arg_string else "ok")

    def _set_acceleration(self, **args: Dict[float]) -> None:
        """Set the acceleration limits."""
        acceleration = args.get("arg_x") or args.get("arg_y")
        cmd = f"SET_VELOCITY_LIMIT ACCEL={acceleration} ACCEL_TO_DECEL={acceleration / 2}"
        self.queue_task(cmd)

    def _set_velocity(self, **args: Dict[float]) -> None:
        """Set the velocity limits."""
        velocity = args.get("arg_x") or args.get("arg_y")
        cmd = f"SET_VELOCITY_LIMIT VELOCITY={velocity}"
        self.queue_task(cmd)

    def _set_gcode_offset(self, **args: Dict[float]) -> None:
        """Set the G-code offsets."""
        offsets = []
        if 'arg_x' in args:
            offsets.append(f"X={args['arg_x']}")
        if 'arg_y' in args:
            offsets.append(f"Y={args['arg_y']}")
        if 'arg_z' in args:
            offsets.append(f"Z={args['arg_z']}")
        offset_str = " ".join(offsets)
        self.queue_task(f"SET_GCODE_OFFSET {offset_str}")

    def _set_probe_offset(self, **args: Dict[float]) -> None:
        """Set the probe offsets."""
        if not args:
            self._send_to_tft(PROBE_OFFSET_TEMPLATE, **(self.printer_state | self.config))
        self._send_to_tft("ok")

    def _load_filament(self) -> None:
        """Load filament into the extruder."""
        params = {
            "length": -25,
            "extruder": 0,
            "zmove": 0
        }
        self._handle_filament(params)

    def _unload_filament(self) -> None:
        """Unload filament from the extruder."""
        params = {
            "length": 25,
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
        self.queue_task(cmd)

    def close(self) -> None:
        """Close the TFT adapter and disconnect."""
        self.ser_conn.disconnect()
        if self.temperature_report_task:
            self.temperature_report_task.cancel()
        if self.position_report_task:
            self.position_report_task.cancel()
        if self.print_status_report_task:
            self.print_status_report_task.cancel()

    def _set_feed_rate(self, arg_s: Optional[int] = None, arg_d: Optional[int] = None) -> None:
        """Set the feed rate."""
        if arg_s is not None:
            self.queue_task(f"M220 S{arg_s}")
        else:
            self._report(f"{FEED_RATE_TEMPLATE}\nok", **self.printer_state)

    def _set_flow_rate(self, arg_s: Optional[int] = None, arg_d: Optional[int] = None) -> None:
        """Set the flow rate."""
        if arg_s is not None:
            self.queue_task(f"M221 S{arg_s}")
        else:
            self._report(f"{FLOW_RATE_TEMPLATE}\nok", **self.printer_state)

    def _report_temperature(self) -> None:
        """Report the current temperature."""
        self._report(f"{TEMPERATURE_TEMPLATE}\nok", **self.printer_state)

    def _report_position(self) -> None:
        """Report the current position."""
        self._report(f"{POSITION_TEMPLATE}\nok", **self.printer_state)

    def _report_firmware_info(self) -> None:
        """Report the firmware information."""
        self._report(FIRMWARE_INFO_TEMPLATE, **(
            self.printer_state |
            { "machine_name": self.machine_name } |
            { "firmware_name": self.firmware_name })
        )

    def _z_offset_apply_probe(self) -> List[str]:
        """Apply the Z offset from the probe."""
        sd_state = self.printer_state.get("print_stats", {}).get("state", "standby")
        if sd_state in ("printing", "paused"):
            self._send_to_tft(error="Not saved - Printing")
        else:
            self.queue_task(["Z_OFFSET_APPLY_PROBE", "SAVE_CONFIG"])

    def _restore_settings(self) -> List[str]:
        """Restore settings from file."""
        sd_state = self.printer_state.get("print_stats", {}).get("state", "standby")
        if sd_state in ("printing", "paused"):
            self._send_to_tft(error="Not saved - Printing")
        else:
            self.queue_task("RESTART")

def load_component(config: ConfigHelper) -> TFTAdapter:
    """Load the TFT adapter component."""
    return TFTAdapter(config)
