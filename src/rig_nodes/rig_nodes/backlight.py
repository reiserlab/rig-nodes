#!/usr/bin/env python3
"""backlight.py — the LED backlight controller: serial driver and plugin vocabulary (pure, no ROS).

The controller (IR and red/green/blue backlight panels, marker LEDs, shock outputs) takes one
ASCII command per line, CR-terminated, at 115200 baud. This module is a line-for-line port of
MATLAB's LEDController.m (maDisplayTools): every method sends the same bytes the MATLAB method
sends, including MATLAB's rounding and number formatting. Two layers:

* `LEDController` — the driver: opens the port, one method per controller command, and the
  reply reader for the few query commands (checkControllerStatus).
* `plan(command, params)` — the protocol plugin vocabulary (LEDControllerPlugin.m's seven
  commands) mapped onto controller lines, with the plugin's argument rules. backlight_node.py
  executes it; an executor can call it to check a protocol before touching hardware.

Importing this module needs only pyserial, so the driver works from plain Python:

    from rig_nodes.backlight import LEDController
    with LEDController("COM6") as led:
        led.set_ir_led_power(50)
        led.turn_on_led()

`port` is anything pyserial's serial_for_url accepts: "COM6", "/dev/ttyACM0", "loop://"
(echoes every byte; the pure tests) or "socket://127.0.0.1:<port>" (the simulator,
sims/backlight_sim.py). Closing the port leaves the LEDs as they are (LEDController.m's behavior).

VERIFY ON THE BENCH: the expected bytes come from LEDController.m's source; that MATLAB's
fprintf on a serialport object appends the CR terminator is assumed, not yet captured.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Mapping

import serial

log = logging.getLogger(__name__)

BAUD_RATE = 115200
TERMINATOR = b"\r"
STATUS_MAX_WAIT_S = 0.5  # checkControllerStatus maxwaittime
WRITE_TIMEOUT_S = 10.0  # MATLAB serialport's default Timeout

# Field order of the addOneStep command, exactly as LEDController.m packs it.
STEP_FIELDS = (
    "NumStep",
    *(f"{color}{field}" for color in ("Red", "Grn", "Blu")
      for field in ("Intensity", "PulseWidth", "PulsePeriod", "PulseNum", "OffTime", "Iteration")),
    "DelayTime",
    "Duration",
)

# The protocol plugin vocabulary (LEDControllerPlugin.m; webDisplayTools js/plugin-registry.js
# is a client of it) and the parameters each command takes, in positional order.
PLUGIN_PARAMS: dict[str, tuple[str, ...]] = {
    "setIRLEDPower": ("power",),
    "setRedLEDPower": ("power", "panel_num", "pattern"),
    "setGreenLEDPower": ("power", "panel_num", "pattern"),
    "setBlueLEDPower": ("power", "panel_num", "pattern"),
    "turnOnLED": (),
    "turnOffLED": (),
    "setVisibleBacklightsOff": (),
}
PLUGIN_COMMANDS = tuple(PLUGIN_PARAMS)
COLOR_PREFIX = {"setRedLEDPower": "RED", "setGreenLEDPower": "GRN", "setBlueLEDPower": "BLU"}
TURN_ON_LINE = "ON 0,0"  # "0,0" = all panels, all quadrants (LEDController.m's comment)
TURN_OFF_LINE = "OFF"


# ── MATLAB-compatible formatting: the bytes on the wire must match what MATLAB sends ──────────


def matlab_round(value: float) -> int:
    """Round half away from zero, like MATLAB's round() (Python's round(2.5) is 2, MATLAB's is 3)."""
    x = float(value)
    if not math.isfinite(x):
        raise ValueError(f"cannot round non-finite value {value!r}")
    whole = math.floor(abs(x))
    if abs(x) - whole >= 0.5:
        whole += 1
    return int(math.copysign(whole, x))


def num2str(value) -> str:
    """A scalar formatted like MATLAB's num2str(): strings unchanged, whole numbers without a
    decimal point, others with max(floor(log10|x|) + 5, 5) significant digits (capped at 16)."""
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    x = float(value)
    if math.isnan(x):
        return "NaN"
    if math.isinf(x):
        return "Inf" if x > 0 else "-Inf"
    if x.is_integer():
        return str(int(x))
    digits = min(max(math.floor(math.log10(abs(x))) + 5, 5), 16)
    return f"{x:.{digits}g}"


def _format_step_value(value) -> str:
    """One addOneStep number: sprintf('%d') when whole, else sprintf('%f')."""
    x = float(value)
    if math.isnan(x):
        return "NaN"
    if math.isinf(x):
        return "Inf" if x > 0 else "-Inf"
    return f"{int(x):d}" if x.is_integer() else f"{x:f}"


def _str2double(text: str) -> float:
    """A reply line as a number, NaN when it is not one (MATLAB's str2double)."""
    try:
        return float(text)
    except ValueError:
        return math.nan


def _require_str(value, name: str) -> str:
    """Text arguments must be strings: an unquoted YAML `pattern: 1010` arrives as a number, and
    MATLAB would send the character with that code instead."""
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a string, got {type(value).__name__} {value!r}")
    return value


# ── line builders, shared by the driver and plan() ──────────────────────────────────────────


def ir_line(power) -> str:
    return f"IR {matlab_round(power)}"


def color_line(prefix: str, power, panel_num=None, pattern=None) -> str:
    """setRed/Green/BlueLEDPower. MATLAB picks the form by argument count, so None ("not given")
    and 0 differ: power → "RED 50"; + panel → "RED 50 2"; + pattern → "RED 50 2 1010"."""
    if pattern is not None and panel_num is None:
        raise ValueError("pattern requires panel_num (the controller reads them by position)")
    line = f"{prefix} {matlab_round(power)}"
    if panel_num is not None:
        line += f" {num2str(panel_num)}"
    if pattern is not None:
        line += f" {_require_str(pattern, 'pattern')}"
    return line


# ── the plugin vocabulary ───────────────────────────────────────────────────────────────────


def plan(command: str, params: Mapping | None = None) -> tuple[list[str], list[str]]:
    """Plugin command → (controller lines, warnings), as LEDControllerPlugin.m maps it.

    Raises ValueError for an unknown command or a missing `power`, TypeError for a non-string
    pattern. Kept from the MATLAB plugin: a pattern without panel_num is dropped (warning), and
    parameters a command does not take are ignored (warning)."""
    params = params or {}
    if command not in PLUGIN_PARAMS:
        raise ValueError(f"unknown plugin command {command!r} (one of {PLUGIN_COMMANDS})")
    allowed = PLUGIN_PARAMS[command]
    warnings = [f"parameter {k!r} ignored: {command} does not take it" for k in params if k not in allowed]
    if "power" in allowed and params.get("power") is None:
        raise ValueError("missing required parameter: power")
    if command == "setIRLEDPower":
        return [ir_line(params["power"])], warnings
    if command in COLOR_PREFIX:
        panel_num, pattern = params.get("panel_num"), params.get("pattern")
        if pattern is not None and panel_num is None:
            warnings.append("pattern ignored because panel_num was not given")
            pattern = None
        return [color_line(COLOR_PREFIX[command], params["power"], panel_num, pattern)], warnings
    if command == "turnOnLED":
        return [TURN_ON_LINE], warnings
    if command == "turnOffLED":
        return [TURN_OFF_LINE], warnings
    return [color_line(p, 0) for p in COLOR_PREFIX.values()], warnings  # setVisibleBacklightsOff


# ── the driver ──────────────────────────────────────────────────────────────────────────────


class LEDController:
    """The controller on one serial port; a port of LEDController.m with every method.

    The port opens on construction (as in MATLAB); close() or a `with` block closes it.

    Args:
        port: serial port or pyserial URL (see the module docstring).
        check_status: read the reply after the query commands (MATLAB docheckstatus).
        display_status: print reply lines instead of returning them (MATLAB dispstatus).
        write_timeout_s: a write that cannot finish in this time raises
            serial.SerialTimeoutException (MATLAB serialport's Timeout, 10 s).
    """

    def __init__(self, port: str, *, check_status: bool = True, display_status: bool = False,
                 write_timeout_s: float = WRITE_TIMEOUT_S) -> None:
        self.port = port
        self.check_status = check_status
        self.display_status = display_status
        self._lock = threading.RLock()  # reentrant: multi-line commands hold it across send_line
        self._serial = serial.serial_for_url(port, baudrate=BAUD_RATE, timeout=STATUS_MAX_WAIT_S,
                                             write_timeout=write_timeout_s)

    # ── lifecycle ──

    @property
    def is_open(self) -> bool:
        return self._serial.is_open

    def close(self) -> None:
        """Close the port. The LEDs stay in whatever state they are in."""
        with self._lock:
            if self._serial.is_open:
                self._serial.close()

    def __enter__(self) -> LEDController:
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # ── wire ──

    def send_line(self, line: str) -> None:
        """Write one command line plus CR."""
        with self._lock:
            log.debug("-> %r", line)
            self._serial.write(line.encode("ascii") + TERMINATOR)

    def _read_status(self) -> list[str]:
        """checkControllerStatus: wait up to 0.5 s for more than one byte, then read lines until
        one byte or fewer is left. Returns the non-empty lines (printed instead when
        display_status is set)."""
        start = time.monotonic()
        waited = 0.0
        while self._serial.in_waiting <= 1:
            waited = time.monotonic() - start
            if waited >= STATUS_MAX_WAIT_S:
                break
            time.sleep(0.001)  # MATLAB spins without sleeping; 1 ms spares a CPU core
        log.debug("waited %f s for controller status", waited)
        status = []
        while self._serial.in_waiting > 1:
            line = self._serial.read_until(TERMINATOR).decode("ascii", errors="replace").strip()
            if not line:
                continue
            if self.display_status:
                print(line)
            else:
                status.append(line)
        return status

    def _query(self, line: str) -> list[str] | None:
        """Send a query and, if check_status is on, return its reply lines."""
        with self._lock:
            self.send_line(line)
            return self._read_status() if self.check_status else None

    # ── LED power ──

    def set_ir_led_power(self, power) -> None:
        self.send_line(ir_line(power))

    def set_red_led_power(self, power, panel_num=None, pattern=None) -> None:
        self.send_line(color_line("RED", power, panel_num, pattern))

    def set_green_led_power(self, power, panel_num=None, pattern=None) -> None:
        self.send_line(color_line("GRN", power, panel_num, pattern))

    def set_blue_led_power(self, power, panel_num=None, pattern=None) -> None:
        self.send_line(color_line("BLU", power, panel_num, pattern))

    def set_visible_backlights_off(self) -> None:
        with self._lock:
            for prefix in COLOR_PREFIX.values():
                self.send_line(color_line(prefix, 0))

    def set_ir_backlights_off(self) -> None:
        self.set_ir_led_power(0)

    def turn_on_led(self) -> None:
        self.send_line(TURN_ON_LINE)

    def turn_off_led(self) -> None:
        self.send_line(TURN_OFF_LINE)

    # ── pulses ──

    def set_pulse_param(self, param: Mapping) -> None:
        """param: pulse_width, pulse_period, number_of_pulses, pulse_train_interval, LED_delay,
        iteration, color (string)."""
        keys = ("pulse_width", "pulse_period", "number_of_pulses", "pulse_train_interval", "LED_delay", "iteration")
        numbers = " ".join(num2str(param[k]) for k in keys)
        self.send_line(f"PULSE {numbers} {_require_str(param['color'], 'color')}")

    def start_pulse(self) -> None:
        self.send_line("RUN")

    def stop_pulse(self) -> None:
        self.send_line("STOP")

    # ── marker ──

    def blink_marker(self, blink_time) -> None:
        self.send_line(f"blink {num2str(blink_time)} red")

    def marker_led_on(self, color: str) -> None:
        self.send_line(f"Marker {_require_str(color, 'color')} ON")

    def marker_led_off(self, color: str) -> None:
        self.send_line(f"Marker {_require_str(color, 'color')} OFF")

    # ── shock ──

    def set_shock_pattern(self, pattern: str) -> None:
        """pattern: 16-character binary string, e.g. "1100110011001100"."""
        self.send_line(f"DIGITAL {_require_str(pattern, 'pattern')}")

    def set_shock_pulse(self, param: Mapping) -> None:
        """param: onTime, offTime, cycles, delayTime."""
        on, off = param["onTime"], param["offTime"]
        self.send_line(f"PULSE {num2str(on)} {num2str(on + off)} {num2str(param['cycles'])} 0 "
                       f"{num2str(param['delayTime'])} 1 D")

    def turn_on_shock(self) -> None:
        self.send_line("DON")

    def turn_off_shock(self) -> None:
        self.send_line("DOFF")

    def start_shock_pulse(self) -> None:
        self.send_line("RUN D")

    def stop_shock_pulse(self) -> None:
        self.send_line("STOP")  # the same bytes as stop_pulse, as in MATLAB

    # ── experiment protocols stored on the controller ──

    def add_one_step(self, one_step: Mapping) -> list[float] | None:
        """One protocol step; keys are STEP_FIELDS (the MATLAB struct's field names) plus an
        optional "Pattern" string. Raises ValueError where MATLAB shows an error dialog. Returns
        the reply lines as numbers (NaN where not numeric), None when check_status is off."""
        step = dict(one_step)  # MATLAB structs are copies: never edit the caller's dict
        for prefix, color in (("Red", "red"), ("Grn", "green"), ("Blu", "blue")):
            intensity = step[f"{prefix}Intensity"]
            if intensity > 0:
                pulse_set_s = ((step[f"{prefix}PulsePeriod"] * step[f"{prefix}PulseNum"] + step[f"{prefix}OffTime"])
                               * step[f"{prefix}Iteration"] / 1000)
                if step["Duration"] < step["DelayTime"] + pulse_set_s:
                    raise ValueError(f"Length of {color} led pulse set is longer than duration "
                                     f"in step {num2str(step['NumStep'])}!")
            elif intensity == 0:
                step[f"{prefix}PulseWidth"] = 0  # the controller lights its indicator when width > 0
        line = "addOneStep " + "".join(_format_step_value(step[f]) + " " for f in STEP_FIELDS)
        if "Pattern" in step:
            line += " " + _require_str(step["Pattern"], "Pattern")  # two spaces before it, as in MATLAB
        status = self._query(line)
        return None if status is None else [_str2double(s) for s in status]

    def remove_all_experiment_steps(self) -> None:
        self.send_line("removeAllSteps")

    def get_experiment_steps(self) -> list[str] | None:
        return self._query("getExperimentSteps")

    def run_experiment(self) -> None:
        self.send_line("runExperiment")

    def stop_experiment(self) -> None:
        self.send_line("stopExperiment")

    def get_experiment_status(self) -> str | None:
        status = self._query("getExperimentStatus")
        if status is None:
            return None
        if not status:
            raise RuntimeError("controller sent no reply to getExperimentStatus")
        return status[0]

    def step_order(self, step) -> None:
        self.send_line(f"STEPORDER {num2str(step)}")

    def get_experiment_step_orders(self) -> list[str] | None:
        return self._query("STEPORDER ")  # trailing space, as in MATLAB

    def remove_all_experiment_step_orders(self) -> None:
        self.send_line("STEPORDER 0")

    # ── other ──

    def sync_camera(self, freq: str) -> None:
        """MATLAB name: synCamera. freq is sent as text."""
        self.send_line(f"SYNC {_require_str(freq, 'freq')}")

    def fly_bowls_enabled(self, enable_map: str) -> None:
        self.send_line(f"enableMap {_require_str(enable_map, 'enable_map')}")

    def set_pattern(self, pattern: str) -> None:
        """Four-panel on/off pattern, 16 characters (4 per panel). For Lisha's rig."""
        _require_str(pattern, "pattern")
        if len(pattern) < 16:
            raise ValueError(f"pattern needs at least 16 characters, got {len(pattern)}")
        with self._lock:
            for panel in range(4):
                self.send_line(f"ON {panel + 1} {pattern[4 * panel:4 * panel + 4]}")

    def reset(self) -> None:
        """Reset the controller; all power values return to 0."""
        self.send_line("RESET")
