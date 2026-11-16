"""Run one supervised v2 swing-up using the independently measured shaft home."""
import argparse
from dataclasses import dataclass
import math
import time

import serial


@dataclass(frozen=True)
class Sample:
    device_ms: int
    counted_deg: float
    mode: int
    rate: float
    angle: float
    pendulum_valid: bool
    zero_ready: bool
    center_ready: bool
    shaft_deg: float
    shaft_valid: bool


def parse_sample(line):
    fields = line.split(",")
    if len(fields) != 20:
        return None
    try:
        v = [float(field) for field in fields]
    except ValueError:
        return None
    if not all(math.isfinite(x) for x in v):
        return None
    shaft_valid = 0 <= v[18] < 4096 and int(v[19]) & 0x38 == 0x20
    return Sample(int(v[0]), math.degrees(v[6]), int(v[8]), v[10], v[12],
                  bool(v[13]) and bool(v[3]) and not (v[4] or v[5]),
                  bool(v[14]), bool(v[15]), v[18] * 360 / 4096, shaft_valid)


def send(port, command):
    port.write((command + "\n").encode("ascii"))


def read(port):
    return port.readline().decode("ascii", errors="replace").strip()


def wait_for(port, match, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = read(port)
        if line.startswith("EVENT,REJECT,"):
            raise RuntimeError(line)
        if match(line):
            return line
    raise RuntimeError("Timed out waiting for firmware response")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def offset(angle, reference):
    return math.remainder(angle - reference, 360)


def run_trial(port, home):
    def configure(command, event):
        send(port, command)
        print(wait_for(port, lambda line: line.startswith("EVENT," + event), 3))

    def sample():
        return parse_sample(wait_for(port, lambda line: parse_sample(line) is not None, 3))

    # Let boards that reset when the serial port opens finish booting.
    time.sleep(2)
    port.reset_input_buffer()
    row = sample()
    require(row.mode == 0 and row.pendulum_valid and row.shaft_valid, "Expected idle firmware and two valid magnets")
    require(row.zero_ready and abs(row.angle) < 3 and abs(row.rate) < .2, "Calibrate the hanging zero and let the pendulum settle")
    require(abs(offset(row.shaft_deg, home)) <= .65, "Manually return to the measured shaft home")
    for command, event in [
        ("TMCVERIFY", "TMC_UART_VERIFIED,"),
        ("CENTER", "CENTER_CONFIRMED"),
        ("POLICY V2", "POLICY,V2"),
        ("LIMITS 7.5 65", "LIMITS_OK,"),
        ("BALANCELIMITS 5 100", "BALANCE_LIMITS,"),
        ("DURATION 50", "PUMP_BUDGET,"),
        ("APPROACHMODEL 134.2 1.17 0.9934 4", "APPROACH_MODEL_FIXED"),
    ]:
        configure(command, event)
    row = sample()
    require(row.mode == 0 and row.center_ready and row.shaft_valid and abs(offset(row.shaft_deg, home)) <= .65,
            "Home moved during preflight")
    origin = row.shaft_deg
    configure("RUN", "SWINGUP_BEGIN,")

    fresh_at = time.monotonic()
    deadline = fresh_at + 105
    last_device_ms = overflow_since = None
    finished = returned = False
    while time.monotonic() < deadline:
        line = read(port)
        if line.startswith("EVENT,"):
            print(line)
        require(not line.startswith(("EVENT,REJECT,", "EVENT,SENSOR_FAULT,", "EVENT,BOOT,")), line)
        if line.startswith("EVENT,STOP,"):
            require(line.startswith("EVENT,STOP,centered"), line)
        finished |= line.startswith("EVENT,TRIAL_FINISH,success_30s_upright,")
        returned |= line.startswith("EVENT,HOME_COMPLETE,")
        row = parse_sample(line)
        now = time.monotonic()
        if row is not None and row.device_ms != last_device_ms:
            last_device_ms = row.device_ms
            fresh_at = now
            require(row.pendulum_valid and row.shaft_valid, "Invalid encoder telemetry")
            require(abs(offset(row.shaft_deg, home)) <= 100, "Shaft travel exceeded 100 degrees")
            discrepancy = abs(offset(row.shaft_deg, origin) - row.counted_deg)
            require(discrepancy <= 5, "Shaft disagrees with commanded steps by over 5 degrees")
            if discrepancy > 3:
                if overflow_since is None:
                    overflow_since = row.device_ms
                require(row.device_ms - overflow_since < 40, "Shaft disagreement exceeded 3 degrees for 40 ms")
            else:
                overflow_since = None
            if returned and row.mode in (0, 5):
                require(abs(offset(row.shaft_deg, home)) <= .7, "Center return outside tolerance")
                print("30-second hold and return verified" if finished else "Returned to center without a 30-second hold")
                return finished
        require(now - fresh_at <= .6, "Lost fresh telemetry")
    raise RuntimeError("Trial timed out")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 serial device")
    parser.add_argument("--home-deg", type=float, required=True, help="Raw shaft angle measured at mechanical center")
    args = parser.parse_args()
    if not 0 <= args.home_deg < 360:
        parser.error("--home-deg must be in [0, 360)")
    with serial.Serial(args.port, 115200, timeout=.1, write_timeout=.3) as port:
        try:
            return 0 if run_trial(port, args.home_deg) else 1
        except (RuntimeError, serial.SerialException) as error:
            print(f"Trial stopped: {error}")
            return 1
        finally:
            try:
                send(port, "STOP")
            except serial.SerialException:
                print("Serial link unavailable. Use the motor power cutoff.")


if __name__ == "__main__":
    raise SystemExit(main())
