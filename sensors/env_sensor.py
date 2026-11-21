"""
Environmental sensor reader. Plain Python, no ROS.

Run on the Pi in its own SSH window:
    cd ~/solara_ws/src/solara-ros/sensors
    source .venv/bin/activate        # if you made one
    python3 env_sensor.py

Prints one reading every PERIOD_S seconds until Ctrl+C.
"""

import time
from datetime import datetime

PERIOD_S = 2.0


class EnvSensor:
    def __init__(self):
        # TODO: set up your sensor here (I2C bus, address, library object).
        pass

    def read(self):
        # TODO: return real values, e.g. {"temperature_c": 22.4, "humidity_pct": 41.0}
        raise NotImplementedError("Fill in EnvSensor.read() for your sensor")


def main():
    sensor = EnvSensor()
    print(f"Reading every {PERIOD_S}s, Ctrl+C to stop")
    try:
        while True:
            reading = sensor.read()
            values = "  ".join(f"{k}={v}" for k, v in reading.items())
            print(f"{datetime.now():%H:%M:%S}  {values}")
            time.sleep(PERIOD_S)
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
