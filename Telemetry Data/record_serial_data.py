import csv
from datetime import datetime
import serial

PORT = "/dev/ttyACM0"
BAUD = 230400
FILENAME = f"balance_bot_log_{datetime.now():%Y-%m-%d_%H-%M-%S}.csv"


def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser, open(
        FILENAME, "w", newline="", encoding="utf-8"
    ) as f:
        writer = csv.writer(f)

        while True:
            line = ser.readline().decode("UTF-8", errors="ignore").strip()
            if not line:
                continue
            print(line)
            row = line.split(",")
            writer.writerow(row)
            f.flush()


if __name__ == "__main__":
    main()
