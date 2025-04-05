import serial
import time
import sys

<<<<<<< HEAD
PORT = "/dev/ttyUSB1"     # Измени на нужный порт (например, COM3 на Windows)
=======
<<<<<<< HEAD
PORT = "/dev/ttyUSB1"     # Измени на нужный порт (например, COM3 на Windows)
=======
PORT = "/dev/ttyUSB0"     # Измени на нужный порт (например, COM3 на Windows)
>>>>>>> fdacc2d65bbe9946c9e713eb8ad866f7cd30ba2b
>>>>>>> cc73dce9704b607a804d51334c234bb02b206d16
BAUDRATE = 115200
TIMEOUT = 1               # В секундах

def parse_distances(line: str):
<<<<<<< HEAD
    if line.startswith("SECTORS:"):
=======
<<<<<<< HEAD
    if line.startswith("SECTORS:"):
=======
    if line.startswith("DISTANCES:"):
>>>>>>> fdacc2d65bbe9946c9e713eb8ad866f7cd30ba2b
>>>>>>> cc73dce9704b607a804d51334c234bb02b206d16
        try:
            parts = line.strip().split()[1:]  # Убираем "DISTANCES:"
            distances = [float(p) for p in parts]
            if len(distances) == 12:
                return distances
        except ValueError:
            pass
    return None

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"Connected to {PORT} at {BAUDRATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port {PORT}: {e}")
        sys.exit(1)

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            distances = parse_distances(line)
            if distances:
                print("Sectors:")
                for i, d in enumerate(distances):
                    if d >= 9999:
                        print(f"[{i:2}] ---")
                    else:
                        print(f"[{i:2}] {int(d):4} mm")
                print("-" * 30)
            else:
                # Можно раскомментировать для отладки:
                # print(f"Raw: {line}")
                pass

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
