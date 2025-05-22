import serial
import time
import sys

RESET_SCRIPT = (
    b"\x03"  # Ctrl-C to break any running code
    b"import microcontroller\r\n"
    b"microcontroller.on_next_reset(microcontroller.RunMode.BOOTLOADER)\r\n"
    b"microcontroller.reset()\r\n"
)

def reset_to_bootloader(port):
    try:
        print(f"Connecting to {port}...")
        with serial.Serial(port, 115200, timeout=1) as ser:
            time.sleep(1)  # Wait for port to settle
            ser.write(RESET_SCRIPT)
            print("Bootloader reset command sent.")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 reset_to_circuitpython_bootloader.py /dev/tty.usbmodemXYZ")
        sys.exit(1)

    port = sys.argv[1]
    reset_to_bootloader(port)
