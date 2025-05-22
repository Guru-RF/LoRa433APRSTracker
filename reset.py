import serial
import time
import sys

def wait_for_prompt(ser, timeout=3):
    """Wait for REPL >>> prompt."""
    end_time = time.time() + timeout
    buffer = b""
    while time.time() < end_time:
        data = ser.read(1)
        if data:
            buffer += data
            if b">>>" in buffer:
                return True
    return False

def enter_bootloader(port):
    try:
        print(f"[INFO] Connecting to {port}...")
        with serial.Serial(port, 115200, timeout=0.5) as ser:
            time.sleep(1.0)  # Let it settle

            ser.write(b"\x03")  # Ctrl-C to interrupt any code
            time.sleep(0.5)

            ser.reset_input_buffer()
            ser.write(b"\r\n")
            time.sleep(0.3)

            # Wait for >>> prompt
            if not wait_for_prompt(ser):
                print("[ERROR] REPL prompt not detected.")
                return False

            print("[INFO] REPL detected. Sending bootloader command...")

            ser.write(b"import microcontroller\r\n")
            time.sleep(0.1)
            ser.write(b"microcontroller.on_next_reset(microcontroller.RunMode.BOOTLOADER)\r\n")
            time.sleep(0.1)
            ser.write(b"microcontroller.reset()\r\n")
            print("[INFO] Sent. Board should now reset into UF2 bootloader.")
            return True

    except Exception as e:
        print(f"[ERROR] Serial error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 circuitpython_reboot_uf2.py /dev/tty.usbmodemXXXX")
        sys.exit(1)

    port = sys.argv[1]
    success = enter_bootloader(port)
    if not success:
        sys.exit(1)
