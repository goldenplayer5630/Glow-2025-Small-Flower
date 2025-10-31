# pip install pyserial
import time
import sys
import argparse
import serial
from serial.tools import list_ports

DEFAULT_BAUD = 115200

SEQUENCE = [
    ("0/OPENLEDRAMP:120,3000", 10.0),
    ("0/CLOSELEDRAMP:0,3000",    10.0),
]

def detect_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    preferred = [p for p in ports if any(k in (p.description or "").lower()
                 for k in ["arduino", "wchusb", "ch340", "usb serial", "cp210", "ftdi"])]
    return (preferred[0].device if preferred else ports[0].device)

def open_serial(port, baud):
    return serial.Serial(port, baudrate=baud, timeout=1)

def settle(ser, settle_s=3.5, wait_ready=False, ready_token="READY", ready_timeout=8.0):
    """
    Allow the MCU to reboot after the port opens.
    Optionally wait for a 'READY' line (customize token/timeout).
    """
    # Clear any stale input
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    # Base settle sleep
    time.sleep(settle_s)

    if wait_ready:
        end = time.time() + ready_timeout
        buf = b""
        while time.time() < end:
            chunk = ser.read(128)
            if chunk:
                buf += chunk
                # Print what we see for debugging
                try:
                    print(chunk.decode("utf-8", "ignore"), end="")
                except Exception:
                    pass
                if ready_token in buf.decode("utf-8", "ignore"):
                    break

def send(ser, msg):
    line = (msg + "\r\n").encode("utf-8")
    ser.write(line)
    ser.flush()

def main():
    parser = argparse.ArgumentParser(description="Loop flower commands with specific delays.")
    parser.add_argument("-p", "--port", help="Serial port (e.g., COM5, /dev/ttyUSB0, /dev/tty.usbmodem*).")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default {DEFAULT_BAUD}).")
    parser.add_argument("--settle", type=float, default=3.5, help="Seconds to wait after opening the port (default 3.5).")
    parser.add_argument("--wait-ready", action="store_true", help="Also wait for a 'READY' line after settle.")
    parser.add_argument("--ready-token", default="READY", help="Token to wait for when --wait-ready is set.")
    parser.add_argument("--ready-timeout", type=float, default=8.0, help="Max seconds to wait for ready token.")
    args = parser.parse_args()

    port = args.port or detect_port()
    if not port:
        print("No serial ports found. Specify one with --port.", file=sys.stderr)
        sys.exit(1)

    print(f"Using port: {port} @ {args.baud} baud. Press Ctrl+C to stop.")
    while True:
        try:
            with open_serial(port, args.baud) as ser:
                settle(ser, args.settle, args.wait_ready, args.ready_token, args.ready_timeout)
                while True:
                    for cmd, delay_s in SEQUENCE:
                        print(f"> {cmd}")
                        send(ser, cmd)
                        time.sleep(delay_s)
        except KeyboardInterrupt:
            print("\nStopping.")
            break
        except Exception as e:
            print(f"[WARN] Serial error: {e}. Retrying in 3s...", file=sys.stderr)
            time.sleep(3)

if __name__ == "__main__":
    main()
