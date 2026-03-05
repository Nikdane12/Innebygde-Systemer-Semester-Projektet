#!/usr/bin/env python3
"""
pi_controller.py
AUT-2606 - Oppgave 8: Kommunikasjon med IO-kort

Raspberry Pi side: communicates with the AVR IO-card over USART.

Message protocol (matches io_card.c):
    Send:    <CMD>:<ARG>\n
    Receive: <CMD>:<VALUE>\n  or  OK\n  or  ERR\n

Supported commands (mirroring the AVR firmware):
    ADC read     : send_command("ADC:<ch>")        -> "ADC:<value>"
    LED control  : send_command("LED:<idx>,<0|1>") -> "OK"
    Servo angle  : send_command("SRV:<angle>")     -> "OK"
    Buzzer freq  : send_command("BUZ:<hz>")        -> "OK"

Usage (interactive demo):
    python3 pi_controller.py [/dev/ttyS0]
"""

import serial
import time
import sys
import threading


# ---------------------------------------------------------------------------
# Low-level serial helper
# ---------------------------------------------------------------------------

class IOCard:
    """Manages USART communication with the AVR IO-card."""

    def __init__(self, port: str = "/dev/ttyS0", baudrate: int = 9600,
                 timeout: float = 1.0):
        self.ser = serial.Serial(port, baudrate=baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=timeout)
        # Small pause to let the AVR reset if DTR toggled the reset pin
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        print(f"[IOCard] Connected on {port} @ {baudrate} baud")

    def close(self):
        self.ser.close()

    # -----------------------------------------------------------------------
    # Core send / receive
    # -----------------------------------------------------------------------

    def send_command(self, cmd: str) -> str:
        """
        Send a command string (newline appended automatically) and return the
        stripped response line.  Raises RuntimeError on timeout.
        """
        message = cmd.strip() + "\n"
        self.ser.write(message.encode("ascii"))
        self.ser.flush()

        raw = self.ser.readline()
        if not raw:
            raise RuntimeError(f"Timeout waiting for response to '{cmd}'")
        return raw.decode("ascii", errors="replace").strip()

    # -----------------------------------------------------------------------
    # Convenience wrappers
    # -----------------------------------------------------------------------

    def read_adc(self, channel: int) -> int:
        """Perform an ADC measurement on *channel* (0-7). Returns integer."""
        if not (0 <= channel <= 7):
            raise ValueError("ADC channel must be 0-7")
        response = self.send_command(f"ADC:{channel}")
        # Expected format: "ADC:<value>"
        if response.startswith("ADC:"):
            return int(response.split(":")[1])
        raise RuntimeError(f"Unexpected ADC response: '{response}'")

    def set_led(self, index: int, state: bool) -> None:
        """Turn LED *index* (0-3) on or off."""
        if not (0 <= index <= 3):
            raise ValueError("LED index must be 0-3")
        response = self.send_command(f"LED:{index},{1 if state else 0}")
        if response != "OK":
            raise RuntimeError(f"LED command failed: '{response}'")

    def set_servo(self, angle: int) -> None:
        """Set servo angle in degrees (0-180)."""
        angle = max(0, min(180, int(angle)))
        response = self.send_command(f"SRV:{angle}")
        if response != "OK":
            raise RuntimeError(f"Servo command failed: '{response}'")

    def set_buzzer(self, freq_hz: int) -> None:
        """Set buzzer frequency in Hz (0 = off)."""
        freq_hz = max(0, int(freq_hz))
        response = self.send_command(f"BUZ:{freq_hz}")
        if response != "OK":
            raise RuntimeError(f"Buzzer command failed: '{response}'")


# ---------------------------------------------------------------------------
# Receive message with framing (Oppgave 8.6)
# ---------------------------------------------------------------------------

def receive_framed_message(ser: serial.Serial,
                           start_char: str = '<',
                           end_char: str = '>') -> str:
    """
    Buffer incoming characters until a complete framed message is received.
    A message starts with *start_char* and ends with *end_char*.

    Example frame:  <ADC:1024>
    Returns the content between the delimiters, e.g. "ADC:1024".
    """
    buffer = ""
    inside = False
    while True:
        byte = ser.read(1)
        if not byte:
            continue
        ch = byte.decode("ascii", errors="replace")
        if ch == start_char:
            inside = True
            buffer = ""
        elif ch == end_char and inside:
            return buffer
        elif inside:
            buffer += ch


# ---------------------------------------------------------------------------
# RS-485 multi-device message structure (Oppgave 8.5)
# ---------------------------------------------------------------------------

def build_rs485_message(device_id: int, cmd: str) -> str:
    """
    Build a simple RS-485 addressed message.

    Format:  @<id>:<CMD>:<ARG>!
      @  - start-of-frame
      id - 1-byte decimal device address (0-254)
      :  - separator
      !  - end-of-frame / checksum placeholder

    All devices read the bus; only the addressed device acts on the message.
    Device 255 is broadcast.

    Example:  "@3:ADC:2!"  -> device 3, read ADC channel 2
    """
    return f"@{device_id}:{cmd}!"


def parse_rs485_message(raw: str) -> tuple:
    """
    Parse a raw RS-485 message.  Returns (device_id, command_string) or
    raises ValueError if the frame is malformed.
    """
    if not (raw.startswith("@") and raw.endswith("!")):
        raise ValueError(f"Invalid RS-485 frame: '{raw}'")
    content = raw[1:-1]                 # strip @ and !
    colon_pos = content.index(":")      # first colon separates ID from rest
    device_id = int(content[:colon_pos])
    command   = content[colon_pos + 1:]
    return device_id, command


# ---------------------------------------------------------------------------
# Plotting helper (Oppgave 8 / 4.3.1)
# ---------------------------------------------------------------------------

def plot_adc_stream(card: IOCard, channel: int = 0,
                    n_samples: int = 100, interval: float = 0.1):
    """
    Read *n_samples* ADC values from *channel* and plot them with matplotlib.
    """
    try:
        import numpy as np
        from matplotlib import pyplot as plt
    except ImportError:
        print("numpy/matplotlib not installed – skipping plot")
        return

    values = []
    timestamps = []
    t0 = time.time()

    print(f"[plot] Collecting {n_samples} samples from ADC channel {channel}…")
    for _ in range(n_samples):
        val = card.read_adc(channel)
        values.append(val)
        timestamps.append(time.time() - t0)
        time.sleep(interval)

    t = np.array(timestamps)
    v = np.array(values)

    plt.figure(figsize=(10, 4))
    plt.plot(t, v, linewidth=1.2, color="steelblue")
    plt.xlabel("Time [s]")
    plt.ylabel("ADC value (12-bit)")
    plt.title(f"ADC channel {channel} – {n_samples} samples")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show(block=True)


# ---------------------------------------------------------------------------
# Demo / interactive test
# ---------------------------------------------------------------------------

def demo(card: IOCard):
    """Simple interactive demonstration of all IO functions."""
    print("\n--- IO-card demo ---")

    # ADC
    for ch in range(4):
        val = card.read_adc(ch)
        voltage = val * 3.3 / 4095
        print(f"  ADC ch{ch}: raw={val:4d}  ({voltage:.3f} V)")

    # LEDs – blink sequence
    print("  LED blink sequence…")
    for i in range(4):
        card.set_led(i, True)
        time.sleep(0.15)
    for i in range(4):
        card.set_led(i, False)
        time.sleep(0.15)

    # Servo sweep
    print("  Servo sweep 0 → 180 → 0°…")
    for angle in list(range(0, 181, 30)) + list(range(180, -1, -30)):
        card.set_servo(angle)
        time.sleep(0.05)

    # Buzzer
    print("  Buzzer 500 Hz for 0.5 s…")
    card.set_buzzer(500)
    time.sleep(0.5)
    card.set_buzzer(0)

    print("  Demo complete.\n")


def interactive_menu(card: IOCard):
    """Simple REPL for manual testing."""
    help_text = """
Commands:
  adc <ch>           - Read ADC channel (0-7)
  led <idx> <0|1>    - Set LED (0-3) off/on
  srv <angle>        - Set servo angle (0-180)
  buz <hz>           - Set buzzer freq Hz (0=off)
  plot [ch] [n]      - Stream-plot ADC channel (default ch=0, n=100)
  demo               - Run full demo
  raw <cmd>          - Send raw command string
  quit               - Exit
"""
    print(help_text)
    while True:
        try:
            line = input("io> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == "quit":
                break
            elif cmd == "adc":
                ch = int(parts[1]) if len(parts) > 1 else 0
                val = card.read_adc(ch)
                print(f"  ADC ch{ch} = {val}  ({val * 3.3 / 4095:.3f} V)")
            elif cmd == "led":
                idx   = int(parts[1])
                state = int(parts[2]) != 0
                card.set_led(idx, state)
                print(f"  LED{idx} -> {'ON' if state else 'OFF'}")
            elif cmd == "srv":
                angle = int(parts[1])
                card.set_servo(angle)
                print(f"  Servo -> {angle}°")
            elif cmd == "buz":
                hz = int(parts[1])
                card.set_buzzer(hz)
                print(f"  Buzzer -> {hz} Hz")
            elif cmd == "plot":
                ch = int(parts[1]) if len(parts) > 1 else 0
                n  = int(parts[2]) if len(parts) > 2 else 100
                plot_adc_stream(card, ch, n)
            elif cmd == "demo":
                demo(card)
            elif cmd == "raw":
                raw_cmd = " ".join(parts[1:])
                resp = card.send_command(raw_cmd)
                print(f"  Response: '{resp}'")
            elif cmd == "help":
                print(help_text)
            else:
                print(f"  Unknown command '{cmd}'. Type 'help'.")
        except Exception as exc:
            print(f"  Error: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS0"

    try:
        card = IOCard(port=port, baudrate=9600, timeout=2.0)
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
        sys.exit(1)

    try:
        interactive_menu(card)
    finally:
        card.close()
        print("Serial port closed.")