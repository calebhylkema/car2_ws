"""Keyboard control for BRAKE_USB_HOLD_TEST_V1 only. No CAN or vehicle control."""
import argparse
import threading
import time


def command_for(keys):
    apply = 'a' in keys or 'space' in keys
    release = 'r' in keys
    if 'x' in keys or apply == release:
        return b'x'
    return b'a' if apply else b'r'


def main():
    import serial
    from pynput import keyboard
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    args = parser.parse_args()
    keys = set()
    lock = threading.Lock()
    quit_requested = threading.Event()

    def name(key):
        if key == keyboard.Key.space:
            return 'space'
        return (getattr(key, 'char', None) or '').lower()

    def press(key):
        if key == keyboard.Key.esc:
            quit_requested.set()
            return False
        with lock:
            keys.add(name(key))

    def release(key):
        with lock:
            keys.discard(name(key))

    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=0.1) as port:
        port.reset_input_buffer()
        port.write(b'x?')
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if 'BRAKE_USB_HOLD_TEST_V1' in port.readline().decode(errors='replace'):
                break
        else:
            raise RuntimeError('Brake HOLD diagnostic not found; no movement requested.')
        print('Hold A or Space = APPLY | Hold R = RELEASE | release = STOP | X = STOP | Esc = EXIT')
        print('Full power; automatic stops at measured raw limits 74 and 564.')
        port.timeout = 0
        buffer = b''
        listener = keyboard.Listener(on_press=press, on_release=release)
        listener.start()
        try:
            while not quit_requested.is_set() and listener.is_alive():
                with lock:
                    command = command_for(keys)
                port.write(command)
                buffer += port.read(port.in_waiting)
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    print(line.decode(errors='replace').strip())
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
            try:
                port.write(b'x')
                port.flush()
            except serial.SerialException:
                pass  # Firmware independently stops on timeout/USB loss.


if __name__ == '__main__':
    main()
