from __future__ import annotations

import abc
import logging
import threading
import time

import serial
import serial.tools.list_ports


class StartButton(abc.ABC):
    def __init__(self) -> None:
        self.stop_threads = threading.Event()
        self.previous_state = 1
        self.thread = threading.Thread(target=self.loop, args=(self.find_port(),))
        self.thread.start()

    def find_port(self) -> str | None:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "ttyACM" in port.device:
                try:
                    with serial.Serial(port.device, 9600, timeout=1) as ser:
                        ser.readlines(3)
                        line = (
                            ser.readline()
                            .decode("utf-8", errors="ignore")
                            .strip()
                            .lower()
                        )
                        if "id=start_button" in line:
                            return port.device
                except serial.SerialException as e:
                    logging.error("Serial exception on %s: %s", port, e)
                except UnicodeDecodeError as e:
                    logging.error(
                        "Unicode exception while reading ID from %s: %s", port, e
                    )
                return None
        return None

    def loop(self, port: str) -> None:
        try:
            with serial.Serial(port, 9600, timeout=0.1) as ser:
                while not self.stop_threads.is_set():
                    try:
                        line = ser.readline().decode("utf-8", errors="ignore").strip()
                        if line:
                            state = int(line.split(",")[1].split("=")[1])
                            if self.previous_state == 1 and state == 0:
                                self.handle_event()
                            self.previous_state = state
                    except UnicodeDecodeError as e:
                        logging.error("Decode error on %s: %s", port, e)
                    time.sleep(0.01)  # Small sleep to reduce CPU usage
        except serial.SerialException as e:
            logging.error("Serial exception on %s: %s", port, e)

    def close(self) -> None:
        self.stop_threads.set()
        self.thread.join()

    @abc.abstractmethod
    def handle_event(self) -> None:
        pass
