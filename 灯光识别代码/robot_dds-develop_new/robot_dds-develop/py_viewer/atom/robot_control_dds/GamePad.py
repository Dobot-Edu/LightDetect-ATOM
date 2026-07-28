from inputs import get_gamepad, UnpluggedError, devices

import threading
import time

from config import GamepadBasic
from config.Helper import get_gamepad_spec


class Gamepad:
    def __init__(self):
        self.keys = {k: 0 for k in GamepadBasic.key_names}
      
        self.gamepad_spec = get_gamepad_spec(devices.gamepads[0].name)

        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.setDaemon(True)

    def start(self):
        self.read_thread.start()

    def read_loop(self):
        """The read loop for events.
        This funnction should be executed in a separate thread for continuous
        event recording.
        """
        while True:
            try:
                events = get_gamepad()
            except UnpluggedError:
                print('Gamepad not connected! Please resart the program!')
                time.sleep(1)
                continue

            for event in events:
                self.update(event)

    def stop(self):
        self.is_running = False

    def reset(self):
        for k in self.keys.keys():
            self.keys[k] = 0

    def update(self, event):
        if event.ev_type in ['Key', 'Absolute']:
            # print(f'event.code: {event.code}, event.state: {event.state}')
            func = getattr(self.gamepad_spec, event.code, None)
            if func:
                func(self.keys, event.state)
                


if __name__ == '__main__':
    gamepad = Gamepad()
    gamepad.start()

    while True:
        time.sleep(0.1)
        print(gamepad.keys)
