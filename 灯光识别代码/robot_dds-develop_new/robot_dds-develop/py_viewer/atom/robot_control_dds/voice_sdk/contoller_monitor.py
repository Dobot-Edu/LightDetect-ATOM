import time
from robot_dds.robot_control_dds.control_sim import Control_sim
from threading import Event, Lock, Thread
from voice_sdk.signal_slot import DoSignal


class ControllerMonitor:
    def __init__(self):
        # dds init
        self.dds = Control_sim()
        self.dds.start()
        self.current_state = 0
        self.last_state = 0
        self.stop_thread = Event()
        self.lock = Lock()
        self.thread_button = Thread(target=self.run_thread_button)
        self.thread_button.daemon = True
        self.thread_button.start()
        self.signal = DoSignal()

    def run_thread_button(self):
        while 1:
            time.sleep(0.02)
            tmp = list(self.dds.msg.wireless_remote)[2]
            if tmp == 36:
                self.current_state=1
            else:
                self.current_state=0

            if self.current_state==1:
                self.signal.emit("button on", status=1)
            if self.current_state-self.last_state==-1:
                self.signal.emit("button off", status=0)
            self.last_state = self.current_state


def dobot_print(message, **kwargs):
    print("print: ", message, kwargs)


if __name__ == "__main__":
    aaa = ControllerMonitor()
    aaa.signal.connect(dobot_print)
    while 1:
        time.sleep(100)