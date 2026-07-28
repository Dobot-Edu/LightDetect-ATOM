import time


class DoSignal:
    def __init__(self):
        self._slots = []

    def connect(self, slot):
        if slot not in self._slots:
            self._slots.append(slot)

    def disconnect(self, slot):
        if slot in self._slots:
            self._slots.remove(slot)

    def emit(self, *args, **kwargs):
        for slot in self._slots:
            slot(*args, **kwargs)


class Button:
    def __init__(self):
        self.clicked = DoSignal()

    def simulate_click(self):
        print("Button clicked")
        self.clicked.emit("Button data", key="value", aaa= 666)

    def slot(self, message, **kwargs):
        time.sleep(1)
        print("111: ", message, kwargs)


if __name__ == "__main__":
    btn = Button()

    btn.clicked.connect(btn.slot)

    while 1:
        btn.simulate_click()
