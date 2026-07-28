class GamepadBase:
    def __init__(self, abs_max_stick, abs_max_trigger, stick_sign=False) -> None:
        self.abs_max_stick = abs_max_stick
        self.abs_max_trigger = abs_max_trigger
        self.stick_sign = stick_sign

    def prepare_base_functions(self):
        stick_offset = 0 if self.stick_sign else 1

        self.process_btn = lambda x: x
        self.process_trigger = lambda x: 1 if x > self.abs_max_trigger / 2 else 0
        self.process_d_pad_positive = lambda x: 1 if x > 0 else 0
        self.process_d_pad_negative = lambda x: 1 if x < 0 else 0
        self.process_stick = lambda x: (x-128)/128

        self.process_stick_lbm = lambda x: x / 255  - 0.5


class Gamepad_8BitDo_Ultimate_C_24G_Wireless_Controller(GamepadBase):
    def __init__(self) -> None:
        abs_max = 255

        super().__init__(abs_max_stick=abs_max, abs_max_trigger=abs_max)
        self.prepare_base_functions()

    def BTN_C(self, keys, value):
        keys['B'] = self.process_btn(value)

    def BTN_EAST(self, keys, value):
        keys['A'] = self.process_btn(value)

    def BTN_NORTH(self, keys, value):
        keys['X'] = self.process_btn(value)

    def BTN_WEST(self, keys, value):
        keys['Y'] = self.process_btn(value)

    def ABS_HAT0Y(self, keys, value):
        keys['DOWN'] = self.process_d_pad_positive(value)
        keys['UP'] = self.process_d_pad_negative(value)

    def ABS_HAT0X(self, keys, value):
        keys['RIGHT'] = self.process_d_pad_positive(value)
        keys['LEFT'] = self.process_d_pad_negative(value)

    def BTN_SELECT(self, keys, value):
        keys['SELECT'] = self.process_btn(value)

    def BTN_START(self, keys, value):
        keys['START'] = self.process_btn(value)

    def BTN_TL(self, keys, value):
        keys['LB'] = self.process_btn(value)

    def BTN_TR(self, keys, value):
        keys['RB'] = self.process_btn(value)

    def ABS_BRAKE(self, keys, value):
        keys['LT'] = self.process_trigger(value)

    def ABS_GAS(self, keys, value):
        keys['RT'] = self.process_trigger(value)

    def ABS_X(self, keys, value):
        keys['LX'] = self.process_stick(value)

    def ABS_Y(self, keys, value):
        keys['LY'] = -self.process_stick(value)

    def ABS_Z(self, keys, value):
        keys['RX'] = self.process_stick(value)

    def ABS_RZ(self, keys, value):
        keys['RY'] = -self.process_stick(value)


class Gamepad_Logitech_Gamepad_F710(GamepadBase):
    def __init__(self) -> None:
        abs_max_stick = 65535
        abs_max_trigger = 255

        super().__init__(abs_max_stick=abs_max_stick,
                         abs_max_trigger=abs_max_trigger, stick_sign=True)
        self.prepare_base_functions()

    def BTN_SOUTH(self, keys, value):
        keys['A'] = self.process_btn(value)

    def BTN_EAST(self, keys, value):
        keys['B'] = self.process_btn(value)

    def BTN_NORTH(self, keys, value):
        keys['X'] = self.process_btn(value)

    def BTN_WEST(self, keys, value):
        keys['Y'] = self.process_btn(value)

    def ABS_HAT0Y(self, keys, value):
        keys['DOWN'] = self.process_d_pad_positive(value)
        keys['UP'] = self.process_d_pad_negative(value)

    def ABS_HAT0X(self, keys, value):
        keys['RIGHT'] = self.process_d_pad_positive(value)
        keys['LEFT'] = self.process_d_pad_negative(value)

    def BTN_SELECT(self, keys, value):
        keys['SELECT'] = self.process_btn(value)

    def BTN_START(self, keys, value):
        keys['START'] = self.process_btn(value)

    def BTN_TL(self, keys, value):
        keys['LB'] = self.process_btn(value)

    def BTN_TR(self, keys, value):
        keys['RB'] = self.process_btn(value)

    def ABS_Z(self, keys, value):
        keys['LT'] = self.process_trigger(value)

    def ABS_RZ(self, keys, value):
        keys['RT'] = self.process_trigger(value)

    def ABS_X(self, keys, value):
        keys['LX'] = self.process_stick(value)

    def ABS_Y(self, keys, value):
        keys['LY'] = -self.process_stick(value)

    def ABS_RX(self, keys, value):
        keys['RX'] = self.process_stick(value)

    def ABS_RY(self, keys, value):
        keys['RY'] = -self.process_stick(value)


class Gamepad_BETOP_BETOP_XF1_BFM_DONGLE(GamepadBase):
    def __init__(self) -> None:
        abs_max_stick = 255
        abs_max_trigger = 255

        super().__init__(abs_max_stick=abs_max_stick,
                         abs_max_trigger=abs_max_trigger, stick_sign=True)
        self.prepare_base_functions()

    def BTN_SOUTH(self, keys, value):
        keys['A'] = self.process_btn(value)

    def BTN_EAST(self, keys, value):
        keys['B'] = self.process_btn(value)

    def BTN_NORTH(self, keys, value):
        keys['X'] = self.process_btn(value)

    def BTN_WEST(self, keys, value):
        keys['Y'] = self.process_btn(value)

    def ABS_HAT0Y(self, keys, value):
        keys['DOWN'] = self.process_d_pad_positive(value)
        keys['UP'] = self.process_d_pad_negative(value)

    def ABS_HAT0X(self, keys, value):
        keys['RIGHT'] = self.process_d_pad_positive(value)
        keys['LEFT'] = self.process_d_pad_negative(value)

    def BTN_SELECT(self, keys, value):
        keys['SELECT'] = self.process_btn(value)

    def BTN_START(self, keys, value):
        keys['START'] = self.process_btn(value)

    def BTN_TL(self, keys, value):
        keys['LB'] = self.process_btn(value)

    def BTN_TL2(self, keys, value):
        keys['LT'] = self.process_btn(value)

    def BTN_TR2(self, keys, value):
        keys['RT'] = self.process_btn(value)

    def BTN_TR(self, keys, value):
        keys['RB'] = self.process_btn(value)

    def ABS_Z(self, keys, value):
        keys['RX'] = self.process_stick(value) - 1

    def ABS_RZ(self, keys, value):
        keys['RY'] = -self.process_stick(value) + 1

    def ABS_X(self, keys, value):
        keys['LX'] = self.process_stick(value) - 1

    def ABS_Y(self, keys, value):
        keys['LY'] = -self.process_stick(value) + 1

    def ABS_RX(self, keys, value):
        keys['RX'] = self.process_stick(value)

    def ABS_RY(self, keys, value):
        keys['RY'] = -self.process_stick(value)

class Gamepad_Logitech_Logitech_Cordless_RumblePad_2(GamepadBase):
    
    def __init__(self) -> None:
        abs_max_stick = 65535
        abs_max_trigger = 255

        super().__init__(abs_max_stick=abs_max_stick,
                         abs_max_trigger=abs_max_trigger, stick_sign=True)
        
        self.prepare_base_functions()

    def BTN_SOUTH(self, keys, value):
        keys['X'] = self.process_btn(value)
    
    def BTN_NORTH(self, keys, value):
        keys['Y'] = self.process_btn(value)

    def BTN_EAST(self, keys, value):
        keys['A'] = self.process_btn(value)

    def BTN_C(self, keys, value):
        keys['B'] = self.process_btn(value)

    def BTN_WEST(self, keys, value):
        keys['LB'] = self.process_btn(value)

    def ABS_HAT0Y(self, keys, value):
        if value==-1:
            keys['UP'] = self.process_d_pad_positive(1)
            keys['DOWN'] = self.process_d_pad_negative(0)
        elif value==0:
            keys['UP'] = self.process_d_pad_negative(1)
            keys['DOWN'] = self.process_d_pad_negative(value)
        elif value==1:
            keys['DOWN'] = self.process_d_pad_positive(value)
            keys['UP'] = self.process_d_pad_negative(value)
        

    def ABS_HAT0X(self, keys, value):
        keys['RIGHT'] = self.process_d_pad_positive(value)
        keys['LEFT'] = self.process_d_pad_negative(value)

    def BTN_SELECT(self, keys, value):
        keys['SELECT'] = self.process_btn(value)

    def BTN_START(self, keys, value):
        keys['START'] = self.process_btn(value)

    def BTN_TL(self, keys, value):
        keys['LT'] = self.process_btn(value)

    def BTN_TR(self, keys, value):
        keys['RT'] = self.process_btn(value)
    
    def BTN_Z(self, keys, value):
        keys['RB'] = self.process_btn(value)

    def ABS_Z(self, keys, value):
        keys['LT'] = self.process_trigger(value)

    def ABS_RZ(self, keys, value):
        keys['RT'] = self.process_trigger(value)

    def ABS_X(self, keys, value):
        keys['LX'] = self.process_stick(value)

    def ABS_Y(self, keys, value):
        keys['LY'] = -self.process_stick(value)

    def ABS_Z(self, keys, value):
        keys['RX'] = self.process_stick(value)

    def ABS_RZ(self, keys, value):
        keys['RY'] = -self.process_stick(value)
    
    def BTN_TR2(self, keys, value):
        keys['START'] = self.process_btn(value)

