from config import GamepadSpec


def get_gamepad_spec(name):
    print(name)
    if name == '8BitDo Ultimate C 2.4G Wireless Controller':
        return GamepadSpec.Gamepad_8BitDo_Ultimate_C_24G_Wireless_Controller()
    if name == 'Logitech Gamepad F710':
        return GamepadSpec.Gamepad_Logitech_Gamepad_F710()
    if name == 'BETOP BETOP XF1 BFM DONGLE':
        return GamepadSpec.Gamepad_BETOP_BETOP_XF1_BFM_DONGLE()
    
    if name == 'Logitech Logitech Cordless RumblePad 2':
        return GamepadSpec.Gamepad_Logitech_Logitech_Cordless_RumblePad_2()
    


    raise Exception(f'Gamepad {name} not found!!!')
