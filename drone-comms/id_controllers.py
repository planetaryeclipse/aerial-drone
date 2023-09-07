from evdev import InputDevice, categorize, ecodes, list_devices
from pprint import pprint
import time

devices = [InputDevice(path) for path in list_devices()]
for device in devices:
    print(device.path, device.name, device.phys)

dev = InputDevice('/dev/input/event4')
# time.sleep(0.1)
dev.close()

dev = InputDevice('/dev/input/event4')
# time.sleep(0.1)  # need delay before accessing active keys
# print(dev)

print()

pprint(dev.capabilities(verbose=True))
time.sleep(0.1)

print()

pprint(dev.capabilities())
time.sleep(0.1)
print()

print(dev.active_keys(verbose=True))
time.sleep(0.1)

dev.close()

print()

exit()

for event in dev.read_loop():
    if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:
        print(f'button: name={event.type}, code={event.code}, val={event.value}')
        print(categorize(event))

        # Buttons
        if event.code == ecodes.BTN_A:
            pass
        elif event.code == ecodes.BTN_B:
            pass
        elif event.code == ecodes.BTN_X:
            pass
        elif event.code == ecodes.BTN_Y:
            pass
        elif event.code == ecodes.BTN_TR:
            pass  # top right trigger
        elif event.code == ecodes.BTN_TL:
            pass
        elif event.code == ecodes.BTN_THUMBR:
            pass  # right thumbstick button
        elif event.code == ecodes.BTN_THUMBL:
            pass  # left thumbstick button
        elif event.code == ecodes.BTN_START:
            pass # start button (right)
        elif event.code == ecodes.BTN_SELECT:
            pass # select button (left)
        elif event.code == ecodes.BTN_MODE:
            pass  # xbox button

        # Axes
        if event.code == ecodes.ABS_HAT0X:
            if event.value == 1:
                print('dpad right')
            elif event.value == 0:
                print('dpad horiz center')
            elif event.value == -1:
                print('dpad left')
        elif event.code == ecodes.ABS_HAT0Y:
            # is inverted for whatever reason
            if event.value == 1:
                print('dpad down')
            elif event.value == 0:
                print('dpad vertical center')
            elif event.value == -1:
                print('dpad up')
        elif event.code == ecodes.ABS_X:
            pass  # left joystick
        elif event.code == ecodes.ABS_Y:
            pass  # left joystick
        elif event.code == ecodes.ABS_RX:
            pass
        elif event.code == ecodes.ABS_RY:
            pass
        elif event.code == ecodes.ABS_Z:
            pass
        elif event.code == ecodes.ABS_RZ:
            pass  # right bumper

