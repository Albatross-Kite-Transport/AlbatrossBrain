from evdev import list_devices, InputDevice, categorize, ecodes
from periphery import GPIO, PWM
import numpy as np
import time
import json
import serial
from pprint import pprint
import random



CENTER_TOLERANCE = 20
STICK_MAX = 255

speed_setpoint = 0.0
steering_setpoint = 0.0

rev_middle = GPIO("/dev/gpiochip4", 10, "out") # pin 18
rev_left = GPIO("/dev/gpiochip0", 6, "out") # pin 13
rev_right = GPIO("/dev/gpiochip4", 12, "out") # pin 22

rev_middle.write(False)
rev_left.write(False)
rev_right.write(False)

pwm_middle = PWM(1, 0)
pwm_left = PWM(0, 0)
pwm_right = PWM(2, 0)

pwm_middle.frequency = 1e7
pwm_left.frequency = 1e7
pwm_right.frequency = 1e7

pwm_middle.duty_cycle = 0.0
pwm_left.duty_cycle = 0.0
pwm_right.duty_cycle = 0.0

dev = InputDevice( list_devices()[0] )
axis = {
    ecodes.ABS_X: 'ls_x', # 0 - 255
    ecodes.ABS_Y: 'ls_y',
    ecodes.ABS_Z: 'rs_x',
    ecodes.ABS_RZ: 'rs_y',
    ecodes.ABS_BRAKE: 'lt',
    ecodes.ABS_GAS: 'rt',

    ecodes.ABS_HAT0X: 'dpad_x', # -1 - 1
    ecodes.ABS_HAT0Y: 'dpad_y'
}

center = {
    'ls_x': STICK_MAX/2,
    'ls_y': STICK_MAX/2,
    'rs_x': STICK_MAX/2,
    'rs_y': STICK_MAX/2
}

last = {
    'ls_x': STICK_MAX/2,
    'ls_y': STICK_MAX/2,
    'rs_x': STICK_MAX/2,
    'rs_y': STICK_MAX/2
}

try:
    for event in dev.read_loop():
        # calibrate zero on Y button
        # if event.type == ecodes.EV_KEY:
        #     if categorize(event).keycode[0] == "BTN_WEST":
        #         center['ls_x'] = last['ls_x']
        #         center['ls_y'] = last['ls_y']
        #         center['rs_x'] = last['rs_x']
        #         center['rs_y'] = last['rs_y']
        #         print( 'calibrated' )

        #read stick axis movement
        if event.type == ecodes.EV_ABS:
            if not event.code in [3, 4] and axis[ event.code ] in [ 'ls_x', 'ls_y', 'rs_x', 'rs_y' ]:
                last[ axis[ event.code ] ] = event.value
                value = (event.value - center[ axis[ event.code ] ]) / (STICK_MAX/2)
                if abs( value ) <= CENTER_TOLERANCE/(STICK_MAX/2):
                    value = 0
                if axis[event.code] in ['ls_y', 'rs_y']:
                    value = -value
                print(axis[event.code], " ", value)

                if axis[ event.code ] == 'ls_y':
                    speed_setpoint = value
                if axis[ event.code ] == 'rs_x':
                    steering_setpoint = value

                if axis[ event.code ] == 'ls_y' or axis[ event.code ] == 'rs_x':
                    # right_speed_unbounded = speed_setpoint + steering_setpoint
                    # left_speed_unbounded = speed_setpoint - steering_setpoint                      

                    pwm_middle.duty_cycle = abs(speed_setpoint)
                    pwm_left.duty_cycle = abs(speed_setpoint)
                    pwm_right.duty_cycle = abs(speed_setpoint)

                    rev_middle = speed_setpoint < 0
                    rev_left = speed_setpoint < 0
                    rev_right = speed_setpoint < 0
                    
                    left_speed = speed_setpoint - steering_setpoint
                    right_speed = speed_setpoint + steering_setpoint
                    if right_speed > 1.0
                        speed_setpoint = 1.0 - steering_setpoint
                    if left_speed < 1.0
                        speed_setpoint = -

                
except Exception as e:
    print(e)


if __name__ == "__main__":
    print ("Ready...")
    ser  = serial.Serial("COM3", baudrate= 9600, 
           timeout=2.5, 
           parity=serial.PARITY_NONE, 
           bytesize=serial.EIGHTBITS, 
           stopbits=serial.STOPBITS_ONE
        )
    data = {}
    data["operation"] = "sequence"

    data=json.dumps(data)
    print (data)
    if ser.isOpen():
        ser.write(data.encode('ascii'))
        ser.flush()
        try:
            incoming = ser.readline().decode("utf-8")
            print (incoming)
        except Exception as e:
            print (e)
            pass
        ser.close()
    else:
        print ("opening error")
