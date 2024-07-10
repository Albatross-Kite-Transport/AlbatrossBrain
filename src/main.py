from evdev import list_devices, InputDevice, categorize, ecodes
import numpy as np
import json
import serial
import traceback
from time import sleep


CENTER_TOLERANCE = 0.0
STICK_MAX = 255.0

speed_setpoint = 0.0
steering_setpoint = 0.0
depower_setpoint = 0.0

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

if __name__ == "__main__":
    ser  = serial.Serial("/dev/ttyUSB0", baudrate= 115200, 
        timeout=2.5, 
        parity=serial.PARITY_NONE, 
        bytesize=serial.EIGHTBITS, 
        stopbits=serial.STOPBITS_ONE
        )
    try:
        print ("Ready...")
        data = {}
        while True:
            event = dev.read_one()
            #read stick axis movement
            if event != None and event.type == ecodes.EV_ABS:
                if not event.code in [3, 4] and axis[ event.code ] in [ 'ls_x', 'ls_y', 'rs_x', 'rs_y' ]:
                    value = (event.value - STICK_MAX/2 ) / (STICK_MAX/2)
                    if abs( value ) <= CENTER_TOLERANCE/(STICK_MAX/2):
                        value = 0
                    if axis[event.code] in ['ls_y', 'rs_y']:
                        value = -value
                    if axis[ event.code ] == 'ls_y': # temporarily switch ls and rs
                        speed_setpoint = value
                    if axis[ event.code ] == 'rs_x':
                        steering_setpoint = value
                    if axis[ event.code ] == 'rs_y':
                        depower_setpoint = value

                    if axis[ event.code ] == 'ls_y' or axis[ event.code ] == 'rs_x' or axis[event.code] == 'rs_y':
                        right_percentage = np.clip(speed_setpoint + steering_setpoint + depower_setpoint, -1.0, 1.0)
                        left_percentage = np.clip(speed_setpoint - steering_setpoint + depower_setpoint, -1.0, 1.0)
                        middle_percentage = speed_setpoint

                        data["m"] = round(middle_percentage, 4)
                        data["l"] = round(left_percentage, 4)
                        data["r"] = round(right_percentage, 4)
                        data["e"] = 1

                        data_json=json.dumps(data)
                        # print (data_json)
                        if ser.isOpen():
                            ser.write(data_json.encode('ascii'))
                            ser.write("\n".encode('ascii'))
                        else:
                            print ("opening error")
            if ser.inWaiting() > 0:
                incoming = ser.readline().decode('utf-8')
                print ("incoming", incoming)
                # decode incoming into pandas, plot graph, figure out whats happening.
    except Exception as e:
        traceback.print_exc()
        print(e)
        ser.close()
