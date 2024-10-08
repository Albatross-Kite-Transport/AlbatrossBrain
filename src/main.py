from evdev import list_devices, InputDevice, categorize, ecodes
import numpy as np
import json
import serial
import traceback
from time import sleep, time
from threading import Thread


data_json = ""

def handle_serial():
    global data_json
    while True:
        sleep(0.2)
        if ser.isOpen():
            # print("outgoing ", data_json)
            ser.write(data_json.encode('ascii'))
            ser.write("\n".encode('ascii'))
        else:
            print ("opening error")
        start_time = time()
        while True:
            if ser.in_waiting > 0:  # Check if there is data waiting to be read
                incoming = ser.readline().decode('utf-8').strip()  # Read the line and decode it
                # print(incoming)
                # print("Received after ", time() - start_time)
                break  # Exit the loop once data is received
            elif time() - start_time > 0.1:  # Check if the timeout has been reached
                break  # Exit the loop if timeout is reached
            else:
                sleep(0.001)

def handle_joystick_inputs():
    global data_json
    CENTER_TOLERANCE = 0.0
    STICK_MAX = 255.0

    speed_setpoint = 0.0
    steering_setpoint = 0.0
    depower_setpoint = 0.0
    right_percentage = 0.0
    left_percentage = 0.0
    middle_percentage = 0.0

    pc = True
    if pc: device=2
    else: device = 0
    dev = InputDevice( list_devices()[device] )
    if pc:
        axis = {
            ecodes.ABS_X: 'ls_x', # 0 - 255
            ecodes.ABS_Y: 'ls_y',
            ecodes.ABS_RX: 'rs_x',
            ecodes.ABS_RY: 'rs_y',
            ecodes.ABS_BRAKE: 'lt',
            ecodes.ABS_GAS: 'rt',

            ecodes.ABS_HAT0X: 'dpad_x', # -1 - 1
            ecodes.ABS_HAT0Y: 'dpad_y'
        }
    else:
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
    max_speed = 1.0
    data = {}
    for event in dev.read_loop():
        #read stick axis movement
        if event != None and event.type == ecodes.EV_ABS:
            # print(event)
            if event.code in axis and axis[ event.code ] in [ 'ls_x', 'ls_y', 'rs_x', 'rs_y' ]:
                value = (event.value - STICK_MAX/2 ) / (STICK_MAX/2)
                if abs( value ) <= CENTER_TOLERANCE/(STICK_MAX/2):
                    value = 0
                if axis[event.code] in ['ls_y', 'rs_y']:
                    value = -value
                if axis[ event.code ] == 'ls_y':
                    speed_setpoint = value
                if axis[ event.code ] == 'rs_x':
                    steering_setpoint = value
                if axis[ event.code ] == 'rs_y':
                    # print(value)
                    depower_setpoint = value

                if axis[ event.code ] == 'ls_y' or axis[ event.code ] == 'rs_x' or axis[event.code] == 'rs_y':
                    right_percentage = np.clip(speed_setpoint - steering_setpoint + depower_setpoint, -1.0, 1.0)
                    left_percentage = np.clip(speed_setpoint + steering_setpoint + depower_setpoint, -1.0, 1.0)
                    middle_percentage = speed_setpoint

                    data["m"] = round(middle_percentage*max_speed, 4)
                    data["l"] = round(left_percentage*max_speed, 4)
                    data["r"] = round(right_percentage*max_speed, 4)
                    data["e"] = 1
                    # data["l"] = 0
                    # data["r"] = 0

                    # K_u = 0.2
                    # T_u = 1.0
                    # data["Kp"] = round(0.6*K_u, 2) # T_swing = 15.5/30.0, K_swing = 0.8
                    # data["Ki"] = round(2*data["Kp"]*T_u, 2)
                    # data["Kd"] = round(data["Kp"]*T_u / 8, 2)
                    # working: 0.0, 0.8, 0.01

                    data["Kp"] = 0.05
                    data["Ki"] = 0.5
                    data["Kd"] = 0.0
                    data["md"] = 1
                    data_json=json.dumps(data)
                    print(data_json)

if __name__ == "__main__":
    ser  = serial.Serial("/dev/ttyUSB0", baudrate= 115200, 
        timeout=2.5, 
        parity=serial.PARITY_NONE, 
        bytesize=serial.EIGHTBITS, 
        stopbits=serial.STOPBITS_ONE
        )
    try:
        print ("Ready...")

        serial_thread = Thread(target=handle_serial)
        joystick_thread = Thread(target=handle_joystick_inputs)

        serial_thread.start()
        joystick_thread.start()

    except KeyboardInterrupt:
        serial_thread.join()
        joystick_thread.join()
    except Exception as e:
        traceback.print_exc()
        print(e)
        ser.close()
