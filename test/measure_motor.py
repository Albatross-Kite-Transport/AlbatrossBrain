import serial, json
from time import time, sleep
import pandas as pd

data = {}
incoming_json = ""
incoming = pd.DataFrame()
measurement_data = pd.DataFrame()
motor = "m"

if __name__ == "__main__":
    ser  = serial.Serial("/dev/ttyUSB0", baudrate= 115200, 
        timeout=2.5, 
        parity=serial.PARITY_NONE, 
        bytesize=serial.EIGHTBITS, 
        stopbits=serial.STOPBITS_ONE
        )
    
    try:
        # send max speed
        data[motor] = 1.0
        data["e"] = 1
        data_json=json.dumps(data)
        # print (data_json)

        # receive data
        start_time = time()
        start_receive_time = time()
        stopped = False
        first_measure = True
        first_measure_time = 0.0
        while not stopped:
            if time() - start_time < 1.0 or time() - start_time > 5.0:
                data[motor] = 0.0
                data["e"] = 0
                data["md"] = 0
                data_json = json.dumps(data)
            else:
                data[motor] = 1.0
                data["e"] = 1
                data["md"] = 0
                data_json = json.dumps(data)
            if ser.isOpen():
                print("writing ", data)
                ser.write(data_json.encode('ascii'))
                ser.write("\n".encode('ascii'))
            else:
                print ("opening error")
            while time() - start_receive_time < 0.01:
                pass
            start_receive_time = time()
            stopped = time() - start_time > 6.0
            while True:
                if ser.inWaiting() > 0:
                    incoming_json = ser.readline().decode('utf-8')
                    incoming_struct = json.loads(incoming_json)
                    if first_measure:
                        first_measure_time = incoming_struct["t"]
                        first_measure = False
                    incoming_struct["time"] = incoming_struct["t"] - first_measure_time
                    print ("incoming", incoming_json)
                    if incoming_struct[motor] == 1.0:
                        incoming = pd.DataFrame([incoming_struct])
                        measurement_data = pd.concat([measurement_data, incoming], ignore_index=True)
                        print(measurement_data.head())
                    break
                elif time() - start_receive_time > 0.01:  # Check if the timeout has been reached
                    print("No data received within 0.01 seconds.")
                    break  # Exit the loop if timeout is reached
            # print(stopped)
        
        # # stop motor
        # data[motor] = 0.0
        # data_json=json.dumps(data)
        # # print (data_json)
        # sleep(1.0)
        # for _ in range(100):
        #     if ser.isOpen():
        #         ser.write(data_json.encode('ascii'))
        #         ser.write("\n".encode('ascii'))
        #     else:
        #         print ("opening error")

        measurement_data.to_csv("measurement_data1.csv")
    except Exception as e:
        print(e)
        # stop motor
        data[motor] = 0.0
        data_json=json.dumps(data)
        # print (data_json)
        for _ in range(100):
            if ser.isOpen():
                ser.write(data_json.encode('ascii'))
                ser.write("\n".encode('ascii'))
            else:
                print ("opening error")
        ser.close()