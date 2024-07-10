import serial, json
from time import time
import pandas as pd

data = {}
incoming_json = ""
incoming = pd.DataFrame()
measurement_data = pd.DataFrame()

if __name__ == "__main__":
    ser  = serial.Serial("/dev/ttyUSB0", baudrate= 115200, 
        timeout=2.5, 
        parity=serial.PARITY_NONE, 
        bytesize=serial.EIGHTBITS, 
        stopbits=serial.STOPBITS_ONE
        )
    
    try:
        # send max speed
        data["l"] = 1.0
        data_json=json.dumps(data)
        # print (data_json)
        if ser.isOpen():
            ser.write(data_json.encode('ascii'))
            ser.write("\n".encode('ascii'))
        else:
            print ("opening error")

        # receive data
        start_time = time()
        while time() - start_time < 1.0:
            if ser.inWaiting() > 0:
                incoming_json = ser.readline().decode('utf-8')
                # print ("incoming", incoming)
                incoming = pd.Dataframe([json.loads(incoming_json)])
                output = pd.concat([measurement_data, incoming], ignore_index=True)
                print(output.head())
        
        # stop motor
        data["l"] = 0.0
        data_json=json.dumps(data)
        # print (data_json)
        for _ in range(100):
            if ser.isOpen():
                ser.write(data_json.encode('ascii'))
                ser.write("\n".encode('ascii'))
            else:
                print ("opening error")

        measurement_data.to_csv("measurement_data.csv")

        measurement_data.plot(x="t")

    except Exception as _:
        # stop motor
        data["l"] = 0.0
        data_json=json.dumps(data)
        # print (data_json)
        for _ in range(100):
            if ser.isOpen():
                ser.write(data_json.encode('ascii'))
                ser.write("\n".encode('ascii'))
            else:
                print ("opening error")
        ser.close()