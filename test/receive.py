import serial, json
from time import time, sleep
import traceback

data = {}
incoming_json = ""

if __name__ == "__main__":
    ser  = serial.Serial("/dev/ttyUSB0", baudrate= 115200, 
        timeout=2.5, 
        parity=serial.PARITY_NONE, 
        bytesize=serial.EIGHTBITS, 
        stopbits=serial.STOPBITS_ONE
        )
    
    try:
        data["e"] = 1
        data["m"] = 0.0
        data["l"] = 0.0
        data["r"] = 0.0
        data["md"] = 0
        data_json=json.dumps(data)

        while True:
            print("sending...")
            if ser.isOpen():
                ser.write(data_json.encode('ascii'))
                ser.write("\n".encode('ascii'))
            else:
                print ("opening error")
            
            start_time = time()
            while time() - start_time < 0.2:
                if ser.inWaiting() > 0:
                    incoming_json = ser.readline().decode('utf-8')

                    print ("Incoming: \t", incoming_json)
                    try:
                        incoming_struct = json.loads(incoming_json)
                    except json.decoder.JSONDecodeError as _:
                        # print("unable to decode")
                        pass
        

    except Exception as e:
        print(e)
        traceback.print_exc()
        data["e"] = 0
        data["l"] = 0
        data["m"] = 0
        data["r"] = 0
        data_json=json.dumps(data)

        print("stopping motor...")
        if ser.isOpen():
            for i in range(10):
                ser.write(data_json.encode('ascii'))
                ser.write("\n".encode('ascii'))
                sleep(0.1)
        else:
            print("opening error")