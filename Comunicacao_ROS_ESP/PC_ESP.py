import time
import json
import serial
import random

if __name__ == "__main__":
    print ("Ready...")
    ser  = serial.Serial("COM5", baudrate= 9600, 
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