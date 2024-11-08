import time
import json
import serial
import random

def send_data(data):
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

def main():
    print ("Ready...")
    global ser
    ser = serial.Serial("COM5", baudrate= 9600, 
           timeout=2.5, 
           parity=serial.PARITY_NONE, 
           bytesize=serial.EIGHTBITS, 
           stopbits=serial.STOPBITS_ONE
        )
    while True:
        data = {}

        data["motor1"] = int(input("m1"))
        data["motor2"] = int(input("m2"))
        data["motor3"] = int(input("m3"))
        data["motor4"] = int(input("m4"))

        send_data(data)

if __name__ == "__main__":
    main()