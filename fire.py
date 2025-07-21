import tensorflow.keras
from PIL import Image, ImageOps
import numpy as np
from picamera import PiCamera
from time import sleep
import os
#import RPi.GPIO as GPIO
from gpiozero import LED
from time import sleep
from gpiozero import MCP3008 
import serial
import time
import requests
import sys
from gpiozero import Servo


servo = Servo(21)

recipient="9445116830"
ser = serial.Serial('/dev/ttyS0', baudrate=9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                    )

ser.write('AT\r\n'.encode())
sleep(1)
ser.write('AT+CMGF=1\r\n'.encode())
sleep(1)



m1 = LED(5)
m2 = LED(6)
m3 = LED(13)
m4 = LED(19)
pump = LED(26)

m1.on()
m2.off()
m3.on()
m4.off()
pump.off()


i=0
j=0

camera = PiCamera()
camera.start_preview()
sleep(1)
camera.stop_preview()


model = tensorflow.keras.models.load_model('keras_model.h5')

class_names = open("labels.txt", "r").readlines()


while True:
    
        wt = MCP3008(0)
        WT = (wt.value * 100)
        print('wt=' + str(round(WT)))
     
        camera.start_preview(fullscreen=False,window=(100,100,640,480))
        camera.capture('seed.jpg')
        
        np.set_printoptions(suppress=True)

        data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)

        image = Image.open('seed.jpg')

        size = (224, 224)
        image = ImageOps.fit(image, size, Image.ANTIALIAS)

        image_array = np.asarray(image)


        normalized_image_array = (image_array.astype(np.float32) / 127.0) - 1

        data[0] = normalized_image_array

        prediction = model.predict(data)
        ynew = model.predict_classes(data)
        print(prediction)
        
        index = np.argmax(prediction)
        class_name = class_names[index]
        confidence_score = prediction[0][index]

        print("Class:", class_name[2:], end="")
        print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")
        
        print("Class:", class_name[2:], end="")
        print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")

        
        if ynew == [0]:
                print(" ")
        if ynew == [1]:
                print(" fire ")
                m1.off()
                m2.off()
                m2.off()
                m2.off()
                pump.on()
                servo.min()
                sleep(1)
                servo.mid()
                sleep(1)
                servo.max()
                sleep(1)
                print(Servo)
                sleep(2)
                sleep(1)
                print('Sending SMS')
                ser.write('''AT+CMGS="'''.encode() + recipient.encode() + '''"\r'''.encode())
                sleep(1)
                ser.write("Fire Alert".encode())
                sleep(1)
                ser.write(chr(26).encode())
                sleep(1)
                pump.off()
        print(ynew)