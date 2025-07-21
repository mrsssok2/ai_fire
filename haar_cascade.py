import cv2
from picamera import PiCamera
from gpiozero import LED
from gpiozero import Servo
from time import sleep

servo = Servo(21)
m1 = LED(5)
pump = LED(26)
m1.off()
pump.off()

fire_cascade = cv2.CascadeClassifier('fire_detection_cascade_model.xml') 

picam = PiCamera()
picam.resolution = (700, 700)

while True:
    frame = picam.capture('/tmp/ccc.jpg')
    frame = cv2.imread('/tmp/ccc.jpg')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    fires = fire_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    for (x, y, w, h) in fires:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
    cv2.imshow('Fire Detection', frame)
    
    if len(fires) > 0:
        m1.off()
        pump.on()
        servo.min()
        sleep(1)
        servo.mid()
        sleep(1)
        servo.max()
        sleep(1)
    else:
        m1.on()
        pump.off()
        
    if cv2.waitKey(1) == ord('q'):
        break

picam.close()
cv2.destroyAllWindows()