import cv2
from time import sleep
from picamera import PiCamera
from gpiozero import LED

# Load Haar cascade for fire detection
fire_cascade = cv2.CascadeClassifier('fire_detection_cascade_model.xml')

# Initialize PiCamera
picam = PiCamera()
picam.resolution = (700, 700)

# GPIO motor pins (adjust as per your wiring)
m1 = LED(5)
m2 = LED(6)
m3 = LED(13)
m4 = LED(19)
pump = LED(26)

# Initially: robot moving forward, pump OFF
m1.on()
m2.off()
m3.on()
m4.off()
pump.off()

print("🔥 Fire detection system started...")

try:
    while True:
        # Capture image
        picam.capture('/tmp/ccc.jpg')
        frame = cv2.imread('/tmp/ccc.jpg')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect fire
        fire = fire_cascade.detectMultiScale(gray, 1.2, 5)

        if len(fire) > 0:
            print("🔥 Fire detected! Stopping robot and activating pump.")

            # Draw rectangle(s) around fire
            for (x, y, w, h) in fire:
                cv2.rectangle(frame, (x-20, y-20), (x+w+20, y+h+20), (0, 0, 255), 2)

            # Stop all motors
            m1.off()
            m2.off()
            m3.off()
            m4.off()

            # Turn on pump
            pump.on()

        else:
            print("✅ No fire. Robot running.")
            
            # Normal forward movement
            m1.on()
            m2.off()
            m3.on()
            m4.off()
            pump.off()

        # Show video feed
        cv2.imshow('Fire Detection', frame)

        # Break on 'q' key
        if cv2.waitKey(1) == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Program interrupted.")

finally:
    print("🔚 Cleaning up...")
    pump.off()
    m1.off()
    m2.off()
    m3.off()
    m4.off()
    picam.close()
    cv2.destroyAllWindows()
