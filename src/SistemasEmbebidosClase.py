import RPi.GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN)

count = 0
while True:
    inputValue=GPIO.input(24)
    if (inputValue == True):
       count += 1
        print("Button Pressed: " + str(count)+ " times.")
        time.sleep(0.3)
    time.sleep(0.01)