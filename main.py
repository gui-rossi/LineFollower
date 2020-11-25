import numpy as np
import cv2
from threading import Thread
import requests, time, imutils, signal, sys
import display, MFRC522
import RPi.GPIO as GPIO
from imutils.video.pivideostream import PiVideoStream

continue_reading = True
authorized_wards = {
    '1': "105,238,137,194,105",
    '2': "174,30,182,198"
}

GPIO.setmode(GPIO.BCM)

motorAenable = 18
motorA1 = 15
motorA2 = 14

motorBenable = 16
motorB1 = 20
motorB2 = 21

GPIO.setup(motorAenable, GPIO.OUT)
GPIO.setup(motorA1, GPIO.OUT)
GPIO.setup(motorA2, GPIO.OUT)
GPIO.setup(motorBenable, GPIO.OUT)
GPIO.setup(motorB1, GPIO.OUT)
GPIO.setup(motorB2, GPIO.OUT)

motorAPWM = GPIO.PWM(motorAenable, 500)
motorBPWM = GPIO.PWM(motorBenable, 500)
motorAPWM.start(0)
motorBPWM.start(0)


def stop():
    GPIO.output(motorA1, False)
    GPIO.output(motorA2, False)
    motorAPWM.ChangeDutyCycle(0)
    GPIO.output(motorB1, False)
    GPIO.output(motorB2, False)
    motorBPWM.ChangeDutyCycle(0)
    print("motor stopped.")


def goForward(duty):
    GPIO.output(motorA1, True)
    GPIO.output(motorA2, False)
    GPIO.output(motorB1, True)
    GPIO.output(motorB2, False)
    motorAPWM.ChangeDutyCycle(duty)
    motorBPWM.ChangeDutyCycle(duty)
    print("direction: forward.\nduty:", duty)


def goReverse(duty):
    GPIO.output(motorA1, False)
    GPIO.output(motorA2, True)
    GPIO.output(motorB1, False)
    GPIO.output(motorB2, True)
    motorAPWM.ChangeDutyCycle(duty)
    motorBPWM.ChangeDutyCycle(duty)
    print("direction: reverse.\nduty:", duty)
    time.sleep(0.5)
    stop()
    time.sleep(0.5)


def turnRight(duty):
    GPIO.output(motorA1, True)
    GPIO.output(motorA2, False)
    GPIO.output(motorB1, False)
    GPIO.output(motorB2, True)
    motorAPWM.ChangeDutyCycle(duty)
    motorBPWM.ChangeDutyCycle(duty)
    time.sleep(0.3)
    stop()
    time.sleep(0.2)


def turnLeft(duty):
    GPIO.output(motorA1, False)
    GPIO.output(motorA2, True)
    GPIO.output(motorB1, True)
    GPIO.output(motorB2, False)
    motorAPWM.ChangeDutyCycle(duty)
    motorBPWM.ChangeDutyCycle(duty)
    time.sleep(0.3)
    stop()
    time.sleep(0.2)


'''
VERMELHO: 172 > H > 181, 127 < S < 202, 75 < V < 140
VERDE: 70 > H > 84, 234 < S < 255, 116 < V < 170
AZUL: 90 < H < 104, 244 < S < 255, 141 < V < 219
AMARELO: 33 < H < 55,  113 < S < 206, 127 < V < 221
'''
red_l = np.array([152, 107, 55])
red_h = np.array([211, 232, 170])

green_l = np.array([70, 234, 116])
green_h = np.array([84, 255, 170])

blue_l = np.array([90, 244, 141])
blue_h = np.array([104, 255, 219])

yellow_l = np.array([33, 113, 127])
yellow_h = np.array([55, 206, 221])

apiUrl = "https://us-central1-medicinedeployer.cloudfunctions.net/api/robot"

cx = 0
stopGlobal = False

def cameraInit():
    print("initializing camera")
    frameSize = (160, 128)
    cam = PiVideoStream(resolution=frameSize, framerate=32).start()
    time.sleep(2)
    print("initialized camera")
    return cam


def waitButtonWart():
    # @TODO implement logic

    print("waiting for the button to be pressed")


def turn180Degrees():
    # @TODO implement logic
    GPIO.output(motorA1, False)
    GPIO.output(motorA2, True)
    GPIO.output(motorB1, True)
    GPIO.output(motorB2, False)
    motorAPWM.ChangeDutyCycle(100)
    motorBPWM.ChangeDutyCycle(100)
    time.sleep(1.3)
    stop()
    print("turned 180 degrees")


def getSentOrder():
    while True:
        request = requests.get(apiUrl + '/getSentOrders').json()
        if not request:
            print('wait')
            time.sleep(1)
        else:
            print("Aisle:", request[0]["aisle"])
            print("Order:", request[0]["orderId"])
            return request[0]


def updateOrderToDelivering(orderId):
    while True:
        request = requests.post(apiUrl + '/deliveringOrder/' + orderId)
        if request.status_code == 200:
            return True


def updateOrderToDelivered(orderId, deliveredAt, nurseThatReceived):
    # payload = {'deliveredAt': deliveredAt, 'nurse' : nurseThatReceived} #data em  UTC!!
    # trocar na linha 27:
    # request = requests.post(apiUrl + '/orderDelivered/' + orderId, data=json.dumps(payload))
    while True:
        request = requests.post(apiUrl + '/orderDelivered/' + orderId)
        if request.status_code == 200:
            return True


def qrCode(frame):
    # define a video input to analyse
    # cap = camera.read()
    # initialize the qr code function
    detector = cv2.QRCodeDetector()
    # capture a camera frame
    img = frame
    # return only the qr code data
    data, *_ = detector.detectAndDecode(img)
    # return true and the data if detected, otherwise false
    if data:
        print("QR code data: ", data)
        return True
    else:
        return False


def followColor(low_hsv, high_hsv):
    #     video_capture = cv2.VideoCapture(0)
    #     video_capture.set(3, 160)
    #     video_capture.set(4, 120)

    # Capture the frames
    # ret, frame = video_capture.read()
    frame = camera.read()

    if frame is None:
        print("no frame")
        return False
    # check if there is a QR code
    if qrCode(frame):
        print("Found QR Code")
        return True
    #         cv2.imwrite('frame.png', frame)
    cv2.imshow('original frame', frame)

    # crop the actual frame
    # frame = frame[60:120, 0:160]

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rgb_frame = frame
    print (rgb_frame)
    # dinstinguish the line
    mask = cv2.inRange(hsv_frame, low_hsv, high_hsv)
    frame = cv2.bitwise_and(frame, frame, mask=mask)

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Find the contours of the frame
    contours, hierarchy = cv2.findContours(blur.copy(), 1, cv2.CHAIN_APPROX_NONE)

    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M['m00'] == 0:
            return False

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        cv2.line(frame, (cx, 0), (cx, 720), (255, 0, 0), 1)
        cv2.line(frame, (0, cy), (1280, cy), (255, 0, 0), 1)

        cv2.drawContours(frame, contours, -1, (0, 255, 0), 1)

    else:
        print("I don't see the line")
        stop()

    return False

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


lcd = display.LCDInit()
camera = cameraInit()

class ImageProcess(Thread):

    def __init__(self, low_hsv, high_hsv):
        Thread.__init__(self)
        self.low_hsv = low_hsv
        self.high_hsv = high_hsv

    def run(self):
        while True:
            if followColor(self.low_hsv, self.high_hsv):
                break
            time.sleep(0)

        stopGlobal = False



class MotorControl(Thread):

    def __init__(self):
        Thread.__init__(self)

    def run(self):
        while stopGlobal:
            # controle dos motores
            if cx >= 100:
                print("Turn Left!")
                turnLeft(90)

            if 100 > cx > 70:
                print("On Track!")
                goForward(80)

            if cx <= 70:
                print("Turn Right")
                turnRight(90)

            time.sleep(0)

def main():
    try:
        while True:
            # waiting order in pharmacy
            orderId = getSentOrder()

            # got order, update it
            print("order", orderId["aisle"])
            lcd.message('Order received\n')
            lcd.message('Ward number: ')
            lcd.message(orderId["aisle"])
            #         updateOrderToDelivering(orderId["orderId"])

            # updated order, follow line to wart
            # 3 == RED, 2 == GREEN, 1 == BLUE, 4 == YELLOW
            stopGlobal = True
            if orderId["aisle"] == "3":
                t1 = ImageProcess(red_l, red_h)
            elif orderId["aisle"] == "2":
                t1 = ImageProcess(green_l, green_h)
            elif orderId["aisle"] == "1":
                t1 = ImageProcess(blue_l, blue_h)
            elif orderId["aisle"] == "4":
                t1 = ImageProcess(yellow_l, yellow_h)
            # crio a thread de controle dos motores
            t2 = MotorControl()

            # starto a thread
            t2.start()
            t1.start()
            # espero a thread t1 acabar para continuar essa
            t1.join()

            time.sleep(2)

            turn180Degrees()

            # arrived at X wart, wait for the button to be pressed
            waitButtonWart()

            # button pressed: update order, wait 5 seconds, turn 180 degrees and go back following the same line until another qrCode is seen
            updateOrderToDelivered(orderId["orderId"], orderId["aisle"], "Nome x")

            time.sleep(5)

            stopGlobal = True
            if orderId["aisle"] == "1":
                t1 = ImageProcess(red_l, red_h)
            elif orderId["aisle"] == "2":
                t1 = ImageProcess(green_l, green_h)
            elif orderId["aisle"] == "3":
                t1 = ImageProcess(blue_l, blue_h)
            elif orderId["aisle"] == "4":
                t1 = ImageProcess(yellow_l, yellow_h)

            # crio a thread de controle dos motores
            t2 = MotorControl()

            # starto a thread
            t2.start()
            t1.start()
            # espero a thread t1 acabar para continuar essa
            t1.join()

            turn180Degrees()
            # restart
    except:
        print("Unexpected error:", sys.exc_info()[0])
        stop()


if __name__ == "__main__":
    main()
