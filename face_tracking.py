import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import RPi.GPIO as GPIO  # for the sensors


# # SENSOR SETUP
# servo_x_pin =
# servo_y_pin =
# sonic_trigger_pin =
# sonic_echo_pin =

def setup():
    global servo_X  # Do I need these glovbla, what should the logic be here?
    global servo_Y
    GPIO.setmode(GPIO.BOARD)       # Numbers GPIOs by physical location
    GPIO.setup(servo_x_pin, GPIO.OUT)
    GPIO.setup(servo_y_pin, GPIO.OUT)
    GPIO.setup(sonic_trigger_pin, GPIO.OUT)
    GPIO.setup(sonic_echo, GPIO.IN)
    GPIO.output(servo_x_pin, GPIO.LOW)
    GPIO.output(servo_y_pin, GPIO.LOW)
    servo_X = GPIO.PWM(servo_x_pin, 50)     # set Frequecy of pulse to 50Hz
    servo_Y = GPIO.PWM(servo_y_pin, 50)
    servo_X.start(0)
    servo_Y.start(0)

def servo_signal_wavelength(angle,
                 servo_max_duty=12.5,
                 servo_min_duty=2.5,
                 max_angle=180,
                 min_angle=0):
    return (servo_max_duty - servo_min_duty)*(angle-min_angle) / (max_angle - min_angle) + servo_min_duty

def servo_move(angle, servo):      # make the servo rotate to specific angle (0-180 degrees)
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    servo.ChangeDutyCycle(servo_signal_wavelength(angle)) # Changes the duty cycle to direct servo

def destroy():
    video_stream.release()
    cv2.destroyAllWindows()
    servo_X.stop()
    servo_Y.stop()
    GPIO.cleanup()


def servo_angle_to_pulse_ms(angle, servo_max_duty=12.5, servo_min_duty=2.5, max_angle=180, min_angle=180):
    return ((servo_max_duty-servo_min_duty)*(angle-min_angle))/(max_angle - min_angle) + servo_min_duty


def locate_faces(frame):
    # Converting to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detecting faces
    faces = face_classifier.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=10,
        minSize=(30, 30)
    )
    return faces

def draw_box(frame, faces):
    # Drawing rectangles around found faces
    for (x,y,w,h) in faces:
        cv2.rectangle(frame, (x,y), (x+h,y+w), (0,0,255), 2)
    return '{} rectangles drawn on frame'.format(len(faces))

def count_people(faces):
    return len(faces)

def move_back_to_centre(faces, frame):
    height, width, _  = frame.shape
    x,y,h,w = faces[0]
    mid_x, mid_y = x+ (h//2), y + (w//2)
    rel_x, rel_y = mid_x - (width//2) , mid_y - (height//2)
    return rel_x, rel_y


def map(value, fromLow, fromHigh, toLow, toHigh):
    return (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow) + toLow


def servo_setup():
    servoPin = 12
    global p
    GPIO.setmode(GPIO.BOARD)  # Numbers GPIOs by physical location
    GPIO.setup(servoPin, GPIO.OUT)  # Set servoPin's mode is output
    GPIO.output(servoPin, GPIO.LOW)  # Set servoPin to low

    p = GPIO.PWM(servoPin, 50)  # set Frequece to 50Hz
    p.start(0)  # Duty Cycle = 0


def servo_move(
        angle):  # make the servo rotate to specific angle (0-180 degrees)
    if (angle < 0):
        angle = 0
    elif (angle > 180):
        angle = 180
    p.ChangeDutyCycle(map(angle, 0, 180, SERVO_MIN_DUTY,
                          SERVO_MAX_DUTY))  # map the angle to duty cycle and output it


def loop():
    while True:
        for dc in range(0, 181, 1):  # make servo rotate from 0 to 180 deg
            servoWrite(dc)  # Write to servo
            time.sleep(0.001)
        time.sleep(0.5)
        for dc in range(180, -1, -1):  # make servo rotate from 180 to 0 deg
            servoWrite(dc)
            time.sleep(0.001)
        time.sleep(0.5)


def destroy(video_stream):
    p.stop()
    # STOP CAMERA AND CV2
    cv2.destroyAllWindows()
    GPIO.cleanup()
    video_stream.close()

def setup_cam()
# setting up the camera feed
camera = PiCamera()
# TODO: look up the cost payoffs for different resolutions and framerates. Maybe do some profiling...
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# small delay so the camera to warmup, TODO: find out why we need this line.
time.sleep(0.1)


# TODO: find the correct locations of the cascade filters on the rpi

face_cascade_path = 'cascade_filters/haarcascade_frontalface_default.xml'
face_classifier = cv2.CascadeClassifier(face_cascade_path)

video_stream = cv2.VideoCapture(0)

while True:
    # Taking frames from the live stream as images. return_code can tell us wether we have ran out of frames ect... But is not used with live feeds
    return_code, frame = video_stream.read()
    # run cascading algorithm and search for face positonal co-ordinates

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

    faces = locate_faces(image)
    # then we want to move the servo's based on this TODO: how do servos move?
    draw_box(image, faces)
    # Displaying the augmented feed, flipping in the horizontal axis to make the display seem like a mirror
    cv2.imshow('LIVE FEED', cv2.flip(image,1))
    key = cv2.waitKey(1)
    if len(faces) == 1:
        move_x, move_y = move_back_to_centre(faces, image)
        print(move_x, move_y)
    if key & 0xFF == 27:  # here 27 represents the esc key
        break

if __name__ == '__main__':     #Program start from here
    print('STARTING PROGRAM...')
    setup()
    print('SETUP COMPLETE...')
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()
