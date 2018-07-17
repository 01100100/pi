import cv2
import os
import time


# TODO: def SetAngle(angle):
# 	duty = angle / 18 + 2
# 	GPIO.output(03, True)
# 	pwm.ChangeDutyCycle(duty)
# 	sleep(1)
# 	GPIO.output(03, False)
# 	pwm.ChangeDutyCycle(0)

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

face_cascade_path = 'C:/Users/david/PycharmProjects/face/venv/Lib/site-packages/cv2/data/haarcascade_frontalface_default.xml'
face_classifier = cv2.CascadeClassifier(face_cascade_path)

video_stream = cv2.VideoCapture(0)

while True:
    # Taking frames from the live stream as images. return_code can tell us wether we have ran out of frames ect... But is not used with live feeds
    return_code, frame = video_stream.read()
    # run cascading algorithm and search for face positonal co-ordinates
    faces = locate_faces(frame)
    # then we want to move the servo's based on this TODO: how do servos move?
    # servo_v =
    # servo_h =
    draw_box(frame, faces)
    # Displaying the augmented feed, flipping in the horizontal axis to make the display seem like a mirror
    cv2.imshow('LIVE', cv2.flip(frame,1))
    key = cv2.waitKey(1)
    # print(count_people(faces))
    # print(frame.shape)
    if len(faces) == 1:
        move_x, move_y = move_back_to_centre(faces, frame)
        print(move_x, move_y)
    if key & 0xFF == 27:  # here 27 represents the esc key
        break


video_stream.release()
cv2.destroyAllWindows()
pi 