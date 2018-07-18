import cv2

video_stream = cv2.VideoCapture(0)

while True:
    # Taking frames from the live stream as images. return_code can tell us wether we have ran out of frames ect... But is not used with live feeds
    return_code, frame = video_stream.read()
    # run cascading algorithm and search for face positonal co-ordinates
    # Displaying the augmented feed, flipping in the horizontal axis to make the display seem like a mirror
    cv2.imshow('LIVE', cv2.flip(frame,1))
    key = cv2.waitKey(1)
    # print(count_people(faces))
    # print(frame.shape)
    if key & 0xFF == 27:  # here 27 represents the esc key
        break


video_stream.release()
cv2.destroyAllWindows()