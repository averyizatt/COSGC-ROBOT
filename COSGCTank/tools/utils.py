import cv2

def resize_frame(frame, size):
    resized = cv2.resize(frame, size)
    return resized.astype('uint8')
