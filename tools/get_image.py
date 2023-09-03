"""
File: get_image.py
Author: Xiang Lei
Created: 2023-07-22 21:50:25
Description: 
"""


import cv2
import os

cap = cv2.VideoCapture(0)

output_dir = 'output_images/'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

frame_count = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        if (frame_count % 10 == 0):
             cv2.imwrite(output_dir + 'frame_%d.jpg' % frame_count, frame)
        frame_count += 1

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()

cv2.destroyAllWindows()
