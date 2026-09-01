import cv2
import numpy as np

image = cv2.imread('/home/pi/Test/1.png',0)
cv2.imwrite('/home/pi/Test/1.png',cv2.Canny(image,200,300))
cv2.imshow('1',cv2.imread('/home/pi/Test/1.png'))
cv2.waitKey()
cv2.destroyALLWindows()
