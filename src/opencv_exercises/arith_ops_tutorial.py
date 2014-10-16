#!/usr/bin/env python

import numpy as np 
import cv2

# x = np.uint8([250])
# y = np.uint8([10])

# print cv2.add(x,y)

# print x+y

img1 = cv2.imread('lena.png')
#print img1.size
img2 = cv2.imread('glass.jpg')[0:210, 0:210]
#print img2.size

#dst = cv2.addWeighted(img1, 0.7, img2, 0.3,0) #to do overlays!

#to do bitwise:
rows, cols, channels = img2.shape
roi = img1[0:rows, 0:cols]

img2gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
mask_inv = cv2.bitwise_not(mask)

# Now black-out the area of logo in ROI
img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)

# Take only region of logo from logo image.
img2_fg = cv2.bitwise_and(img2,img2,mask = mask)

# Put logo in ROI and modify the main image
dst = cv2.add(img1_bg,img2_fg)
img1[0:rows, 0:cols ] = dst


cv2.imshow('dst',img1)
cv2.waitKey(0)
cv2.destroyAllWindows()