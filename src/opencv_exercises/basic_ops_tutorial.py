#!/usr/bin/env python

import cv2
import numpy as np 
from matplotlib import pyplot as plt

# img = cv2.imread('lena.png')

# px = img[100,100]
# #print px

# blue = img.item(100,100,0)
# #to reassign: img.itemset((100,100,0),100)
# #print blue

# print img.shape #size of the picture
# print img.size #total number of pixels
# print img.dtype #type of image data

# #ROI = Region Of Image
# #thing = img[xscale, yscale]
# #img[newx, newy] = thing

# b,g,r = cv2.split(img) #split is costly!
# #remerge img = cv2.merge((b,g,r))
# img[:,:,2] = 0 #reassign all red to 0

# cv2.imshow('image', img)
# cv2.waitKey(0)

BLUE = [255,0,0]

img1 = cv2.imread('lena.png')

replicate = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_REPLICATE)
reflect = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_REFLECT)
reflect101 = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_REFLECT_101)
wrap = cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_WRAP)
constant= cv2.copyMakeBorder(img1,10,10,10,10,cv2.BORDER_CONSTANT,value=BLUE)

plt.subplot(231),plt.imshow(img1,'gray'),plt.title('ORIGINAL')
plt.subplot(232),plt.imshow(replicate,'gray'),plt.title('REPLICATE')
plt.subplot(233),plt.imshow(reflect,'gray'),plt.title('REFLECT')
plt.subplot(234),plt.imshow(reflect101,'gray'),plt.title('REFLECT_101')
plt.subplot(235),plt.imshow(wrap,'gray'),plt.title('WRAP')
plt.subplot(236),plt.imshow(constant,'gray'),plt.title('CONSTANT')

plt.show()