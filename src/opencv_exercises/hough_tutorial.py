import cv2
import numpy as np

img = cv2.imread('glass.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 150, apertureSize = 3)

minLineLength = 10
maxLineGap = 10

circles = cv2.HoughCircles(edges, cv2.cv.CV_HOUGH_GRADIENT, 1, img.shape[0]/8, param1=900, param2=150, minRadius=20, maxRadius=700)
# for rho, theta in lines[0]:
# 	a = np.cos(theta)
# 	b = np.sin(theta)
# 	x0 = a*rho
# 	y0 = b*rho
# 	x1 = int(x0 + 1000*-b)
# 	y1 = int(y0 + 1000*a)
# 	x2 = int(x0 - 1000*-b)
# 	y2 = int(y0 - 1000*a)

# for x1,y1,x2,y2 in lines[0]:
# 	cv2.line(img, (x1, y1), (x2, y2), (0,0,255), 2)

if circles is not None:
	print 'Circles detected'
	for c in circles[0,:]:
	    # draw the outer circle
	    cv2.circle(img,(c[0],c[1]),c[2],(0,255,0),2)
	    # draw the center of the circle
	    cv2.circle(img,(c[0],c[1]),2,(0,0,255),3)

cv2.imshow('houghlines.jpg', img)
k = cv2.waitKey(0) & 0xFF
if k == 27:
	cv2.destroyAllWindows()
