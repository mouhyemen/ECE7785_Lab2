import cv2
import numpy as np
import math

def nothing(x):
	pass

def main():
	# Create the tracking window
	cv2.namedWindow('Tracker')
	cv2.createTrackbar('track','Tracker', 0,  1, nothing)
	cv2.createTrackbar('minH','Tracker', 90,180, nothing)
	cv2.createTrackbar('minS','Tracker', 70,255, nothing)
	cv2.createTrackbar('minV','Tracker',100,255, nothing)
	cv2.createTrackbar('maxH','Tracker',120,180, nothing)
	cv2.createTrackbar('maxS','Tracker',200,255, nothing)
	cv2.createTrackbar('maxV','Tracker',255,255, nothing)
	cv2.createTrackbar(	'i1' ,'Tracker',  0, 10, nothing)
	cv2.createTrackbar(	'i2' ,'Tracker',  5, 10, nothing)

	# Read from webcam
	camera = cv2.VideoCapture(0)

	X, Y = [], []
	counter = 0
	
	# Loop through each webcam frame
	while(True):
		counter += 1
		ret, frame = camera.read()
		track = cv2.getTrackbarPos('track', 'Tracker')

		# If track is set to 1, then track, else have static values
		if track == 1:
			i1 = cv2.getTrackbarPos('i1', 'Tracker')
			i2 = cv2.getTrackbarPos('i2', 'Tracker')
			minH = cv2.getTrackbarPos('minH', 'Tracker')
			minS = cv2.getTrackbarPos('minS', 'Tracker')
			minV = cv2.getTrackbarPos('minV', 'Tracker')
			maxH = cv2.getTrackbarPos('maxH', 'Tracker')
			maxS = cv2.getTrackbarPos('maxS', 'Tracker')
			maxV = cv2.getTrackbarPos('maxV', 'Tracker')
			colorLower = (minH,minS,minV)
			colorUpper = (maxH,maxS,maxV)
		else:
			# lime-green
			colorLower = ( 25, 50,  0)
			colorUpper = ( 90,255,255)
			i1, i2 = 3, 10

			# # blue
			# colorLower = ( 90, 70,100)
			# colorUpper = (120,200,255)
			# i1, i2 = 0, 5

			# # orange
			# colorLower = (  0, 75,145)
			# colorUpper = ( 50,255,255)
			# i1, i2 = 5, 10

		# Convert to HSV, create mask, perform erosion/dilation
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)
		mask = cv2.erode(mask, None, iterations=i1)
		mask = cv2.dilate(mask, None, iterations=i2)
		
		# Detect contours
		(_,cnts, _) = cv2.findContours(mask.copy(), \
			cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		clone = frame.copy()
		if counter%10 == 0:
			X, Y = [], []

		# For each contour, calc area and perimeter, centroid
		for (i,c) in enumerate(cnts):
			area = cv2.contourArea(c)
			perimeter = cv2.arcLength(c, True)

			if area > 300 and perimeter > 90:
				# print ("Contour #%d -- area: %.2f, perimeter: %.2f" % (i + 1, area, perimeter))
				c = max(cnts, key=cv2.contourArea)
				M = cv2.moments(c)
				(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				X.append(cX)
				Y.append(cY)

		# Average each x and y location
		if X and Y:
			cX = int(sum(X)/len(X))
			cY = int(sum(Y)/len(Y))

			(startX, endX) = (int(cX - 15), int(cX + 15))
			(startY, endY) = (int(cY - 15), int(cY + 15))
			cv2.line(clone, (startX, cY), (endX, cY), (0, 0, 255), 3)
			cv2.line(clone, (cX, startY), (cX, endY), (0, 0, 255), 3)

		# Display the results
		cv2.imshow('clone',clone)
		cv2.imshow('mask',mask)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	camera.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()

