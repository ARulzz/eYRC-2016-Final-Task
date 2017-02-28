import cv2
a=cv2.imread('test.jpg')
k=0
for i in range(0, a.shape[0]-10, a.shape[0]/6):
	for j in range(0, a.shape[1]-10, a.shape[1]/9):
		image=a.copy()[i:i+(a.shape[0]/6), j:j+(a.shape[1]/9)]
		midPt=image[image.shape[0]/2, image.shape[1]/2]
		b=midPt[0]
		g=midPt[1]
		r=midPt[2]
		color=""
		if b>127 and g>127 and r>127:
			color="nothing"
		elif b>127:
			color="blue"
		elif g>127:
			color="green"
		elif r>127:
			color="red"
		else:
			color="Can't say"
		print (k+1)," (",i,", ",j,") ",color
		cv2.imshow('image',image)
		cv2.waitKey(0)
		# gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		# ret,thresh = cv2.threshold(gray,170,255,1)
		# contours,h = cv2.findContours(thresh,1,2)
		# for cnt in contours:
		# 	approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
		# 	if len(approx)==3:
		# 		return "Triangle"
		# 	elif len(approx)==4:
		# 		return "4-sided"
		# 	else:
		# 		return "Circle"
		k=k+1
