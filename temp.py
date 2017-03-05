import cv2
import time
cap=cv2.VideoCapture(1)
while True:
	time.sleep(10)
	ret,frame=cap.read()
	cv2.imshow('img',frame)
	time.sleep(1)
	cv2.imwrite('img.jpg',frame)
	quit()
	cv2.waitKey(0)

cap.release()
cv2.destroyAllWindows()