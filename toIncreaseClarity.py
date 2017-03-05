import cv2
a=cv2.imread('test_image1.png')
b=cv2.imread('test_image2.png')
c=cv2.imread('test_image3.png')
d=a.copy()
for i in range(0,a.shape[0]):
	for j in range(0,a.shape[1]):
		for k in range(0,a.shape[2]):
			a[i][j][k] = (a[i][j][k] + b[i][j][k]) / 2

cv2.imshow('a',a)
cv2.waitKey(0)
cv2.imshow('b',b)
cv2.waitKey(0)
cv2.imshow('c',c)
cv2.waitKey(0)
cv2.imshow('d',d)
cv2.waitKey(0)
cv2.destroyAllWindows()

print a[1][1]
print b[1][1]
# print c[1][1]
print d[1][1]