# -*- coding: utf-8 -*-
import cv2
import numpy as np

arena_images=[]

def getType(size):
    if(size[0]>1200 and size[0]<=1600):
        return 'medium'
    elif(size[0]<400 and size[0]>10):
        return 'small'
    elif(size[0]>1600):
        return 'large'

def getSize(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(gray,170,255,1)
	contours,h = cv2.findContours(thresh,1,2)
	cnt = contours[0]
	A = cv2.contourArea(cnt)
	P = cv2.arcLength(cnt,True)
	return (A,P)

def getShape(image):
	#return 'pass shape'
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(gray,200,255,1)#170 default,255,1
	contours,h = cv2.findContours(thresh,1,2)
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)

		if len(approx)==3:
			return "Triangle"
		elif len(approx)==4:
			return "4-sided"
		else:
			return "Circle"

def getObjects(arena):
    column=[]
    global arena_images
    for i in range(1,arena.shape[0],arena.shape[0]/6):
        for j in range(1,arena.shape[1],arena.shape[1]/9):
            column+=[arena.copy()[i:i+(arena.shape[0]/6-1), j:j+(arena.shape[1]/9)-1]]
        arena_images+=[column]
        column=[]

def gamma_correction(img, correction):
    img = img/255.0
    img = cv2.pow(img, correction)
    return np.uint8(img*255)

def getColor(image,flag):
        height, width, channel = image.shape
        midheight=height/2
        midwidth=width/2
    	b=image[midheight,midwidth,0]
    	g=image[midheight,midwidth,1]
    	r=image[midheight,midwidth,2]

        image[midheight][midwidth]=[0,0,255]

        cv2.imwrite('dot.png',image)
        cv2.imshow("Dot", image)
        cv2.waitKey(0)

        b=np.int_(b)
        g=np.int_(g)
        r=np.int_(r)

        if(flag==1):
            print b,g,r
        #check for white
        if(abs((b-g))<20):
            if(abs(b-r)<20):
                if(r>30):
                    return 'white'
                else:
                    return 'black'
	   #find greater value color
        if(b>g):
            if(b>r):
                return 'blue'
            else:
                return 'red'
        else:
            if(g>r):
                return 'green'
            else:
                return 'red'


# arena = cv2.imread('original.jpg')
arena = cv2.imread('test_images/original.jpg')
arena = gamma_correction(arena, 0.5)
cv2.imwrite('arena.png',arena)
cv2.imshow("After Gamma Correction", arena)
cv2.waitKey(0)

getObjects(arena)

#check output of colors
for i in range(0,6):
    for j in range(0,9):
        if(i==0 and j==3):
            #print i+1,j+1,getColor(arena_images[i][j],0),getShape(arena_images[i][j],1)
            print i+1,j+1,getColor(arena_images[i][j],0),getShape(arena_images[i][j]),getSize(arena_images[i][j]),getType(getSize(arena_images[i][j]))
        else:
            print i+1,j+1,getColor(arena_images[i][j],0),getShape(arena_images[i][j]),getSize(arena_images[i][j]),getType(getSize(arena_images[i][j]))

cv2.imwrite('arena.png',arena)
cv2.imshow("Final", arena)
cv2.waitKey(0)
