import cv2
import serial
import heapq
import time
arena_images=[]
work_area=[]
door_area=[]
obstacles = []
props={}
RobotPosition=(2,4)
RobotOrientation='l' # can take values 'u' (up), 'd' (down), 'r' (right), 'l' (left)

class PriorityQueue:

	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))

	def get(self):
		return heapq.heappop(self.elements)[1]

def getRobotOrientation():
	pass

def sendPath(path, signal):
	global RobotPosition
	global RobotOrientation
	movements=''
	for (a,b) in path:
		(x,y)=RobotPosition
		if x==a:
			if b==y+1:
				if RobotOrientation=='u':
					movements += 'rf'
				elif RobotOrientation=='d':
					movements += 'lf'
				elif RobotOrientation=='l':
					movements += 'bf'
				elif RobotOrientation=='r':
					movements += 'f'
				RobotOrientation = 'r'
			elif b==y-1:
				if RobotOrientation=='u':
					movements += 'lf'
				elif RobotOrientation=='d':
					movements += 'rf'
				elif RobotOrientation=='l':
					movements += 'f'
				elif RobotOrientation=='r':
					movements += 'bf'
				RobotOrientation = 'l'
		elif y==b:
			if a==x+1:
				if RobotOrientation=='u':
					movements += 'bf'
				elif RobotOrientation=='d':
					movements += 'f'
				elif RobotOrientation=='l':
					movements += 'lf'
				elif RobotOrientation=='r':
					movements += 'rf'
				RobotOrientation = 'd'
			elif a==x-1:
				if RobotOrientation=='u':
					movements += 'f'
				elif RobotOrientation=='d':
					movements += 'bf'
				elif RobotOrientation=='l':
					movements += 'rf'
				elif RobotOrientation=='r':
					movements += 'lf'
				RobotOrientation = 'u'
		RobotPosition = (a,b)
	# movements += '#'

	ser=serial.Serial("COM5",9600)
	print movements
	for i in range(0,len(movements)-1):
		ser.write(movements[i])
		time.sleep(1)
		while True:
			if ser.read() == '!':
				break		
		print movements[i]
	if signal == 'pick':
		ser.write(b'1')
		time.sleep(1)
		print '1'
	elif signal == 'place':
		ser.write(b'0')
		time.sleep(1)
		print '0'
	ser.close()

def getObjects(arena):
	column=[]
	global arena_images
	for i in range(1,arena.shape[0],arena.shape[0]/6):
		for j in range(1,arena.shape[1],arena.shape[1]/9):
			column+=[arena.copy()[i:i+(arena.shape[0]/6-1), j:j+(arena.shape[1]/9)-1]]
		arena_images+=[column]
		column=[]

# for debugging
def dispImages():
	print "Door"
	for i in range(0,6):
		cv2.imshow('door',arena_images[i][0])
		print props[i,0]
		cv2.waiti(0)
	print "Work"
	for i in range(0,6):
		for j in range(1,9):
			cv2.imshow('work',arena_images[i][j])
			if getColor(arena_images[i][j]) != 'white':
				print props[i,j]
			cv2.waitKey(0)

def getColor(image):
	[b,g,r]=image[image.shape[0]/2, image.shape[1]/2]
	if b>127 and g>127 and r>127:
		return 'white'
	elif b>127:
		return 'blue'
	elif g>127:
		return 'green'
	elif r>127:
		return 'red'

def getShape(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(gray,170,255,1)
	contours,h = cv2.findContours(thresh,1,2)
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
		if len(approx)==3:
			return "Triangle"
		elif len(approx)==4:
			return "Square"
		else:
			return "Circle"


def getSize(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(gray,170,255,1)
	contours,h = cv2.findContours(thresh,1,2)
	cnt = contours[0]
	A = cv2.contourArea(cnt)
	P = cv2.arcLength(cnt,True)
	return (A,P)


def getOccupiedObjectsProps():
	global work_area
	global door_area
	global props
	global obstacles
	for i in range(0,6):
		for j in range(0,9):
			image = arena_images[i][j]
			color = getColor(image)
			if color != 'white':
				if j!= 0:
					work_area += [(i,j)]
					obstacles += [(i,j)]
				else:
					door_area += [(i,j)]
				props[(i,j)] = (color,getShape(image),getSize(image))

# , [1, 1], [1, -1], [-1, 1], [-1, -1]
def neighbors(node):
	possibleDirections = [[1, 0], [0, 1], [-1, 0], [0, -1]]
	result = []
	for dir in possibleDirections:
		neighbor = (node[0] + dir[0], node[1] + dir[1])
		if 0 <= neighbor[0] < 6 and 0 <= neighbor[1] < 9:
			result.append(neighbor)
	return result

def cost(next, goal):
	if next == goal:
		return 0
	elif next in obstacles:
		return 100000
	return 1

def heuristic(a, b):
	distance = abs(a[0] - b[0]) + abs(a[1] - b[1])
	return distance

def a_star(start, goal):
	frontier = PriorityQueue()
	frontier.put(start, 0)
	came_from = {}
	cost_so_far = {}
	came_from[start] = None
	cost_so_far[start] = 0
	while not frontier.empty():
		current = frontier.get()
		if current == goal:
			break
		for next in neighbors(current):
			new_cost = cost_so_far[current] + cost(next, goal)
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				priority = new_cost + heuristic(goal, next)
				frontier.put(next, priority)
				came_from[next] = current			
	path = [goal]
	total_cost=cost_so_far[came_from[goal]]
	if total_cost >= 1000:
		return([],0)
	else:
		while came_from[current] != start:
			current = came_from[current]
			path.append(current)
		path.append(start)
		path.reverse()
		return (path, len(path))


cap = cv2.VideoCapture(1)
for i in range(0,30):
	ret, image = cap.read()
image = cap.read()
del(cap)

# for the time being, we'll use the digital arena
arena=cv2.imread("test_images/original.jpg")
# arena=cv2.imread("arena_test.png")


# Do processing to deblur the image

getObjects(arena)
getRobotOrientation()
getOccupiedObjectsProps()

# for i in range(0,6):
	# for j in range(0,9):
		# cv2.imshow('img',arena_images[i][j]);
		# if getColor(arena_images[i][j]) != 'white':
			# print props[(i,j)]
		# cv2.waitKey(0)

# Deleting the entries in the door area which doesnt have a match in the work area
p=0
while p < len(door_area):
	flag=True
	i=door_area[p]
	for j in work_area:
		if props[i] == props[j]:
			flag=False
			break
	if flag:
		door_area.remove(i)
		p=p-1
	p=p+1

for p in range(0,len(door_area)):
	paths = PriorityQueue()
	for i in door_area:
		flag =True
		nmatches=0
		matches=[]
		for j in work_area:
			if props[i] == props[j] and j[1]!= 0:
				nmatches += 1
				matches += [j]
		(pathRM, pathLenRM) = a_star(RobotPosition, matches[0])  # RM - Robot to Match
		(pathMD, pathLenMD) = a_star(matches[0], i)  # MD - Match to Door
		pathLenTotal = pathLenRM + pathLenMD
		match = matches[0]
		if nmatches != 1:
			for k in range(1, nmatches):
				(nextPathRM, nextPathLenRM) = a_star(RobotPosition, matches[k])  # RM - Robot to Match
				(nextPathMD, nextPathLenMD) = a_star(matches[k], i)  # MD - Match to Door

				if pathRM == [] or pathMD == []: # checking if the first computed paths are null
					if nextPathRM == [] or nextPathMD == []: # checking if the next computed paths are null
						flag = False
					else: # the next computed paths are not null, but the first ones are null
						pathLenTotal = nextPathLenRM + nextPathLenMD
						pathRM = nextPathRM
						pathMD = nextPathMD
						match = matches[k]
				else: # the first computed paths are not null
					if nextPathRM == [] or nextPathMD == []: # checking if the next computed paths are null
						pass
					else: # both the computed paths are not null
						if (nextPathLenRM+nextPathLenMD) < (pathLenRM+pathLenMD):
							pathLenTotal = nextPathLenRM + nextPathLenMD
							pathRM = nextPathRM
							pathMD = nextPathMD
							match = matches[k]
		if pathRM == [] or pathMD == []: # checking if the first computed paths are null
			flag = False
		if flag:		 
			paths.put([i, match, pathRM, pathMD], pathLenTotal)
	[start, match, pathRM, pathMD] = paths.get()
	# print "Robot pos= ", RobotPosition
	# print "start= ",start
	# print "match = ",match
	# print "PathRM = ",pathRM
	# print "PathMD = ",pathMD
	sendPath(pathRM[1:], 'pick')
	ser=serial.Serial("COM5",9600)
	while True:
		if ser.read() == '!':
			break;
	ser.close()		
	sendPath(pathMD[1:], 'place')
	ser=serial.Serial("COM5",9600)
	while True:
		if ser.read() == '!':
			break;		
	ser.close()
	door_area.remove(start)
	work_area.remove(match)
	obstacles.remove(match)
	obstacles.append(start)
	RobotPosition = pathMD[-2]
	print "Robot is at ",RobotPosition
# 

print "THE END"

# for debugging
# dispImages()
# print work_area
# print props
