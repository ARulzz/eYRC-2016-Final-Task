import cv2
import heapq
arena_images=[]
work_area=[]
door_area=[]
props={}
RobotPosition=(2,4)

class PriorityQueue:

	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))

	def get(self):
		return heapq.heappop(self.elements)[1]

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
	for i in range(0,6):
		for j in range(0,9):
			image = arena_images[i][j]
			color = getColor(image)
			if color != 'white':
				if j!= 0:
					work_area += [(i,j)]
				else:
					door_area += [(i,j)]
				props[(i,j)] = (color,getShape(image),getSize(image))

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
	elif next in work_area:
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



arena=cv2.imread("test_images/sample_arena.jpg")
getObjects(arena)
getOccupiedObjectsProps()
print door_area
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
print door_area
print work_area

for p in range(0,len(door_area)):
	paths = PriorityQueue()
	for i in door_area:
		nmatches=0
		matches=[]
		for j in work_area:
			if props[i] == props[j] and j[1]!= 0:
				nmatches += 1
				matches += [j]
		# print "p= ",p
		# print "i= ",i
		# print matches
		(pathRM, pathLenRM) = a_star(RobotPosition, matches[0])  # RM - Robot to Match
		(pathMD, pathLenMD) = a_star(matches[0], i)  # MD - Match to Door
		pathLenTotal = pathLenRM + pathLenMD
		match = matches[0]
		if nmatches != 1:
			for k in range(1, nmatches):
				(nextPathRM, nextPathLenRM) = a_star(RobotPosition, matches[k])  # RM - Robot to Match
				(nextPathMD, nextPathLenMD) = a_star(matches[k], i)  # MD - Match to Door
				if (nextPathLenRM+nextPathLenMD) < (pathLenRM+pathLenMD):
					pathLenTotal = nextPathLenRM + nextPathLenMD
					pathRM = nextPathRM
					pathMd = nextPathMD
					match = matches[k]
		paths.put([i, match, pathRM, pathMD], pathLenTotal)
	print paths.elements
	[start, match, pathRM, pathMD] = paths.get()
	print "start= ",start
	print "match = ",match
	print "PathRM = ",pathRM
	print "PathMD = ",pathMD
	door_area.remove(start)
	work_area.remove(match)
	work_area.append(start)
	RobotPosition = pathMD[-1]
	print "Robot is at ",RobotPosition

print "THE END"

# for debugging
# dispImages()
# print work_area
# print props
