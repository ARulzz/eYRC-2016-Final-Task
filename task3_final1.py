
 # Team Id: eYRC-LM#2855
 # Author List: Aarya R. Shankar, Jilvin Jacob
 # Filename: all_functions.c
 # Theme: Launch A Module
 # Functions: getImages(), getColor(image), getShape(image), getSize(image), getOccupiedObjectsProps(), neighbors(tuple, tuple), cost(tuple, tuple), heuristic(tuple, tuple), a_star(tuple, tuple), gamma_correction(image, int), disp_images(), sendPath(list, string), getRobotOrientation
			# class PriorityQueue: __init__(self), empty(self), put(self, tuple priority), get(self)
 # Global variables: arena_images, work_area, door_area, obstacles, props, RobotPosition, RobotOrientation

import cv2
import serial
import heapq
import time
import "crop.py"
arena_images=[]
work_area=[]
door_area=[]
obstacles = []
props={}
RobotPosition=(2,4)
RobotOrientation='l' # can take values 'u' (up), 'd' (down), 'r' (right), 'l' (left)



# class for implementing Priority Queue for A* algorithm
class PriorityQueue:
'''
	* Function Name:	__init__
	* Input:			None
	* Output:			None
	* Logic:			Creates an instance of this class, and initializes it to an empty list.
	* Example Call:		a = PriorityQueue()
						Here a is an instance of PriorityQueue
	'''
	def __init__(self):
		self.elements = []

	'''
	* Function Name:	empty
	* Input:			None
	* Output:			Returns the length of the list. By default, it will return zero.
	* Logic:			If the number of elements in the list is zero, then it is empty
	* Example Call:		a.empty() where a in an object of PriorityQueue class
	'''
	def empty(self):
		return len(self.elements) == 0

	'''
	* Function Name:	put
	* Input:			item -> The data to be inserted into the priority queue. In this case, it is a tuple containing the indices of a cell.
						priority -> The priority of the data to be inserted. In this case, it is the sum of the total cost of travelling from the 
						starting point to the indices in item and the distance of the goal from the indices in the item.
	* Output:			None
	* Logic:			The item will be inserted to the list in such a way that it remains sorted in the ascending order of the priorities of the data in it
	* Example Call:		frontier.put((2,3),2)
	'''
	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))

	'''
	* Function Name:	get
	* Input:			None
	* Output:			Returns the second element of the first item in the list.
	* Logic:			An item is popped from the priority queue, which is the one with the least priority.
						The second element of the first item in the list is the tuple or location, while the first one is its priority.
	* Example Call:		frontier.get()
	'''
	def get(self):
		return heapq.heappop(self.elements)[1]

def getRobotOrientation():
	pass

# for sending the correct instructions for movements, pick and place to the robot via the zigbee
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

# * Function Name:	getImages
# * Input:			None
# * Output:			None
# * Logic:			Splits the given input image, and stores each cell in a 10x10 matrix called board_images
# * Example Call:		getImages()

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


# * Function Name:	getColor
# * Input:			image -> variable which stores one of the 60x60 images, present in the cells of the input board
# * Output:			returns the color of the pixel at (30,30)
# * Logic:			No matter what the image is, its mid-point (ie, pixel at (30,30)) will have the color of its main image.
# 					If the cell is blank, the mid point will also be white.
# 					If the cell is black (obstacle), its mid point will also be black.
# 					If the cell contains a triangle, a circle, or a 4-sided figure of any colour among red, blue, and green, its mid point will be of that colour.
# 					So by considering the BGR values of that point, we can get the color.
# * Example Call:		getColor(image)


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

'''
* Function Name:	getShape
* Input:			image -> variable which stores one of the 60x60 images, present in the cells of the input board
* Output:			returns the shape of the object (as "Circle", "4-sided", "Triangle") in the iamge
* Logic:			First, we convert the image into a grayscale image. Then we put a threshold values of 170, and 255, since that will be appropriate for the given green, red, and blue colours.
					Then we find the contours. The number of contour points will be equal to the vertices of the shape.
					If it is 3, it is a triangle.
					If it is 4, it is a 4-sided.
					If it is not either one of these, it is a circle (as the given figues can be only circle, 4-sided, or triangle).
* Example Call:		getShape(image)
'''

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

'''
* Function Name:	getSize(image)
* Input:			image -> variable which stores one of the 60x60 images, present in the cells of the input board
* Output:			(area, perimeter) -> returns a tuple containing the area, and perimeter of the figure in the image
* Logic:			The size of a particular shaped object can be characterized by its area and perimeter.
					Therefore, the area, and the perimeter of the contour using the corresponding functions.
* Example Call:		getSize(image)
'''

def getSize(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(gray,170,255,1)
	contours,h = cv2.findContours(thresh,1,2)
	cnt = contours[0]
	A = cv2.contourArea(cnt)
	P = cv2.arcLength(cnt,True)
	return (A,P)

'''
* Function Name:	getOccupiedObjectsProps
* Input:			None
* Output:			None
* Logic:			We have already split and the whole image, and stored it in an array.
					If the color of the image is white, that co-ordinate is not occupied.
					Else, it is. The co-ordinates of the occupied cells are stored in the list occupied_grids.
					If the colour of the image is not black either, then it is not an obstacle, meaning it has an object.
					If it has an object, we store its indices in a list called objects and 
					we store its proerties which defines it (ie, color, shape and size) in a dictionary props with the indices as the key.
* Example Call:		getOccupiedObjectsProps()
'''
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

def gamma_correction(img, correction):
    img = img/255.0
    img = cv2.pow(img, correction)
    return np.uint8(img*255)


'''
* Function Name:	neighbors
* Input:			node -> A tuple containing the indices of the current node under consideration in A* algorithm
* Output:			Returns a list containg tuples, which are the indices of the neighbors of the node in the input grid.
* Logic:			The neighbors of the node (x, y) can be (x+1, y), (x, y+1), (x-1, y), and (x, y-1).
					The one within the boundaries of the input grid image are the valid neighbors.
* Example Call:		neighbors((2,3)
'''

# for adding diagonal paths: , [1, 1], [1, -1], [-1, 1], [-1, -1]   [obstacles finding for diagonal not set]
def neighbors(node):
	possibleDirections = [[1, 0], [0, 1], [-1, 0], [0, -1]]
	result = []
	for dir in possibleDirections:
		neighbor = (node[0] + dir[0], node[1] + dir[1])
		if 0 <= neighbor[0] < 6 and 0 <= neighbor[1] < 9:
			result.append(neighbor)
	return result


'''
* Function Name:	cost
* Input:			next -> tuple containing the indices of a location that our path may move to.
					goal -> tuple conatining the indices of the current end point.
* Output:			Returns the cost of the movement to indices stored in next.
* Logic:			Since for a particular starting point, all the occupied grids excluding its goal, are obstacles, the movement to any obstacle is given a
					hige value (say, 10000) so that while considering the cost, it won't be the least if there is another longer but obstacle free path,
					and the rest are given the cost of 1. The movement to the goal is given as 0 (not necessary).
* Example Call:		cost((2,3),(4,5))
'''
# try to find obstacles in the adjacent cells of diagonal movement
#	elif (abs(current[0] - next[0]) + abs(current[1] - next[1])) == 2:
		# enter code
		# return 100000
def cost(next, goal):
	if next == goal:
		return 0
	elif next in obstacles:
		return 100000
	return 1

'''
* Function Name:	heuristic
* Input:			a -> a tuple containing a pair of indices
					b -> a tuple containing the indices of the location of our goal
* Output:			Returns the estimated distance from a to b.
* Logic:			We need to calculate the distance so that we can prioritise the location the path should go to.
					It is calculated by the taking the difference of the corresponding x-coordinates and corresponding y-coordinates of a and b
* Example Call:		heuristic((1,2),(4,5))
'''
# implementing heuristic function and A*

def heuristic(a, b):
	distance = abs(a[0] - b[0]) + abs(a[1] - b[1])
	return distance


'''
* Function Name:	a_star
* Input:			start -> tuple that contains the indices of the starting point
					goal -> tuple that contains the indices of the end point
* Output:			returns a tuple containing the path as a list having the coordinates of the locations in the path,
					and the number of movements that happens while following the path.
* Logic:			We need to find the shortest path between start and goal, which excludes all obstacles and other objects. For this, we use A* algorithm.
					We create an instance of the class PriorityQueue, named frontier. We start from start, so first we put start with the priority 0 into frontier.
					We declare a dictionary came_from with a location as its key, and the location where it came from as the value, so that we can retrace the path later on.
					We declare a dictioanry cost_so_far, with a location as its key, and the total cost to come to that location from start as the value, so that we can choose the cheapest path while traversing and get the total cost at the end.
					We will save start as the key and none as the value in the dictionary came_from, as start did come from nowhere.
					We will save start as the key and 0 as the value in the dictionary cost_so_far, as cost of getting to start from start is zero.
					While our priority queue, ie frontier is not empty, we will do the following:
						1. Pop an element with the least priority from frontier, and store it in current.
						2. Compare it to the goal, if it we got our path, so we'll break from the loop.
						3. We will get the neighbors of the location current, and we will iterate throught them, saving the current neighbor under assessment as next..
						4. During each iteration:
							i.   We calculate the new_cost, ie the cost of getting to the neighbor next from start, which is the sum of cost_so_far of theo current location, and the cost of next by using the funtion next(next ,goal).
							ii.  if next is not in the dictionary cost_so_far (ie if next has not been visited yet) or if the new_cost is less than cost_so_far of next (ie if the new cost we calculated is less than the cost of getting to next from start), we do:
							     	a. assign the cost_so_far of next as new_cost
							     	b. calculate the priority of next as the sum of new_cost and the distance between goal and enxt using the function heuristic(goal, next)
							     	c. we add next to the frontier with the calculated priority
							     	d. we also add that the next came from current to the came_from dictionary.
					The usage of priority queue ensures that we always choose the path with the least cost and distance.
					Now we will calculate the total cost of getting to the goal from start. Since the cost is manipulated in such a way that, if it goes through atleast one obstacle,
					the cost will be 10000, we know that if the total cost of the path exceeds 10000, there is no obstacle free path to our goal.
					To calculate the toal cost, we take the location from where the goal came in our path. Then take the cost so far of that location.
					If it exceeds 10000, we will return a tuple an empty list and an integer 0, signifying that there is no path available.
					Else, we trace back our path, using the came_from dictionary, and appending the right location to the list path.
					We add one to each element of the location indice, because our co-ordinates are 1 greater than out indices.
					Now we have our path from the goal to the starting point. We need to reverse it to get the path from starting point to the goal.
					Once we do that, we return a tuple conating the list path, and the total number of movements.
					The total number of movements is always one grater than the total elements of our path. So we return length of path + 1 as the total number of movements.					
* Example Call:		a_star((2,3),(7,8))
'''

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
arena = cap.read()
del(cap)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image file")
ap.add_argument("-c", "--coords", help = "comma seperated list of source points")
args = vars(ap.parse_args())
 
# load the image and grab the source coordinates (i.e. the list of
# of (x, y) points)
# NOTE: using the 'eval' function is bad form, but for this example
# let's just roll with it -- in future posts I'll show you how to
# automatically determine the coordinates without pre-supplying them
pts = np.array(eval("[(34, 21), (627, 19), (624, 405), (44, 415)]"), dtype = "float32")
 
# apply the four point tranform to obtain a "birds eye view" of
# the image
warped = four_point_transform(arena, pts)
# show the original and warped images
# cv2.imshow("Original", image)
# cv2.imshow("Warped", warped)
# cv2.imwrite("arena_test.png", warped)
# cv2.waitKey(0)

arena = gamma_correction(arena, 0.5)
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
			for k in range(1, nmatches): # iterating through rest of the matches
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
ser=serial.Serial("COM5",9600)
ser.write(b'.')
ser.close()
print "THE END"