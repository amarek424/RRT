import sys

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology 
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch
    

'''
Render the problem  
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)    
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)    
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)    
    plt.show()

	
'''
Find distance between 2 points
'''
def calculateDistance(point1, point2):
		x1 = point1[0]
		y1 = point1[1]
		x2 = point2[0]
		y2 = point2[1]
		
		distance = np.sqrt(((x1 - x2)**2) + ((y1 - y2)**2))
		return distance
	
'''
Find nearest x and return distance
'''	
def nearestPoint(newPoints, x_new):
	x_near = -1
	distance = 15 # No distance will be greater than this
	for i, coordinates in newPoints.items():
		if newPoints[i] != newPoints[x_new]:
			distance_new = calculateDistance(newPoints[x_new], newPoints[i])
			if distance_new < distance:
				distance = distance_new
				x_near = i
				
	return x_near, distance
	
'''
Grow a simple RRT 
'''
def growSimpleRRT(points):
	newPoints = dict()
	adjListMap = dict()
    
    # Your code goes here
	for i, coordinates in points.items():
		# add to newPoints
		newPoints[i] = points[i]
		adjListMap[i] = []
		# if this is not just initialization, pick next sample and find nearest
		if i != 1:
			x_new = i
			x_near, distance = nearestPoint(newPoints, x_new)
			if x_near != -1:	
				# update adjListMap
				adjListMap[x_near].append([x_new, distance])
				adjListMap[x_new].append([x_near, distance])
			else:
				newPoints.remove(i)
				adjListMap.remove(i)
				print "Error"
	
	return newPoints, adjListMap

'''
Perform basic search 
'''
def basicSearch(tree, start, goal):
	path = []
    
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.
	
	# use this to keep track of visited nodes
	visited = dict()
	for key, pairs in tree.items():
		visited[key] = False
	
	queue = [start]
	backpointers = dict()
	# Do a depth first search, append new points to the start of the queue
	while len(queue) != 0:
		key = queue.pop(0)
		visited[key] = True
		# populate queue with neighbors
		if key != goal:
			# append neighbors to the front of queue
			for adjNode in tree[key]:
				if not visited[adjNode[0]]:
					queue = [adjNode[0]] + queue
					# set backpointers
					backpointers[adjNode[0]] = key	
		# found the goal
		else:
			break
			
	# Retrieve path
	path.append(goal)
	while path[0] != start:
		path = [backpointers[path[0]]] + path
		
	if path[len(path) - 1] != goal:
		print "No path found!"
    	
	return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):
    
    # Your code goes here
    # You could start by copying code from the function
    # drawProblem and modify it to do what you need.
    # You should draw the problem when applicable.
		
	# graphing for points, tree, path here
	fig, ax = setupPlot()   
	
	# tree
	verts = []
	codes = []
	for p, adjNodes in tree.items():
		for pair in adjNodes:
			# Pick up the pen to the root and draw to each adj
			verts.append((points[p][0]/10., points[p][1]/10.))
			codes.append(Path.MOVETO)
			verts.append((points[pair[0]][0]/10., points[pair[0]][1]/10.))
			codes.append(Path.LINETO)
	rrt = Path(verts, codes)
	patch = patches.PathPatch(rrt, facecolor='none', lw=1)
	ax.add_patch(patch)
	
	# path
	if len(path) != 0:
		x = []
		y = []
		for p in range(0, len(path)):
			key = path[p]
			x.append(points[key][0]/10.)
			y.append(points[key][1]/10.)
		ax.plot(x, y, color="orange")
	
	# if robotStart
	if robotStart != None:
		patch = createPolygonPatch(robotStart, 'green')
		ax.add_patch(patch)   
	
	# if robotGoal
	if robotGoal != None:
		patch = createPolygonPatch(robotGoal, 'red')
		ax.add_patch(patch) 
	
	# if polygons
	if polygons != None:
		for p in range(0, len(polygons)):
			patch = createPolygonPatch(polygons[p], 'gray')
			ax.add_patch(patch) 
	
	# plot
	plt.show()
	
	return 

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.
	
	# Place the robot coordinates at the point
	tempRobot = []
	for i in range(0, len(robot)):
		lst = list(robot[i])
		lst[0] = lst[0] + point[0]
		lst[1] = lst[1] + point[1]
		tempRobot.append(tuple(lst))
	
	# See the robot is inside an obstacle or vice versa
	for polygon in obstacles:
		polyBorders = Path(polygon)
		for point in tempRobot:
			if polyBorders.contains_point(point):
				return False
	# Vice versa
	roboBorders = Path(tempRobot)
	for polygon in obstacles:
		for point in polygon:
			if roboBorders.contains_point(point):
				return False
				
	# Check for intersects
	for polygon in obstacles:
		for i in range(0, len(polygon)):
			curr1 = tuple(polygon[i])
			next1 = (0, 0)
			if i == (len(polygon) - 1):
				next1 = tuple(polygon[0])
			else:
				next1 = tuple(polygon[i+1])
			# robot lines
			for k in range(0, len(tempRobot)):
				curr2 = tempRobot[k]
				next2 = (0,0)
				if k == (len(tempRobot) - 1):
					next2 = tempRobot[0]
				else:
					next2 = tempRobot[k+1]
				# Check intersection
				line1 = Path([curr1, next1])
				line2 = Path([curr2, next2])
				if line1.intersects_path(line2, filled=False):
					return False
					
	# Make sure it's in the graph area
	boundaries = Path([[0.0, 0.0], [0.0, 10.0], [10.0, 10.0], [10.0, 0.0]])
	catch = True
	for point in tempRobot:
		if not boundaries.contains_point(point):
			catch = False
			
	if catch:
		return True
		
	return False
'''
Checks if the robot would collide along the path
'''
def isCollisionFreePath(robot, point1, point2, obstacles):
	
	# Place the robot coordinates into easy access array
	tempRobot = []
	for i in range(0, len(robot)):
		lst = list(robot[i])
		tempRobot.append(lst)
		
	# Check for collision on lines between all points of the robot
	for xy in tempRobot:
		temp1 = [point1[0] + xy[0], point1[1] + xy[1]]
		temp2 = [point2[0] + xy[0], point2[1] + xy[1]]
		if not isCollisionFree([temp1, temp2], [0.0, 0.0], obstacles):
			return False
		
	return True
	
'''
Find nearest x and return distance while accounting for obstacles
'''	
def nearestPointWithoutCollision(robot, points, x_new, obstacles):
	x_near = -1
	distance = 15 # No distance will be greater than this
	if x_new in points:
		return x_near, distance
		
	for i, coordinates in points.items():
		distance_new = calculateDistance(x_new, points[i])
		if distance_new < distance and isCollisionFreePath(robot, x_new, points[i], obstacles):
			distance = distance_new
			x_near = i
				
	return x_near, distance
	
'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

	points = dict()
	tree = dict()
	path = []
    # Your code goes here.
	
	# Check if the start an end points are valid
	if not isCollisionFree(robot, goalPoint, obstacles):
		print "Invalid input, the end point is invalid."
		exit(1)
	if not isCollisionFree(robot, startPoint, obstacles):
		print "Invalid input, the start point is invalid."
		exit(1)
	points[1] = startPoint
	tree[1] = []
	
	# Trivial solution
	if startPoint == goalPoint:
		print "Trivial solution."
		tree[1].append([2, 0.0])
		points[2] = goalPoint
		tree[2] = []
		tree[2].append([1, 0.0])
	# Direct solution
	elif isCollisionFreePath(robot, startPoint, goalPoint, obstacles):
		print "Direct solution."
		distance = calculateDistance(startPoint, goalPoint)
		tree[1].append([2, distance])
		points[2] = goalPoint
		tree[2] = []
		tree[2].append([1, distance])
	# Generate RRT
	else:
		print "Generating RRT."
		counter = 3
		while True:
			# Generate random point
			x = np.random.randint(0,100) / 10.0
			y = np.random.randint(0,100) / 10.0
			x_new = (x, y)
			# Continue if collision free, otherwise generate new point
			if isCollisionFree(robot, x_new, obstacles):
				# Find nearestPoint
				x_near, distance = nearestPointWithoutCollision(robot, points, x_new, obstacles)
				# Found a valid nearest point
				if x_near != -1:
					points[counter] = x_new
					tree[counter] = []
					tree[counter].append([x_near, distance])
					tree[x_near].append([counter, distance])			
					# If this can connect to goal, we're done
					if isCollisionFreePath(robot, x_new, goalPoint, obstacles):
						points[2] = goalPoint
						tree[2] = []
						tree[2].append([counter, distance])
						tree[counter].append([2, distance])
						break
					counter = counter + 1	
		
	# Find path
	path = basicSearch(tree, 1, 2)
	
	robotStart = []
	robotGoal = []
	for i in range(0, len(robot)):
		lst = list(robot[i])
		lst[0] = lst[0] + startPoint[0]
		lst[1] = lst[1] + startPoint[1]
		robotStart.append(tuple(lst))
	for i in range(0, len(robot)):
		lst = list(robot[i])
		lst[0] = lst[0] + goalPoint[0]
		lst[1] = lst[1] + goalPoint[1]
		robotGoal.append(tuple(lst))
	displayRRTandPath(points, tree, path, robotStart, robotGoal, obstacles)
    
	return points, tree, path

if __name__ == "__main__":
    
    # Retrieve file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print "" 
    print "The input points are:"
    print str(points)
    print ""
    
    points, adjListMap = growSimpleRRT(points)

    # Search for a solution  
    path = basicSearch(adjListMap, 1, 20)    

    # Your visualization code 
    displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 



