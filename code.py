import sys
from typing import NewType
from api import *
from time import sleep
import numpy as np
import random as r
import math

'''
The problem statement given is ( in brief ):
		There are 6 levels that are needed to be solved. The backdrop is a 200*200 image
		consisting of blue bots, and green nodes. An efficient code is required to move 
		the bot to all the nodes in an effective way.
		Level 1 : 1 bot 1 node
		Level 2 : 1 bot multiple nodes
		Level 3 : 2 bots multiple nodes
		Level 4 : multiple bots multiple nodes
		Level 5 : 2 bots with multiple nodes and red obstacles
		Level 6 : multiple bots with multiple nodes and red obstacles
	
	Solution : 
		Level 1 is acheived using A* algorithm which creates an efficient path from bot pos to destination
		Level 2 is acheived by moving the bot to the immediate shortest node and repeating it for every other node
		Level 4 is a decentralized approach for the bots to move to the nodes. Each bot knows the position of 
				the nodes and the position of the other bots, but dont know where the other bots are headed. Each bot 
				creates a path to the immediate shortest node and splits the path to n numbers and sets them as checkpoints.
				At every checkpoint including the starting point, a check is made to see if destination is already visited 
				by another bot. Another property is associated with each bot that at every checkpoint, a bot checks its surrounding
				(a square range around the bot **resembling the action of sensors**), if another bot is nearer to it. If a bot is detected in its square, the bot
				checks if it is nearer to the destination or the other detected bot is closer to the node (irrespective of the path of 
				the other bot). If the other bot is closer, the bot drops the node from its list and checks for the next better node, else 
				it goes on to proceed to the node and a check is executed at the next checkpoint.A bot stops if all the nodes are removed from its list.
		Level 3 is the same as level 4 and level 5, 6 is also the same but also includes red obstacles

	some Points about code :
		To move a bot from pos to destination level1 is used.
		The bot touches the nearest point of the green node. and uses the center of the node for estimated distance
		min1 is a global variable that stores the destination of the bot , The path , and the index of dest in the original _greenzone_list

'''
#######    YOUR CODE FROM HERE #######################
import random


grid=[]
neigh=[[-1,-1],[-1,0],[-1,1],[0,1],[1,1],[1,0],[1,-1],[0,-1]]

green = get_greenZone_list()

img = get_Map()

class Node:
	def __init__(self,value=0,point=(0,0)):
		self.value = value  #0 for blocked,1 for unblocked
		self.point = point
		self.parent = None
		self.move=None
		self.H = 0
		self.G = 0

def get_path(start, dest2):                         # This function returns a path from given start to dest using A* algo.
	global grid, openlist, closedlist

	openlist =[]           # list containing open nodes
	closedlist =[]         # List containing Visited nodes

	grid =[]


	for i in range(200):      # Creating a matrix whose each element is a class 
		grid.append([])
		for j in range(200):
			grid[i].append(None)
			value=1
			if (img[i][j][0]==0 and img[i][j][1]==0 and img[i][j][2]==0): #value is set to 0 if it is an obstacle else 1
				value=0
			grid[i][j] = Node(value,(i,j)) 
			grid[i][j].H = geth((i,j),dest2)   # It sets the heuristic value of the node
	
	grid[start[0]][start[1]].G = 0
	openlist.append(grid[start[0]][start[1]])

	current = grid[start[0]][start[1]]

	while (len(openlist)>0):  #while loop to execute A* algo that finds the shortest path to dest

		links = neighbours(current)  # gets the available neighbours and are appended to openlist
		openlist.remove(current)
		for i in range(len(links)):
			openlist.append(links[i])
	
		closedlist.append(current)
		current = find_min(openlist) # finds min of open list and is set as current
		
		if ((current.point[0]==dest2[0]) and (current.point[1]==dest2[1])):
			
			lst1=[current.point]
			while ((current.parent[0] != start[0]) or (current.parent[1] != start[1])):  # creates a list of nodes theat are obtained by getting parent of each node
				lst1.append(current.parent)
				current = grid[current.parent[0]][current.parent[1]]
			lst1.reverse()  # the list is reversed to get the path from start to dest.
			return lst1
	
def isValid(pt):
	return pt[0]>=0 and pt[1]>=0 and pt[0]<200 and pt[1]<200

def geth(point,dest):        # This function returns the eulers heuristic function
	x,y = point[0],point[1]
	a,b = dest[0],dest[1]
	h = math.sqrt(((x-a)**2)+((y-b)**2))

	return h

def find_min(oplist):      # returns the element containing the least H+G value
	global grid, openlist
	current = Node(0,(0,0))
	current.H =10000
	for i in range(len(openlist)):
		if((openlist[i].H + openlist[i].G) < (current.H + current.G)):
			current = oplist[i]
	return current


def neighbours(point):  #returns valid neighbours
	global grid, neigh, closedlist
	x,y = point.point
	links=[]
	for i in range(len(neigh)):
		newX=x+neigh[i][0]
		newY=y+neigh[i][1]
		if not isValid((newX,newY)):   # Checks for validity
			continue
		if grid[newX][newY].value==0:   # Checks for obstacle
			continue 
		if closedlist.count(grid[newX][newY])!=0: # checks existence of node in closedlist
			continue 
		if openlist.count(grid[newX][newY])!=0:  #  Checks existence of node in openlist
			continue 

		if (img[newX][newY][0]==0 and img[newY][newY][1]==0 and img[newX][newY][2]==255):  # sets the G value as 2.5 points more than the current node
			grid[newX][newY].G = point.G+2.5
		elif (i==0 or i==2 or i==4 or i==6):    # sets the G value as 1.4 points more than the current node if it is a diagonal node
			grid[newX][newY].G = point.G+1.4
		else:                                 # sets the G value as 1 point more than the current node if it is a non-diagonal node
			grid[newX][newY].G = point.G+1
		grid[newX][newY].parent = (x,y)
		links.append(grid[newX][newY])

	return links

def get_coord(start, end):              # Returns the point in the boundary of the green node, given the center of the green node.
	cord = [[0,0],1000]
	for i in range(green[end[1]][0][0],green[end[1]][2][0]+1):
		for j in range(green[end[1]][0][1],green[end[1]][1][1]+1):
			k = math.sqrt(((i-start[0])**2)+((j-start[1])**2))
			if ( k < cord[1]):
				cord = [[i,j],k]
	return cord[0]

# print(dest)
openlist =[]
closedlist =[]
min1=[[],[],[]]



########## Default Level 1 ##########
def level1(botId):
	global min1
	start = get_botPose_list()[0]
	if (len(min1[1])!=0):           # if it is not level 1, then min1[] contains the path and has non zero length
		lst1 = min1[1]
	else:                           # else it is level 1 and the center destination is found
		dest = get_greenZone_list()

		sumx, sumy = 0, 0
		for i in range(4):
			sumx += dest[0][i][0]
			sumy += dest[0][i][1]
		lst1 = get_path(start, [int(sumx/4), int(sumy/4)])

	mission_complete=False
	pos=get_botPose_list()

	for i in range(len(lst1)):   # executes the motion of the bot using points from lst1
		successful_move, mission_complete = send_command(botId,neigh.index([lst1[i][0]-pos[botId][0],lst1[i][1]-pos[botId][1]])+1)
		pos=get_botPose_list()
		if successful_move:
			pass
			print("YES")
		else:
			print("NO")
		if mission_complete:
			print("MISSION COMPLETE")

	for i in range(green[min1[2]][0][0],green[min1[2]][2][0]+1):        # removes the green color from the image if the node is reached.
		for j in range(green[min1[2]][0][1],green[min1[2]][1][1]+1):
			img[i][j]=[255,255,255]


def level2(botId):
	global min1

	dest1 = get_greenZone_list()

	lst=[]
	for b in range(len(dest1)):               # creates a list containing center of all the green zones
		sumx, sumy = 0, 0
		for j in range(4):
			sumx += dest1[b][j][0]
			sumy += dest1[b][j][1]
		lst.append([[int(sumx/4), int(sumy/4)],b])

	length = len(lst)
	for i in range(length):                  # Traverses a list to find element with shortest distance from bot
		min1 = [[0,0],0,0]
		start = get_botPose_list()[0]
		min1= [[0,0],[],0]

		m=10000
		for j in range(len(lst)):            # Creates path to all available greenzones
			k = get_path(start, get_coord(start, lst[j]))
			if ( len(k) < m):                #Checks for least cost path
				m = len(k)
				min1 = [lst[j][0],k,lst[j][1]]   # Sets the node with least cost as min1     (min1 is a global variable)
		lst.remove([min1[0],min1[2]])
		min1 = [get_coord(start, [min1[0],min1[2]]),min1[1],min1[2]]
		level1(botId)                            # Executes the movement of bot now that min1 is set


#      Check the comments for Level 4 as It is the same for level 3,5 and 6.

def level3(botId):
	global min1

	lst_org = get_original_greenZone_list()
	var = []
	check = True


	while check:
	
		lst2 = get_greenZone_list()

		lst=[]
		for i in range(len(lst2)):
			g=0
			sumx, sumy = 0, 0
			for n in range(len(var)):
				if (lst2[i] == (lst_org[var[n]])):
					g=1
					break
			if g==1:
				continue
			for j in range(4):
				sumx += lst2[i][j][0]
				sumy += lst2[i][j][1]
			lst.append([[int(sumx/4), int(sumy/4)],lst_org.index(lst2[i])])

		if (len(lst))==0:
			check = False
			continue
		
		start = get_botPose_list()[botId]
		min1= [[0,0],1000,0]
	
		for j in range(len(lst)):
		
			k = math.sqrt(((start[0]-lst[j][0][0])**2)+((start[1]-lst[j][0][1])**2))
			if ( k < min1[1]):
				min1 = [lst[j][0],k,lst[j][1]]

		lst.remove([min1[0],min1[2]])
		path1 = get_path(start, get_coord(start, [min1[0],min1[2]]))
		min1 = [get_coord(start, [min1[0],min1[2]]),min1[1],min1[2]]
		e = 0
		a = 0
		d = 0
		f = 8
		if min1[1]<=30:
			f=1
		
		for x in range(f):
			lst3 = get_greenZone_list()

			for z in range(0,len(lst3)):
				if ( (min1[0][0] >= lst3[z][0][0]) and (min1[0][0] <= lst3[z][2][0]) and (min1[0][1] >= lst3[z][0][1]) and (min1[0][1] <= lst3[z][2][1])):
					var.append(min1[2])
					e = 1
					break
			if e == 0:
				break
			pos = get_botPose_list()
			beg = pos[botId]
			for l in range(2):
				if l==botId:
					continue
				else:
					if ((pos[l][0]>(beg[0]-50)) and (pos[l][0]<(beg[0]+50)) and (pos[l][1]>(beg[1]-50)) and (pos[l][1]<(beg[1]+50))):
						if (math.sqrt(((min1[0][0]-beg[0])**2)+((min1[0][1]-beg[1])**2)) > math.sqrt(((pos[l][0]-min1[0][0])**2)+((pos[l][1]-min1[0][1])**2))):
							var.append(min1[2])
							a = 1
							break
			if a==1:
				print(a)
				break

			min1[1] = path1[d:(int(len(path1)*((x+1)/f)))]
			d = (int(len(path1)*((x+1)/f)))
			level1(botId)


def level4(botId):
	global min1

	lst_org = get_original_greenZone_list()
	var = []
	check = True

	while check:                   # This while loop is used for Each destination in its list

		lst2 = get_greenZone_list()
	
		lst=[]
		for i in range(len(lst2)):  # This for loop creates a list of available greenzones with their centers
			g=0
			sumx, sumy = 0, 0
			for n in range(len(var)):
				if (lst2[i] == (lst_org[var[n]])):
					g=1
					break
			if g==1:
				continue
			for j in range(4):
				sumx += lst2[i][j][0]
				sumy += lst2[i][j][1]
			lst.append([[int(sumx/4), int(sumy/4)],lst_org.index(lst2[i])])


	
		if (len(lst))==0:             # checks if the available nodes are there or not, if not breaks the loop and ends function
			check = False
			continue
		
		start = get_botPose_list()[botId]
		min1= [[0,0],1000,0]

		for j in range(len(lst)):    # This for loop traverses the list to find the greenzone with least cost and sets it as min1

			k = math.sqrt(((start[0]-lst[j][0][0])**2)+((start[1]-lst[j][0][1])**2))
			if ( k < min1[1]):
				min1 = [lst[j][0],k,lst[j][1]]

	
		lst.remove([min1[0],min1[2]])
		path1 = get_path(start, get_coord(start, [min1[0],min1[2]]))   # creates a path between bot and min1(destination)
		min1 = [get_coord(start, [min1[0],min1[2]]),min1[1],min1[2]]
		e = 0
		a = 0
		d = 0
		f = 8                # f is the number of checkpoints in a path
		if min1[1]<=30:      # no checkpoints if the dest is very near to the bot
			f=1
		
		for x in range(f):                      # for-loop for every checkpoint
			lst3 = get_greenZone_list()
	
			for z in range(0,len(lst3)):   # This for loop checks if the current destiation still exixts in the greenzone_list     
				if ( (min1[0][0] >= lst3[z][0][0]) and (min1[0][0] <= lst3[z][2][0]) and (min1[0][1] >= lst3[z][0][1]) and (min1[0][1] <= lst3[z][2][1])):
					var.append(min1[2])
					e = 1
			if e == 0:
				break
			pos = get_botPose_list()
			beg = pos[botId]
			# print(pos)
			for l in range(len(pos)):      # This for loop checks its surrounding for other bots
				if l==botId:
					continue
				else:
					if ((pos[l][0]>(beg[0]-50)) and (pos[l][0]<(beg[0]+50)) and (pos[l][1]>(beg[1]-50)) and (pos[l][1]<(beg[1]+50))):      
						if (math.sqrt(((min1[0][0]-beg[0])**2)+((min1[0][1]-beg[1])**2)) > math.sqrt(((pos[l][0]-min1[0][0])**2)+((pos[l][1]-min1[0][1])**2))):     # If bot is present, it is checked which bot is nearer
							var.append(min1[2])  # if the other bot is nearer, its pos in orig_greenzone_list is appended to the Var and whole outerloop broken
							a = 1
							break
			if a==1:
				print(a)
				break

			min1[1] = path1[d:(int(len(path1)*((x+1)/f)))]    # if everything is satisfied, its path is executed according to the checkpoint
			d = (int(len(path1)*((x+1)/f)))                   # path is split into number of checkpoints
			level1(botId)



def level5(botId):
	global min1

	lst_org = get_original_greenZone_list()
	var = []
	check = True

	while check:
		lst2 = get_greenZone_list()
		lst=[]
		for i in range(len(lst2)):
			g=0
			sumx, sumy = 0, 0
			for n in range(len(var)):
				if (lst2[i] == (lst_org[var[n]])):
					g=1
					break
			if g==1:
				continue
			for j in range(4):
				sumx += lst2[i][j][0]
				sumy += lst2[i][j][1]
			lst.append([[int(sumx/4), int(sumy/4)],lst_org.index(lst2[i])])

		if (len(lst))==0:
			check = False
			continue
		
		start = get_botPose_list()[botId]
		min1= [[0,0],1000,0]
		for j in range(len(lst)):
			k = math.sqrt(((start[0]-lst[j][0][0])**2)+((start[1]-lst[j][0][1])**2))
			if ( k < min1[1]):
				min1 = [lst[j][0],k,lst[j][1]]
		lst.remove([min1[0],min1[2]])
		path1 = get_path(start, get_coord(start, [min1[0],min1[2]]))
		min1 = [get_coord(start, [min1[0],min1[2]]),min1[1],min1[2]]
		e = 0
		a = 0
		d = 0
		f = 8
		if min1[1]<=30:
			f=1
		
		for x in range(f):
			lst3 = get_greenZone_list()

			for z in range(0,len(lst3)):
				if ( (min1[0][0] >= lst3[z][0][0]) and (min1[0][0] <= lst3[z][2][0]) and (min1[0][1] >= lst3[z][0][1]) and (min1[0][1] <= lst3[z][2][1])):
					var.append(min1[2])
					e = 1
					break
			if e == 0:
				break
			pos = get_botPose_list()
			beg = pos[botId]
			for l in range(2):
				if l==botId:
					continue
				else:
					if ((pos[l][0]>(beg[0]-50)) and (pos[l][0]<(beg[0]+50)) and (pos[l][1]>(beg[1]-50)) and (pos[l][1]<(beg[1]+50))):
						if (math.sqrt(((min1[0][0]-beg[0])**2)+((min1[0][1]-beg[1])**2)) > math.sqrt(((pos[l][0]-min1[0][0])**2)+((pos[l][1]-min1[0][1])**2))):
							var.append(min1[2])
							a = 1
							break
			if a==1:
				print(a)
				break

			min1[1] = path1[d:(int(len(path1)*((x+1)/f)))]
			d = (int(len(path1)*((x+1)/f)))
			level1(botId)

def level6(botId):    
	global min1

	lst_org = get_original_greenZone_list()
	var = []
	check = True

	while check:
		lst2 = get_greenZone_list()
		lst=[]
		for i in range(len(lst2)):
			g=0
			sumx, sumy = 0, 0
			for n in range(len(var)):
				if (lst2[i] == (lst_org[var[n]])):
					g=1
					break
			if g==1:
				continue
			for j in range(4):
				sumx += lst2[i][j][0]
				sumy += lst2[i][j][1]
			lst.append([[int(sumx/4), int(sumy/4)],lst_org.index(lst2[i])])

		if (len(lst))==0:
			check = False
			continue
		
		start = get_botPose_list()[botId]
		min1= [[0,0],1000,0]

		for j in range(len(lst)):
			k = math.sqrt(((start[0]-lst[j][0][0])**2)+((start[1]-lst[j][0][1])**2))
			if ( k < min1[1]):
				min1 = [lst[j][0],k,lst[j][1]]

		lst.remove([min1[0],min1[2]])
		path1 = get_path(start, get_coord(start, [min1[0],min1[2]]))
		min1 = [get_coord(start, [min1[0],min1[2]]),min1[1],min1[2]]
		e = 0
		a = 0
		d = 0
		f = 8
		if min1[1]<=30:
			f=1
		
		for x in range(f):
			lst3 = get_greenZone_list()

			for z in range(0,len(lst3)):
				if ( (min1[0][0] >= lst3[z][0][0]) and (min1[0][0] <= lst3[z][2][0]) and (min1[0][1] >= lst3[z][0][1]) and (min1[0][1] <= lst3[z][2][1])):
					var.append(min1[2])
					e = 1
					break
			if e == 0:
				break
			pos = get_botPose_list()
			beg = pos[botId]
			for l in range(len(pos)):
				if l==botId:
					continue
				else:
					if ((pos[l][0]>(beg[0]-50)) and (pos[l][0]<(beg[0]+50)) and (pos[l][1]>(beg[1]-50)) and (pos[l][1]<(beg[1]+50))):
						if (math.sqrt(((min1[0][0]-beg[0])**2)+((min1[0][1]-beg[1])**2)) > math.sqrt(((pos[l][0]-min1[0][0])**2)+((pos[l][1]-min1[0][1])**2))):
							var.append(min1[2])
							a = 1
							break
			if a==1:
				print(a)
				break

			min1[1] = path1[d:(int(len(path1)*((x+1)/f)))]
			d = (int(len(path1)*((x+1)/f)))
			level1(botId)



#######    DON'T EDIT ANYTHING BELOW  #######################

if  __name__=="__main__":
	botId = int(sys.argv[1])
	level = get_level()
	if level == 1:
		level1(botId)
	elif level == 2:
		level2(botId)
	elif level == 3:
		level3(botId)
	elif level == 4:
		level4(botId)
	elif level == 5:
		level5(botId)
	elif level == 6:
		level6(botId)
	else:
		print("Wrong level! Please restart and select correct level")

