import cv2
import numpy as np
import heapq
import time

class Node():
	def __init__(self, coord) :
		self.coord = coord
		self.weight = np.inf
		self.parent = None

	def print_me(self):
		print("Coord :", self.coord)
		print("Weight :", self.weight)
	
	def __lt__(self, nxt):
		return self.weight < nxt.weight

def backtrack(node, amg) :
	pts = []
	cost = node.weight
	while node.parent != None :
		print(f'Y coord : {node.coord[0]} and X coord : {node.coord[1]}')
		pts.append([node.coord[1], node.coord[0]])
		node = node.parent

	npts = pts[::-1]
	for pt in npts :
		amg[249-pt[0]][pt[1]] = [0,255,0]
		cv2.imshow('Path', amg)
		cv2.waitKey(1)

	cv2.imshow('Path', amg)
	cv2.waitKey(0)

	print(f'Cost {cost}')
	return pts, amg

def eq(x1,y1,x2,y2,x,y,f):
	m = (y2-y1)/(x2-x1)
	if (f == 1):
		c = (m*x) - y <= (m*x1) - y1
	else:
		c = (m*x) - y >= (m*x1) - y1
	return c

def create_map():
	m = np.zeros((250,400))
	am = np.zeros((250,400,3))
	hl = 40.4145
	for y in range(m.shape[0]):
		for x in range(m.shape[1]):
			if (((y - 65) ** 2) + ((x - 300) ** 2) <= ((40) ** 2) ):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if ((x > 200-35) and (x < 200 + 35) and (y <= 150) and eq(200,150-hl,165,150-(hl/2),x,y,1) and eq(200,150-hl,235,150-(hl/2),x,y,1) ):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if ((x > 200-35) and (x < 200 + 35) and (y >= 150) and eq(200,150+hl,165,150+(hl/2),x,y,2) and eq(200,150+hl,235,150+(hl/2),x,y,2) ):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(80,70,105,150,x,y,1) and (y >= 250-180))):
				m[y,x]=1
				am[y,x]=[0,0,255]
			if (eq(36,65,115,40,x,y,1) and eq(36,65,105,150,x,y,2) and (eq(115,40,80,70,x,y,2) and (y <= 250-180))):
				m[y,x]=1
				am[y,x]=[0,0,255]

	return m,am

def inObs(x,y,img) :
	if 250-(y+5) > 0 :
		if img[249-y-5][x] == 1 : return True
		
	if 250-(y-5) < 250 :
		if img[249-(y-5)][x] == 1 : return True
		
	if 250-(y+5) > 0 and  x+5 < 399 :
		if img[249-(y+5)][x+5] == 1 : return True
	
	if 250-(y+5) > 0 and x-5 > 0 :
		if img[249-(y+5)][x-5] == 1 : return True
	
	if 250-(y-5) < 250 and x+5 < 399:
		if img[249-(y-5)][x+5] == 1 : return True

	if 250-(y-5) < 250 and x-5 > 0: 
		if img[249-(y-5)][x-5] == 1 : return True
	
	if x-5 > 0 :
		if img[249-y][x-5] == 1 : return True
	
	if x+5 < 399:
		if img[249-y][x+5] == 1 : return True
		
	return False


def checkLeft(node, im) :
	img = im.copy()
	x, y = node.coord[0], node.coord[1]
	xc, yc = x-1,y
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else : return False

def checkRight(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x+1,y
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else : return False

def checkUp(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x,y+1
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else : return False

def checkDown(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x,y-1
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else : return False

def checkdr(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x+1,y-1
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else :
		return False

def checkdl(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x-1,y-1
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else : return False

def checkur(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x+1,y+1
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else :return False

def checkul(node, img) :
	x, y = node.coord[0], node.coord[1]
	xc, yc = x-1,y+1
	if xc >= 0 and xc < 400 and yc >= 0 and yc < 250 :
		return (not inObs(xc,yc,img))
	else : return False
		

def findNeigh(node, visited, nodelist, obs):
	x, y = node.coord[0], node.coord[1]
	ne = []
	lm = checkLeft(node,obs)
	rm = checkRight(node,obs)
	um = checkUp(node,obs)
	dm = checkDown(node,obs)
	drm = checkdr(node,obs)
	dlm = checkdl(node,obs)
	urm = checkur(node,obs)
	ulm = checkul(node,obs)
	
	if lm : 
		lnode = Node([x-1,y])
		xc, yc = x-1,249-y
		if not visited[yc][xc] and [x-1,y] not in nodelist :
			ne.append([lnode,1])

	if um : 
		xc, yc = x,249-(y+1)
		unode = Node([x,y+1])
		if not visited[yc][xc] and [x,y+1] not in nodelist:
			ne.append([unode,1])

	if rm : 
		xc, yc = x+1,249-y
		rnode = Node([x+1,y])
		if not visited[yc][xc] and [x+1,y] not in nodelist:
			ne.append([rnode,1])

	if dm : 
		xc, yc = x,249-(y-1)
		dnode = Node([x,y-1])
		if not visited[yc][xc] and [x,y-1] not in nodelist:
			ne.append([dnode,1])

	if urm : 
		xc, yc = x+1,249-(y+1)
		urnode = Node([x+1,y+1])
		if not visited[yc][xc] and [x+1,y+1] not in nodelist:
			ne.append([urnode,1.4])

	if ulm : 
		xc, yc = x-1,249-(y+1)
		ulnode = Node([x-1,y+1])
		if not visited[yc][xc] and [x-1,y+1] not in nodelist:
			ne.append([ulnode,1.4])

	if drm : 
		xc, yc = x+1,249-(y-1)
		drnode = Node([x+1,y-1])
		if not visited[yc][xc] and [x+1,y-1] not in nodelist:
			ne.append([drnode,1.4])

	if dlm : 
		xc, yc = x-1,249-(y-1)
		dlnode = Node([x-1,y-1])
		if not visited[yc][xc] and [x-1,y-1] not in nodelist:
			ne.append([dlnode,1.4])
		
	return ne

def explore(aimg, pts) :
	for pt in pts :
		amg[pt[0]][pt[1]] = [255,255,255]
		cv2.imshow('Path', amg)
		cv2.waitKey(1)

	return amg

def dijkstra(ipoint, fpoint, im, am) :
	img = im.copy()
	amg = am.copy()
	visited = np.zeros((250,400), dtype=bool)
	inode = Node(ipoint)
	inode.weight = 0
	nlist = [inode]
	heapq.heapify(nlist)
	pts = []
	allpts = []
	nodelist = [ipoint]

	while len(nlist) != 0 :
		pnode = nlist[0]
		pcoord = pnode.coord
		pweight = pnode.weight

		visited[249-pcoord[1]][pcoord[0]] = True
		heapq.heappop(nlist)
		if pcoord == fpoint[::-1] :
			print('Goal Reached')
			amg = explore(amg, allpts)
			pts, amg = backtrack(pnode, amg)
			cv2.imwrite('Path.jpg', amg)
			break 
		else :
			img1 = img.copy()
			children = findNeigh(pnode, visited, nodelist, img1)
			for i,cbox in enumerate(children) : 
				cnode = cbox[0]
				wt = cbox[1]
				cweight = cnode.weight
				ccoord = cnode.coord
				nodelist.append(ccoord)
				# print(ccoord)

				if cweight > pweight+wt :
					cweight = pweight+wt
					cnode.weight = pweight+wt
					cnode.parent = pnode
					allpts.append([249-ccoord[1], ccoord[0]])
					heapq.heappush(nlist, cnode)

	return pts

if __name__ == "__main__" :
	stime = time.time()
	img, amg= create_map()
	cv2.imwrite('img.jpg', amg)

	ipointx = input('Enter start x point')
	ipointy = input('Enter start y point')

	fpointx = input('Enter final x point')
	fpointy = input('Enter final y point')
	ipoint = [int(ipointx),int(ipointy)]
	fpoint = [int(fpointy),int(fpointx)]
	print(ipoint[0], ipoint[1], fpoint[0], fpoint[1])
	
	if (ipoint[0] < 0 or ipoint[0] > 399 or ipoint[1] < 0 or ipoint[1] > 249) or (fpoint[0] < 0 or fpoint[0] > 249 or fpoint[1] < 0 or fpoint[1] > 399):
		print('Out of bounds')
	else :
		if img[ipoint[1]][ipoint[0]] == 1 or img[fpoint[0]][fpoint[1]] == 1 :
			print('Given points in obstacle space')
		else :
			pts = dijkstra(ipoint, fpoint, img, amg)
	etime = time.time()
	print(f'Total Time {etime-stime}')
