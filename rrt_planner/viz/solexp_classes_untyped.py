# coding: utf8
"""
VIZ module
written by Grégoire Roussel on Jan. 2018 to help develop BoxRRT on burger
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import collections as mc



#############################################
##          EXPERIENCE CLASS               ##
#############################################

class Experience:
	"""
	represents an RRT-STAR experience
	The objective is to display the results to understand if the algorithm performed well
	"""

	def __init__(self):
		self.dim= (0, 0, 0, 0)
		self.title= "title"
		self.start= (0, 0)
		self.goal = (0, 0, 0, 0)
		self.obstacles= []

	def configFromFile(self, filename):
		"""
		load experience and stores it into a experience class
		"""
		self.dim = (0, 0, 0, 0)
		self.start = (0, 0)
		self.goal= (0, 0, 0, 0)
		self.obstacles = []
		expfile = open(filename, "r")
		title = expfile.readline().strip('\n')
		self.title = title

		for line in expfile:
			# print(line)
			param = line.strip('\n').split(" ")
			print(param)
			# Handling the param
			action = param[0]
			x = float(param[1])
			y = float(param[2])

			if(action == 'D'):
				# dimensions
				self.dim = (x, y, float(param[3]), float(param[4]))
			elif(action == 'S'):
				# start
				self.start = (x, y)
			elif (action == 'G'):
				self.goal = (x, y, float(param[3]), float(param[4]))
			elif (action == 'O'):
				self.obstacles.append((x, y, float(param[3]), float(param[4])))
			else:
				print("unrecognized action in" + param)
		expfile.close()

	def getBorderObstacles(self):
		"""
		creates 4 obstacles to limit the state space
		order: bottom, , left, right, up
		first is greedier (takes common squares if possible)
		"""
		x, y, w, h = self.dim
		return [(x -1, y - 1, w+2 , 1), (x - 1, y, 1, h + 1),
				(x + w, y, 1, h + 1), (x, y + h, w, 1)]

	def __str__(self):
		string = "XP[" + self.title + "] " + str(self.dim) + " S=" + \
			str(self.start) + " G=" + str(self.goal) + \
			" O=" + str(self.obstacles)
		return string

	def display(self, show=True):
		"""
		creates a Matplorlib context to display experience
		"""
		# plt.ioff()
		fig = plt.figure()
		ax = fig.add_subplot(111, aspect="equal")
		x, y, w, h = self.dim
		ax.axis([x - 1, w + 1, y - 1, h + 1])
		ax.set_title(self.title)
		ax.plot(self.start[0], self.start[1], "r.")
		# ax.plot(exp.goal[0], exp.goal[1], "b.")

		borderObstacles = self.getBorderObstacles()
		ax.add_patch(patches.Rectangle(
			(self.goal[0], self.goal[1]), self.goal[2], self.goal[3], facecolor='green'))

		for x, y, w, h in self.obstacles:
			ax.add_patch(patches.Rectangle((x, y), w, h))

		for x, y, w, h in borderObstacles:
			ax.add_patch(patches.Rectangle(
				(x, y), w, h, facecolor="black", alpha=0.7))

		if(show):
			plt.show()
		return (fig, ax)


#############################################
##            SOLUTION CLASS               ##
#############################################

class Solution:
	"""
	represents an RRT-star solution
	contains the related experience file path;
	the computed optimal path
	and representation of the tree
	"""

	def __init__(self):
		self.exp = 0
		self.path = []
		self.tree = []
		self.title = "voidTitle"

	def configFromFile(self, solFileName):
		"""
		load experience and stores it into a experience class
		"""
		solFile = open(solFileName, "r")
		self.title = solFile.readline().strip('\n')
		expFileName = solFile.readline().strip('\n')
		self.exp = Experience()
		self.exp.configFromFile(expFileName)

		self.path = []
		self.tree = []

		for line in solFile:
			# print(line)
			param = line.strip('\n').split(" ")
			# print(param)
			# Handling the param
			action = param[0]
			x = float(param[1])
			y = float(param[2])
			if(action == 'W'):
				# waypoint
				self.path.append([x, y])
			elif(action == 'T'):
				# tree
				x2 = float(param[3])
				y2 = float(param[4])
				self.tree.append([(x, y), (x2, y2)])
			else:
				print("unrecognized action in" + param)
		solFile.close()

	def display(self, displayTree=True, displayPath=True):

		fig, ax = self.exp.display(show=False)
		if(displayPath):
			pathX, pathY = zip(*self.path)
			ax.plot(pathX, pathY, 'g-')

		if(displayTree):
			lineCol = mc.LineCollection(self.tree, linewidths=0.5, colors='lightblue')
			ax.add_collection(lineCol)

		plt.show()
		return (fig, ax)
