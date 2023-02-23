from pyray import *
import numpy as np
collision_map = {}
objects = []
WIDTH = 0
HEIGHT = 0
class Game:
	def __init__(self, width, height):
		WIDTH = width
		HEIGHT = height
	
	def initGraphics(self):
		init_window(800, 450, "Game")

	def mainLoop(self):
		while not window_should_close():
			begin_drawing()
			clear_background(WHITE)
			draw_text("Hello world", 190, 200, 20, VIOLET)
			end_drawing()
		close_window()

class ObjectHandler:
	def __init__(self):
		pass

	def createParticle(self, mass, position, velocity):
		objects.append(Particle(mass, position, velocity))

class ObjectIdentifier:
	nextFreeID = 0
	def __init__(self):
		self.id = nextFreeID
		nextFreeID += 1

	def getID(self):
		return self.id
	
class Particle:
	def __init__(self, mass, position, velocity):
		self.mass = mass
		self.position = np.array([position[0], position[1]])
		self.velocity = np.array([velocity[0], velocity[1]]) 
		self.object_identifier = ObjectIdentifier()
		self.id = self.object_identifier.getID()

	def updateCollisionMap(self):
		for x in np.arange(self.position[0],self.position[0]+self.velocity[0],1):
			for y in np.arange(self.position[1],self.position[1]+self.velocity[1],1):
				collision_map[(self.position + np.array([x,y])).round()].append(self.id)

	def checkCollisions(self):
		for x in np.arange(self.position[0],self.position[0]+self.velocity[0],1):
			for y in np.arange(self.position[1],self.position[1]+self.velocity[1],1):
				collision_map[(self.position + np.array([x,y])).round()]

	def collide(self, other):
		pass
"""
class Physics:
	def __init__(center, mass, rotation, rotation_anchor):
		self.center = center  # (float x, float y), m -- defines center of mass, absolute
		self.mass = mass  # float, kg
		self.rotation = rotation  # float, radians
		self.rotation_anchor = rotation_anchor # (float x, float y), m OR False -- defines anchor for anchored rotation
		
	def ApplySurfaceForceAngle(self, dt, location, angle, magnitude):
		pass

	def ApplySurfaceForceXY(self, dt, location, vector):
		pass

	def ApplyCOMForceAngle(self, dt, angle, magnitude):
		acceleration = (dt * magnitude) / self.mass
		self.center[0] += acceleration * math.cos(angle)
		self.center[1] += acceleration * math.sin(angle)

	def ApplyCOMForceXY(self, dt, vector):
		self.center[0] += dt * vector[0] / self.mass
		self.center[1] += dt * vector[1] / self.mass

class Object(Physics):
	def __init__(center, mass, rotation, rotation_anchor, bounding_box):
		pass
"""