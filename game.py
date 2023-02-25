from pyray import *
import numpy as np
from math import *
collision_map = {}
objects = []
FRAMERATE = 60
WIDTH = 100
HEIGHT = 50
class Game:
	def __init__(self):
		self.object_handler = ObjectHandler()

	def initGraphics(self):
		init_window(WIDTH, HEIGHT, "Game")

	def mainLoop(self):
		while not window_should_close():
			# clear tilemap
			collision_map.clear()

			# Clear screen
			begin_drawing()
			clear_background(WHITE)

			for obj in objects:
				obj.updateCollisionMap()
				obj.draw()

			end_drawing()

			for obj in objects: obj.updatePosition()

			for obj in objects: obj.checkCollisions()
			
		close_window()

class ObjectHandler:
	def __init__(self):
		pass

	def createParticle(self, mass, position, velocity, gravity, color):
		objects.append(Particle(mass, position, velocity, gravity, color))

class ObjectIdentifier:
	nextFreeID = 0
	def __init__(self):
		self.identifier = ObjectIdentifier.nextFreeID
		ObjectIdentifier.nextFreeID += 1

	def getID(self):
		return self.identifier

class Particle:
	def __init__(self, mass, position, velocity, gravity, color):
		self.mass = mass
		self.gravity = gravity
		self.position = np.array([float(position[0]), float(position[1])])
		self.velocity = np.array([float(velocity[0]), float(velocity[1])])
		self.object_identifier = ObjectIdentifier()
		self.identifier = self.object_identifier.getID()
		self.color = color

	def draw(self):
		draw_pixel_v(tuple(self.position.round()), self.color)

	def setVelocity(self, velocity):
		self.velocity = velocity

	def getPosition(self):
		return self.position

	def updatePosition(self):
		if self.gravity: self.velocity[1] += 9.81 / FRAMERATE
		self.position += self.velocity / FRAMERATE
		if self.position[0] > WIDTH:
			self.velocity[0] *= -1
			self.position[0] = WIDTH
		elif self.position[0] < 0:
			self.velocity[0] *= -1
			self.position[0] = 0
			
		if self.position[1] > HEIGHT:
			self.velocity[1] *= -1
			self.position[1] = HEIGHT
		elif self.position[1] < 0:
			self.velocity[1] *= -1
			self.position[1] = 0

	def updateCollisionMap(self):
		# for position in [pos for pos = self.position in pos <= self.position+self.velocity/FRAMERATE).append(self.position):
		# 	key = tuple(position).round()
		# 	if not key in collision_map.keys():
		# 		collision_map[key] = []
		# 	collision_map[key].append(self.identifier)
		
		for x in [self.position[0]] + list(np.arange(self.position[0],self.position[0]+self.velocity[0]/FRAMERATE,1)):
			for y in [self.position[1]] + list(np.arange(self.position[1],self.position[1]+self.velocity[1]/FRAMERATE,1)):
				key = tuple((self.position + np.array([x,y])).round())
				if not key in collision_map.keys():
					collision_map[key] = []
				collision_map[key].append(self.identifier)

	def checkCollisions(self):
		for x in [self.position[0]] + list(np.arange(self.position[0],self.position[0]+self.velocity[0]/FRAMERATE,1)):
			for y in [self.position[1]] + list(np.arange(self.position[1],self.position[1]+self.velocity[1]/FRAMERATE,1)):
				if not tuple((self.position + np.array([x,y])).round()) in collision_map.keys(): break
				for colliding_object in collision_map[tuple((self.position + np.array([x,y])).round())]:
					other = objects[colliding_object]
					if (colliding_object != self.identifier): #and np.linalg.norm(other.getPosition() - self.getPosition()) < 100+max(np.linalg.norm(self.velocity), np.linalg.norm(other.velocity))/(FRAMERATE)):
						for xx in [self.position[0]] + list(np.arange(self.position[0],self.position[0]+self.velocity[0]/FRAMERATE,1)):
							for yy in [self.position[1]] + list(np.arange(self.position[1],self.position[1]+self.velocity[1]/FRAMERATE,1)):
									if self.identifier in collision_map[tuple((self.position + np.array([xx,yy])).round())]: collision_map[tuple((self.position + np.array([xx,yy])).round())].remove(self.identifier)
						# print(collision_map[tuple((self.position + np.array([xx,yy])).round())])
						# print("COLLISION!!!!!" + str(self.identifier))
						self.collide(other)
						return

	def collide(self, other):
		v0 = self.velocity  # initial self velocity
		v1 = other.velocity  # initial other velocity
		a0 = atan2(v0[1], v0[0])  # initial self angle
		a1 = atan2(v1[1], v1[0])  # initial other angle
		r0 = self.position  # self position
		r1 = other.position  # other position
		m0 = self.mass  # mass of self
		m1 = other.mass  # mass of other
		# print(v0 - ((2 * m1)/(m0+m1)) * ((np.dot(v0-v1, r0-r1))/(np.linalg.norm(r0-r1)**2)) * (r0-r1))
		self.setVelocity((v0 - ((2 * m1)/(m0+m1)) * ((np.dot(v0-v1, r0-r1))/(np.linalg.norm(r0-r1)**2)) * (r0-r1)))
		# print(v1 - ((2 * m0)/(m0+m1)) * ((np.dot(v0-v1, r0-r1))/(np.linalg.norm(r1-r0)**2)) * (r1-r0))
		other.setVelocity((v1 - ((2 * m0)/(m0+m1)) * ((np.dot(v0-v1, r0-r1))/(np.linalg.norm(r1-r0)**2)) * (r1-r0)))
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
