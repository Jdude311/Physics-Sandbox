from pyray import *
import numpy as np
from math import *
import itertools
collision_map = {}
objects = []
FRAMERATE = 165
WIDTH_PIXELS = 1000
HEIGHT_PIXELS = 500
# SCALE = 5 # pixels per meter
# WIDTH_METERS = WIDTH_PIXELS / SCALE
# HEIGHT_METERS = HEIGHT_PIXELS / SCALE
WIDTH_METERS = 10
SCALE = WIDTH_PIXELS / WIDTH_METERS # pixels per meter
HEIGHT_METERS = HEIGHT_PIXELS / SCALE

GRAVITY = 9.81
class Game:
	def __init__(self):
		self.object_handler = ObjectHandler()

	def initGraphics(self):
		init_window(WIDTH_PIXELS, HEIGHT_PIXELS, "Game")
		set_target_fps(FRAMERATE)

	def mainLoop(self):
		while not window_should_close():
			FRAMERATE = get_fps()

			# clear tilemap
			collision_map.clear()

			for obj in objects:
				obj.updateCollisionMap()

			for obj in objects:
				obj.checkCollisions()

			# Clear screen
			begin_drawing()
			clear_background(WHITE)

			for obj in objects:
				obj.draw()

			end_drawing()


			for obj in objects: obj.updatePosition()

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
		self.radius = self.mass
		self.gravity = gravity
		self.position = np.array([float(position[0]), float(position[1])])
		self.velocity = np.array([float(velocity[0]), float(velocity[1])])
		self.object_identifier = ObjectIdentifier()
		self.identifier = self.object_identifier.getID()
		self.color = color

	def draw(self):
		draw_circle_v(tuple((self.position * SCALE).round() ), self.radius * SCALE, self.color)

	def setVelocity(self, velocity):
		self.velocity = velocity

	def getPosition(self):
		return self.position

	def updatePosition(self):
		if self.gravity: self.velocity[1] += GRAVITY / FRAMERATE
		self.position += self.velocity / FRAMERATE
		if self.position[0] > WIDTH_METERS:
			self.velocity[0] *= -1
			self.position[0] = WIDTH_METERS
		elif self.position[0] < 0:
			self.velocity[0] *= -1
			self.position[0] = 0

		if self.position[1] > HEIGHT_METERS:
			self.velocity[1] *= -1
			self.position[1] = HEIGHT_METERS
		elif self.position[1] < 0:
			self.velocity[1] *= -1
			self.position[1] = 0

	def updateCollisionMap(self):
		# for position in [pos for pos = self.position in pos <= self.position+self.velocity/FRAMERATE).append(self.position):
		# 	key = tuple(position).round()
		# 	if not key in collision_map.keys():
		# 		collision_map[key] = []
		# 	collision_map[key].append(self.identifier)

		check_position = self.position.copy()
		key = check_position.round()
		increment = np.array((0,0) if np.linalg.norm(self.velocity/FRAMERATE) == 0 else self.velocity/np.linalg.norm(self.velocity/FRAMERATE))
		while True:
			key = tuple(check_position.round())
			if not key in collision_map.keys():
				collision_map[key] = []
			collision_map[key].append(self.identifier)

			# Increment and check if final position reached yet
			check_position += increment
			if (increment == (0,0)).all(): break
			if np.sign(self.velocity[0]) * (check_position[0] - (self.position + (self.velocity/FRAMERATE))[0]) >= 0:
				if np.sign(self.velocity[1]) * (check_position[1] - (self.position + (self.velocity/FRAMERATE))[1]) >= 0:
					break

	def removeFromCollisionMap(self):
		check_position = self.position.copy()
		key = tuple(check_position.round())
		increment = np.array((0,0) if np.linalg.norm(self.velocity/FRAMERATE) == 0 else self.velocity/np.linalg.norm(self.velocity/FRAMERATE))
		while True:
			key = tuple(check_position.round())
			if key in collision_map.keys() and self.identifier in collision_map[key]:
				collision_map[key].remove(self.identifier)

			# Increment and check if final position reached yet
			check_position += increment
			if (increment == (0,0)).all(): break
			if np.sign(self.velocity[0]) * (check_position[0] - (self.position + (self.velocity/FRAMERATE))[0]) >= 0:
				if np.sign(self.velocity[1]) * (check_position[1] - (self.position + (self.velocity/FRAMERATE))[1]) >= 0:
					break

	def checkCollisions(self):
		self.removeFromCollisionMap()  # Other object will not need to check this one for collisions anymore
		check_position = self.position.copy()
		key = check_position.round()
		increment = np.array((0,0) if np.linalg.norm(self.velocity/FRAMERATE) == 0 else self.velocity/np.linalg.norm(self.velocity/FRAMERATE))
		while True:
			key = tuple(check_position.round())
			if key in collision_map.keys():
				for colliding_object in set(collision_map[key]):
					other = objects[colliding_object]
					if other.identifier == self.identifier: continue
					print("calling collide")
					if np.linalg.norm(other.getPosition() - check_position) <= self.radius + other.radius:
						self.collide(other)
					print("collide called")
					break
				break

			# Increment and check if final position reached yet
			check_position += increment
			if (increment == (0,0)).all(): break
			if np.sign(self.velocity[0]) * (check_position[0] - (self.position + (self.velocity/FRAMERATE))[0]) >= 0:
				if np.sign(self.velocity[1]) * (check_position[1] - (self.position + (self.velocity/FRAMERATE))[1]) >= 0:
					break
		# for x in [self.position[0]] + list(np.arange(self.position[0],self.position[0]+self.velocity[0]/FRAMERATE,1)):
		# 	for y in [self.position[1]] + list(np.arange(self.position[1],self.position[1]+self.velocity[1]/FRAMERATE,1)):
		# 		if not tuple((self.position + np.array([x,y])).round()) in collision_map.keys(): break
		# 		for colliding_object in collision_map[tuple((self.position + np.array([x,y])).round())]:
		# 			other = objects[colliding_object]
		# 			if (colliding_object != self.identifier): #and np.linalg.norm(other.getPosition() - self.getPosition()) < 100+max(np.linalg.norm(self.velocity), np.linalg.norm(other.velocity))/(FRAMERATE)):
		# 				for xx in [self.position[0]] + list(np.arange(self.position[0],self.position[0]+self.velocity[0]/FRAMERATE,1)):
		# 					for yy in [self.position[1]] + list(np.arange(self.position[1],self.position[1]+self.velocity[1]/FRAMERATE,1)):
		# 						if self.identifier in collision_map[tuple((self.position + np.array([xx,yy])).round())]: collision_map[tuple((self.position + np.array([xx,yy])).round())].remove(self.identifier)
		# 				# print(collision_map[tuple((self.position + np.array([xx,yy])).round())])
		# 				# print("COLLISION!!!!!" + str(self.identifier))
		# 				self.collide(other)
		# 				return

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
