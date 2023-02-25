from game import *
from random import randrange

game = Game()
game.initGraphics()
for i in range(100):
    game.object_handler.createParticle(randrange(1,10000), np.array([randrange(0,100), randrange(0,50)]), np.array([randrange(-10,10),randrange(-10,10)]), True, (randrange(0,255), randrange(0,255), randrange(0,255), 255))

game.object_handler.createParticle(1, np.array([300.0, 300.0]), np.array([1,-1]), False, RED)
game.object_handler.createParticle(10, np.array([400, 350]), np.array([-1,-2]), False, BLUE)
game.mainLoop()
