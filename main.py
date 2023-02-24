from game import *
from random import randrange

WIDTH = 800
HEIGHT = 450
game = Game(800,450)
game.initGraphics()
for i in range(100):
    game.object_handler.createParticle(randrange(1,100), np.array([randrange(0,800), randrange(0,450)]), np.array([randrange(-10,10),randrange(-10,10)]), (randrange(0,255), randrange(0,255), randrange(0,255), 255))

game.object_handler.createParticle(1, np.array([300.0, 300.0]), np.array([-1,-1]), BLUE)
game.mainLoop()
