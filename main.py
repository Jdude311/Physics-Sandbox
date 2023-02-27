from game import *
from random import randrange

game = Game()
game.initGraphics()
for i in range(50):
    game.object_handler.createParticle(randrange(50,100)/1000, np.array([randrange(0,1000)/100, randrange(0,500)/100]), np.array([randrange(-100,100)/100,randrange(-100,100)/100]), False, (randrange(0,255), randrange(0,255), randrange(0,255), 255))

game.object_handler.createParticle(0.1, np.array([3, 3]), np.array([1,-1]), False, RED)
game.object_handler.createParticle(0.1, np.array([4, 3.5]), np.array([-1,-2]), False, BLUE)
# game.object_handler.createParticle(0.1, np.array([4, 4]), np.array([0,0]), False, BLUE)
game.mainLoop()
