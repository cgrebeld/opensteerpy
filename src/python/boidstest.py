#!/usr/bin/env python

###
# Simple test of opensteer module
# boids.Boid have flocking behaviour (like Boids.cpp plugin)
# They are constrained a screen-sized region of the XZ plane
##
import sys
import time
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from random import randint
import boids
import opensteer as os

doc = """
Press F1 to add a boid.
Press F2 to remove one.
"""
screen = None
worldRad = 60.0
screenRad = 400
screenSize = screenRad*2,screenRad*2
pixPerUnit = screenRad / worldRad
fg = 0.8,0.8,0.8,1
bg = 0.1,0.1,0.1,1.0
numBoids = 30

def worldToScreen(x,y):
    return (int(x * pixPerUnit + screenRad), int(y * pixPerUnit + screenRad))
def worldVecToScreen(v):
    return worldToScreen(v.x,v.z)
def worldDistToScreen(d):
    return int(d*pixPerUnit)

class BoxObstacle(os.BoxObstacle): 
    """ For some swigish reason, only classes derived from os.BoxObstacle work """
    pass
class SphereObstacle(os.SphereObstacle):
    pass

class BoidGadget(boids.Boid):
    def __init__(self):
        boids.Boid.__init__(self)
        self.setRadius(1.0)

    def draw(self):
        p = self.position()
        r = self.radius()
        f = self.forward() * (r)
        s = self.side() * r
        a = p + f
        b = p + s
        c = p - s
        glColor4f(*fg)
        glBegin(GL_LINE_LOOP)
        glVertex(worldVecToScreen(a))
        glVertex(worldVecToScreen(b))
        glVertex(worldVecToScreen(c))
        glEnd()

def updateAndDraw(currentTime, elapsedTime):
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    for b in boids.flock:
        b.update(currentTime,elapsedTime)
        b.draw()
    pygame.display.flip()

def addBoid():
    b= BoidGadget()
    b.setMaxForce(27)
    b.setMaxSpeed(9.0)
    b.moveTo(randint(-worldRad,worldRad), 0, randint(-worldRad,worldRad))
    boids.flock.append(b)
    
def initializeWorld():
    global worldRad
    boids.createWorld(worldRad)
    for n in xrange(numBoids):
        addBoid()
    # big box that encloses the world
    box = BoxObstacle(worldRad*1.8,5,worldRad*1.8)
    boids.obstacles.append(box)
#    sph = SphereObstacle(worldRad, os.Vec3.zero)
#    sph.setSeenFrom(sph.both)
#    boids.obstacles.append(sph)


def initializeDisplay(w, h):
    pygame.display.set_mode((w,h), pygame.OPENGL|pygame.DOUBLEBUF)
    glClearColor(0.0, 0.0, 0.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, w, 0, h);
    glMatrixMode(GL_MODELVIEW);

def loop():
    t = time.clock()
    glClearColor(*bg)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_F1:
                    addBoid()
                    print("%d boids"%len(boids.flock))
                elif event.key == pygame.K_F2:
                    boids.flock.pop()
                    print("%d boids"%len(boids.flock))

        t2 = time.clock()
        elapsedTime = t2 - t
        t = t2
        updateAndDraw(t, elapsedTime)

def main():
    try:
        global numBoids
        numBoids = int(sys.argv[1])
    except IndexError: pass

    print("Using %d boids"%numBoids)
    print(doc)

    pygame.init()
    initializeDisplay(*screenSize)
    initializeWorld()
    loop()

if __name__ == '__main__':
    main()
