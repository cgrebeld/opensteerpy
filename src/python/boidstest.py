#!/usr/bin/env python

###
# Simple test of opensteer module
# boids.Boid have flocking behaviour (like Boids.cpp plugin)
# They are constrained a screen-sized region of the XZ plane
##
import sys, time, math
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

# assuming 800x800
screenRad = 400
screenSize = screenRad*2,screenRad*2
pixPerUnit = screenRad / worldRad
fg = 0.8,0.8,0.8,1
bg = 0.1,0.1,0.1,1.0
numBoids = 15

def worldToScreen(x,y):
    return (int(x * pixPerUnit), int(y * pixPerUnit))
def worldVecToScreen(v):
    return worldToScreen(v.x,v.z)
def worldDistToScreen(d):
    return int(d*pixPerUnit)

class BoxObstacle(os.BoxObstacle): 
    """ For some swigish reason, only classes derived from os.BoxObstacle work """
    pass
class SphereObstacle(os.SphereObstacle):
    pass

def drawCicle(posV, r):
    return (worldToScreen(posV.x + r*math.cos(a/100), posV.z + r*math.sin(a/100)) for a in range(0, int(2*3.14159*100), int(0.2*100)))

class BoidGadget(boids.Boid):
    def __init__(self):
        boids.Boid.__init__(self)
        self.setRadius(2.0)

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
        for v in drawCicle(p, r):
            glVertex(v);
        glEnd()

        pV = worldVecToScreen(p)
        glBegin(GL_LINES)
        glColor4f(1,1,1,1)
        glVertex(pV)
        glVertex(worldVecToScreen(p + self.forward() * self.speed()))
        glColor4f(1,0,0,1)
        glVertex(pV)
        glVertex(worldVecToScreen(p + self.avoidance))
        glColor4f(0,1,0,1)
        glVertex(pV)
        glVertex(worldVecToScreen(p + self.separation))
        glColor4f(0,0,1,1)
        glVertex(pV)
        glVertex(worldVecToScreen(p + self.cohesion))
        
        glEnd()


def drawBox(pos, r):
    glColor4f(1,1,1,1)
    glBegin(GL_LINE_LOOP)
    vtx = ((-r, r),
           (r, r),
           (r, -r),
           (-r, -r))
    map(glVertex, [worldVecToScreen(os.Vec3(v[0],0,v[1]) + pos) for v in vtx])
    glEnd()

def updateAndDraw(currentTime, elapsedTime):
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    for b in boids.flock:
        b.update(currentTime,elapsedTime)
        b.draw()
    drawBox(os.Vec3.zero, worldRad - 2)
    drawBox(os.Vec3(40,-20,0), 10)

def addBoid():
    b= BoidGadget()
    b.setMaxForce(27)
    b.setMaxSpeed(20.0)
    b.setSpeed(randint(-20, 20));
    b.moveTo(randint(-worldRad,worldRad), 0, randint(-worldRad,worldRad))
    boids.flock.append(b)
    
def initializeWorld():
    global worldRad
    boids.createWorld(worldRad)
    for n in xrange(numBoids):
        addBoid()
    # big box that encloses the world
            #box = BoxObstacle(worldRad-2,worldRad-2,worldRad-2)
    r = worldRad - 2
    box = BoxObstacle(r*2, r*2, r*2)
    box.setSeenFrom(box.inside)
    box.setPosition(os.Vec3(0,0,0))
    boids.obstacles.append(box)
    box = BoxObstacle(20, 20, 20)
    box.setSeenFrom(box.outside)
    box.setPosition(os.Vec3(40,-20,0))
    boids.obstacles.append(box)


def bumpBoids():
    for b in boids.flock:
        b.applySteeringForce(os.Vec3(randint(-2, 2), 0, randint(-2,2)), 1.0)

def initializeDisplay(w, h):
    pygame.display.set_mode((w,h), pygame.OPENGL|pygame.DOUBLEBUF)
    glClearColor(0.0, 0.0, 0.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-w/2, w/2, -h/2, h/2);
    glMatrixMode(GL_MODELVIEW);
    print "init display",w,"x",h

def loop():
    t = time.clock()
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
                elif event.key == pygame.K_F3:
                    bumpBoids()

        t2 = time.clock()
        elapsedTime = t2 - t
        t = t2
        glClearColor(*bg)
        updateAndDraw(t, elapsedTime)
        pygame.display.flip()

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
