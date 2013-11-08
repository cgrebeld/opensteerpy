import opensteer as os

minTimToCollision = 0.5

_worldRadius = 0.0
_db = None
obstacles = []
flock = []

class Boid(os.TrivialVehicle):
    """ basic flocker """
    def __init__(self):
        os.TrivialVehicle.__init__(self)
        # a pointer to this boid's interface object for the proximity database
        self._proximityToken = _db.allocateToken(self)
        self.reset()

    def reset(self):
        os.TrivialVehicle.reset(self)
        # steering force is clipped to this magnitude
        self.setMaxForce(8.0)
        # velocity is clipped to this magnitude
        self.setMaxSpeed(2.0)
        self.setRadius(0.5)
        # initial slow speed
        self.setSpeed(self.maxSpeed() * 0.3)
        # randomize initial orientation
        self.randomizeHeadingOnXZPlane()
        v = os.RandomUnitVectorOnXZPlane() * 10
        # randomize initial position
        self.setPosition(v)
        # notify proximity database that our position has changed
        self._proximityToken.updateForNewPosition(self.position())

    def moveTo(self,x,y,z):
        self.setPosition(os.Vec3(x,y,z))
        # notify proximity database that our position has changed
        self._proximityToken.updateForNewPosition(self.position())
        
    def __str__(self):
        p = self.position()
        return 'pos=(%f,%f,%f), @ %f'%(p.x,p.y,p.z,self.speed())

    def update(self, currentTime, elapsedTime):
        # steer to flock and avoid obstacles if any
        v = self.steerToFlock()
        v.setYtoZero()
        self.applySteeringForce (v, elapsedTime)
        # wrap around to contrain boid within the spherical boundary
        self.sphericalWrapAround ()
        # notify proximity database that our position has changed
        self._proximityToken.updateForNewPosition (self.position());

    def steerToFlock(self):
        # basic flocking
        # avoid obstacles if needed
        avoidance = self.steerToAvoidObstacles (minTimToCollision, obstacles)
        if avoidance != os.Vec3.zero:
            return avoidance;
        separationRadius =  5.0
        separationAngle  = -0.707
        separationWeight =  12.0

        alignmentRadius = 7.5
        alignmentAngle  = 0.7
        alignmentWeight = 8.0

        cohesionRadius = 9.0
        cohesionAngle  = -0.15
        cohesionWeight = 8.0

        maxRadius = max (separationRadius,
                         max (alignmentRadius,
                              cohesionRadius))

        # find all flockmates within maxRadius using proximity database
        neighbors = self._proximityToken.findNeighbors (self.position(), maxRadius);

        # determine each of the three component behaviors of flocking
        separation = self.steerForSeparation (separationRadius,
                                              separationAngle,
                                              neighbors)
        alignment  = self.steerForAlignment  (alignmentRadius,
                                              alignmentAngle,
                                              neighbors)
        cohesion   = self.steerForCohesion   (cohesionRadius,
                                              cohesionAngle,
                                              neighbors)

        # apply weights to components (save in variables for annotation)
        separationW = separation * separationWeight
        alignmentW = alignment * alignmentWeight
        cohesionW = cohesion * cohesionWeight
        return separationW + alignmentW + cohesionW

    def sphericalWrapAround(self):
        pos = self.position()
        if pos.length() > _worldRadius:
            pos.sphericalWrapAround(os.Vec3.zero, _worldRadius)
            pos.setYtoZero()
            self.setPosition(pos)


def createWorld(worldRadius):
    global _worldRadius, _db
    _worldRadius = worldRadius
    _db = os.ProximityDatabase(os.Vec3(0,0,0),
                               os.Vec3(worldRadius,worldRadius,worldRadius), 
                               os.Vec3(100,100,100))

