#ifndef OPENSTEER_LOCALSPACEOBSTACLES_H
#define OPENSTEER_LOCALSPACEOBSTACLES_H


#include "OpenSteer/Vec3.h"
#include "OpenSteer/LocalSpace.h"
#include "OpenSteer/AbstractVehicle.h"


namespace OpenSteer {
        // ----------------------------------------------------------------------------
    // LocalSpaceObstacle: a mixture of LocalSpace and Obstacle methods

     typedef LocalSpaceMixin<Obstacle> LocalSpaceObstacle;
    // ----------------------------------------------------------------------------
    // BoxObstacle: a box-shaped (cuboid) obstacle of a given height, width,
    // depth, position and orientation.  The box is centered on and aligned
    // with a local space.


     class BoxObstacle : public LocalSpaceObstacle
    {
    public:
        float width;  // width  of box centered on local X (side)    axis
        float height; // height of box centered on local Y (up)      axis
        float depth;  // depth  of box centered on local Z (forward) axis

        // constructors
        BoxObstacle (float w, float h, float d) : width(w), height(h), depth(d) {}
        BoxObstacle (void) :  width(1.0f), height(1.0f), depth(1.0f) {}

        virtual ~BoxObstacle() { /* Nothing to do. */ }
        
        
        // find first intersection of a vehicle's path with this obstacle
        void findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                              AbstractObstacle::PathIntersection& pi)
            const;
    };


    // ----------------------------------------------------------------------------
    // PlaneObstacle: a planar obstacle of a given position and orientation.
    // The plane is defined as the XY (aka side/up) plane of a local space.
    // The +Z (forward) half-space is considered "outside" the obstacle.  
    //
    // This is also the base class for several other obstacles which represent
    // 2d shapes (rectangle, triangle, ...) arbitarily oriented and positioned
    // in 2d space.  They specialize this class via xyPointInsideShape which
    // tests if a given point on the XZ plane is inside the obstacle's shape.


    class PlaneObstacle : public LocalSpaceObstacle
    {
    public:
        // constructors
        PlaneObstacle (void) {}
        PlaneObstacle (const Vec3& s,
                       const Vec3& u,
                       const Vec3& f,
                       const Vec3& p)
        : LocalSpaceObstacle( s, u, f, p )
        {
            /*
            setSide (s);
            setUp (u);
            setForward (f);
            setPosition (p);
             */
        }
        virtual ~PlaneObstacle() {};

        // find first intersection of a vehicle's path with this obstacle
        void findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                              AbstractObstacle::PathIntersection& pi)
            const;

        // determines if a given point on XY plane is inside obstacle shape
        virtual bool xyPointInsideShape (const Vec3& /*point*/,
                                         float /*radius*/) const
        {
            return true; // always true for PlaneObstacle
        }
    };


    // ----------------------------------------------------------------------------
    // RectangleObstacle: a rectangular obstacle of a given height, width,
    // position and orientation.  It is a rectangle centered on the XY (aka
    // side/up) plane of a local space.


    class RectangleObstacle : public PlaneObstacle
    {
    public:
        float width;  // width  of rectangle centered on local X (side) axis
        float height; // height of rectangle centered on local Y (up)   axis

        // constructors
        RectangleObstacle (float w, float h) : width(w), height(h) {}
        RectangleObstacle (void) :  width(1.0f), height(1.0f) {}
        RectangleObstacle (float w, float h, const Vec3& s,
                           const Vec3& u, const Vec3& f, const Vec3& p,
                           AbstractObstacle::seenFromState sf) 
            : PlaneObstacle( s, u, f, p ), width(w), height(h)
        {
            /*
            setSide (s);
            setUp (u);
            setForward (f);
            setPosition (p);
             */
            setSeenFrom (sf);
        }
        
        virtual ~RectangleObstacle() { /* Nothing to do. */ }

        // determines if a given point on XY plane is inside obstacle shape
        bool xyPointInsideShape (const Vec3& point, float radius) const;
    };


} // namespace OpenSteer

#endif
