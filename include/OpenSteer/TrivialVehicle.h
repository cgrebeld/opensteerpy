// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2005, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// TrivalVehicle
// Like SimpleVehicle, but without Annotations
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_TRIVIALVEHICLE_H
#define OPENSTEER_TRIVIALVEHICLE_H

#define NO_ANNOT  1 // Don't add annotation stuff to SteerLibrary

#include "OpenSteer/AbstractVehicle.h"
#include "OpenSteer/SteerLibrary.h"

namespace OpenSteer {

    // TrivialVehicleLS adds concrete LocalSpace methods to AbstractVehicle
    typedef LocalSpaceMixin<OpenSteer::AbstractVehicle> TrivialVehicleLS;

    // TrivialVehicleLSSteer adds concrete steering methods to TrivialVehicleLS
    typedef SteerLibraryMixin<TrivialVehicleLS> TrivialVehicleLSSteer;

    // TrivialVehicle adds concrete vehicle methods to TrivialVehicleLSSteer
    class TrivialVehicle : public OpenSteer::TrivialVehicleLSSteer
    {
    public:

        typedef TrivialVehicleLSSteer Super;
        // constructor
        TrivialVehicle ();

        // destructor
        virtual ~TrivialVehicle ();

        // reset vehicle state
        void reset (void)
        {
            // reset LocalSpace state
            resetLocalSpace ();

            // reset SteerLibraryMixin state
            // (XXX this seems really fragile, needs to be redesigned XXX)
            Super::reset ();

            setMass (1);          // mass (defaults to 1 so acceleration=force)
            setSpeed (0);         // speed along Forward direction.

            setRadius (0.5f);     // size of bounding sphere

            setMaxForce (0.1f);   // steering force is clipped to this magnitude
            setMaxSpeed (1.0f);   // velocity is clipped to this magnitude

            // reset bookkeeping to do running averages of these quanities
            resetSmoothedPosition ();
            resetSmoothedCurvature ();
            resetSmoothedAcceleration ();
        }

        // get/set mass
        float mass (void) const {return _mass;}
        float setMass (float m) {return _mass = m;}

        // get velocity of vehicle
        Vec3 velocity (void) const {return forward() * _speed;}

        // get/set speed of vehicle  (may be faster than taking mag of velocity)
        float speed (void) const {return _speed;}
        float setSpeed (float s) {return _speed = s;}

        // size of bounding sphere, for obstacle avoidance, etc.
        float radius (void) const {return _radius;}
        float setRadius (float m) {return _radius = m;}

        // get/set maxForce
        float maxForce (void) const {return _maxForce;}
        float setMaxForce (float mf) {return _maxForce = mf;}

        // get/set maxSpeed
        float maxSpeed (void) const {return _maxSpeed;}
        float setMaxSpeed (float ms) {return _maxSpeed = ms;}

        // ratio of speed to max possible speed (0 slowest, 1 fastest)
        float relativeSpeed (void) const {return speed () / maxSpeed ();}


        // apply a given steering force to our momentum,
        // adjusting our orientation to maintain velocity-alignment.
        void applySteeringForce (const OpenSteer::Vec3& force, const float deltaTime);

        // the default version: keep FORWARD parallel to velocity, change
        // UP as little as possible.
        virtual void regenerateLocalSpace (const OpenSteer::Vec3& newVelocity,
                                           const float elapsedTime);

        // alternate version: keep FORWARD parallel to velocity, adjust UP
        // according to a no-basis-in-reality "banking" behavior, something
        // like what birds and airplanes do.  (XXX experimental cwr 6-5-03)
        void regenerateLocalSpaceForBanking (const OpenSteer::Vec3& newVelocity,
                                             const float elapsedTime);

        // adjust the steering force passed to applySteeringForce.
        // allows a specific vehicle class to redefine this adjustment.
        // default is to disallow backward-facing steering at low speed.
        // xxx experimental 8-20-02
        virtual OpenSteer::Vec3 adjustRawSteeringForce (const OpenSteer::Vec3& force,
                                             const float deltaTime);

        // apply a given braking force (for a given dt) to our momentum.
        // xxx experimental 9-6-02
        void applyBrakingForce (const float rate, const float deltaTime);

        // predict position of this vehicle at some time in the future
        // (assumes velocity remains constant)
        OpenSteer::Vec3 predictFuturePosition (const float predictionTime) const;

        // get instantaneous curvature (since last update)
        float curvature (void) const {return _curvature;}

        // get/reset smoothedCurvature, smoothedAcceleration and smoothedPosition
        float smoothedCurvature (void) {return _smoothedCurvature;}
        float resetSmoothedCurvature (float value = 0)
        {
            _lastForward = OpenSteer::Vec3::zero;
            _lastPosition = OpenSteer::Vec3::zero;
            return _smoothedCurvature = _curvature = value;
        }
        OpenSteer::Vec3 smoothedAcceleration (void) {return _smoothedAcceleration;}
        OpenSteer::Vec3 resetSmoothedAcceleration (const OpenSteer::Vec3& value = OpenSteer::Vec3::zero)
        {
            return _smoothedAcceleration = value;
        }
        OpenSteer::Vec3 smoothedPosition (void) {return _smoothedPosition;}
        OpenSteer::Vec3 resetSmoothedPosition (const OpenSteer::Vec3& value = OpenSteer::Vec3::zero)
        {
            return _smoothedPosition = value;
        }
/*
        // give each vehicle a unique number
        int serialNumber;
        static int serialNumberCounter;

        // draw lines from vehicle's position showing its velocity and acceleration
        void annotationVelocityAcceleration (float maxLengthA, float maxLengthV);
        void annotationVelocityAcceleration (float maxLength)
            {annotationVelocityAcceleration (maxLength, maxLength);}
        void annotationVelocityAcceleration (void)
            {annotationVelocityAcceleration (3, 3);}
*/

        // set a random "2D" heading: set local Up to global Y, then effectively
        // rotate about it by a random angle (pick random forward, derive side).
        void randomizeHeadingOnXZPlane (void)
        {
            setUp (OpenSteer::Vec3::up);
            setForward (RandomUnitVectorOnXZPlane ());
            setSide (localRotateForwardToSide (forward()));
        }

        virtual void update(const float currentTime, const float elapsedTime) {};

    private:

        float _mass;       // mass (defaults to unity so acceleration=force)

        float _radius;     // size of bounding sphere, for obstacle avoidance, etc.

        float _speed;      // speed along Forward direction.  Because local space
                           // is velocity-aligned, velocity = Forward * Speed

        float _maxForce;   // the maximum steering force this vehicle can apply
                           // (steering force is clipped to this magnitude)

        float _maxSpeed;   // the maximum speed this vehicle is allowed to move
                           // (velocity is clipped to this magnitude)

        float _curvature;
        OpenSteer::Vec3 _lastForward;
        OpenSteer::Vec3 _lastPosition;
        OpenSteer::Vec3 _smoothedPosition;
        float _smoothedCurvature;
        OpenSteer::Vec3 _smoothedAcceleration;

        // measure path curvature (1/turning-radius), maintain smoothed version
        void measurePathCurvature (const float elapsedTime);
    };


} // namespace OpenSteer
    
    
// ----------------------------------------------------------------------------
#endif // OPENSTEER_TRIVIALVEHICLE_H
