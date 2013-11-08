%module(directors="1") opensteer
%feature("director");
%feature("autodoc", "1");
%include "std_vector.i"

#define NO_ANNOT 1

%{
#define NO_ANNOT 1
#include "OpenSteer/Utilities.h"
#include "OpenSteer/Vec3.h"
#include "OpenSteer/Vec3Utilities.h"
#include "OpenSteer/SharedPointer.h"
#include "OpenSteer/Proximity.h"
#include "OpenSteer/AbstractVehicle.h"

#include "OpenSteer/Pathway.h"
#include "OpenSteer/SegmentedPathway.h"
#include "OpenSteer/Path.h"
#include "OpenSteer/SegmentedPath.h"
#include "OpenSteer/QueryPathAlikeBaseDataExtractionPolicies.h"
#include "OpenSteer/QueryPathAlikeUtilities.h"
#include "OpenSteer/QueryPathAlikeMappings.h"
#include "OpenSteer/QueryPathAlike.h"

#include "OpenSteer/PolylineSegmentedPath.h"
#include "OpenSteer/PolylineSegmentedPathwaySegmentRadii.h"
#include "OpenSteer/PolylineSegmentedPathwaySingleRadius.h"
#include "OpenSteer/SegmentedPathAlikeUtilities.h"

#include "OpenSteer/Obstacle.h"
#include "OpenSteer/LocalSpaceObstacles.h"
#include "OpenSteer/SteerLibrary.h"
#include "OpenSteer/TrivialVehicle.h"
%}

%include "OpenSteer/Utilities.h"
%include "OpenSteer/Vec3.h"
%include "OpenSteer/Vec3Utilities.h"
 //%include "OpenSteer/SharedPointer.h"

%include "OpenSteer/Proximity.h"
%template (ProximityToken) OpenSteer::SimpleLQProximityToken<OpenSteer::AbstractVehicle*>;
%template (ProximityDatabase)    OpenSteer::SimpleLQProximityDatabase<OpenSteer::AbstractVehicle*>;

%include "OpenSteer/LocalSpace.h"
%include "OpenSteer/AbstractVehicle.h"
%include "OpenSteer/Pathway.h"
%include "OpenSteer/SegmentedPathway.h"
%include "OpenSteer/Path.h"
%include "OpenSteer/SegmentedPath.h"
%include "OpenSteer/QueryPathAlikeBaseDataExtractionPolicies.h"
%include "OpenSteer/QueryPathAlikeUtilities.h"
%include "OpenSteer/QueryPathAlikeMappings.h"
%include "OpenSteer/QueryPathAlike.h"
%include "OpenSteer/PolylineSegmentedPath.h"
%include "OpenSteer/PolylineSegmentedPathwaySegmentRadii.h"
%include "OpenSteer/PolylineSegmentedPathwaySingleRadius.h"
%include "OpenSteer/SegmentedPathAlikeUtilities.h"
%include "OpenSteer/Obstacle.h"
%template (LocalSpaceObstacle) OpenSteer::LocalSpaceMixin<OpenSteer::Obstacle>;
%template(ObstacleGroup) std::vector<OpenSteer::AbstractObstacle*>;

%include "OpenSteer/LocalSpaceObstacles.h"
%include "OpenSteer/SteerLibrary.h"

%template (_TrivialVehicleLS) OpenSteer::LocalSpaceMixin<OpenSteer::AbstractVehicle>;
%template (_TrivialVehicleLSSteer) OpenSteer::SteerLibraryMixin<OpenSteer::LocalSpaceMixin<OpenSteer::AbstractVehicle> >;
%include "OpenSteer/TrivialVehicle.h"


