/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ScapulothoracicJoint.cpp                      *
 * -------------------------------------------------------------------------- *
 * ScapulothoracicJoint is offered as an addition to the OpenSim API          *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "ScapulothoracicJoint.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) 
//=============================================================================
//_____________________________________________________________________________
/*
* Default constructor.
*/
ScapulothoracicJoint::ScapulothoracicJoint() : Joint()
{
    constructCoordinates();
    constructProperties();
}


//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
ScapulothoracicJoint::ScapulothoracicJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
                OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
                SimTK::Vec3 ellipsoidRadii, SimTK::Vec2 wingingOrigin, double wingingDirection, bool reverse) :
    Joint(name, parent, locationInParent,orientationInParent, body, locationInBody, orientationInBody, reverse)
{
    constructCoordinates();
    constructProperties();

    upd_thoracic_ellipsoid_radii_x_y_z() = ellipsoidRadii;
    upd_scapula_winging_axis_origin(0) = wingingOrigin[0];
    upd_scapula_winging_axis_origin(1) = wingingOrigin[1];
    upd_scapula_winging_axis_direction() = wingingDirection;
    updBody().setJoint(*this);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct properties with their default values.
 */
void ScapulothoracicJoint::constructProperties()
{
    setAuthors("Ajay Seth");
    Vec3 radii(NaN);
    Vector origin(2, 0.0);
    double dir = 0;

    origin.size();
    constructProperty_thoracic_ellipsoid_radii_x_y_z(radii);
    constructProperty_scapula_winging_axis_origin(origin);
    constructProperty_scapula_winging_axis_direction(dir);
}



//_____________________________________________________________________________
/**
* Perform some set up functions that happen after the
* object has been deserialized or copied.
*
* @param model containing this ScapulothoracicJoint.
*/
void ScapulothoracicJoint::connectToModel(Model& model)
{
    // COORDINATE SET
    CoordinateSet& coords = upd_CoordinateSet();

    // Some initializations
    int nq = coords.getSize();  

    // Note- should check that all coordinates are used.
    if( nq > _numMobilities){
        cout << "ScapulothoracicJoint::Too many coordinates specified."<< endl;
        for(int i = nq-1; i >= _numMobilities; --i){
            cout << "ScapulothoracicJoint:: removing coordinate: "<< coords[i].getName() << endl;
            coords.remove(i);
        }
    }

    Array<string> axisNames;
    axisNames.append("abduction");
    axisNames.append("elevation");
    axisNames.append("upward-rot");
    axisNames.append("internal-rot");

    while(nq < _numMobilities){
        Coordinate *coord = new Coordinate;
        string name = getName() + "_" + axisNames[nq];
        coord->setName(name);
        coord->setMotionType(Coordinate::Rotational);
        coord->setRangeMin(-SimTK::Pi/2);
        coord->setRangeMax(SimTK::Pi/2);

        cout << "ScapulothoracicJoint::Insufficient number of coordinates, adding coordinate: " << name << "."<< endl;
        coords.adoptAndAppend(coord);
        nq++;
    }

    // Base class checks that parent is valid
    Super::connectToModel(model);
}




//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
* Scale a joint based on XYZ scale factors for the bodies.
*
* @param aScaleSet Set of XYZ scale factors for the bodies.
* @todo Need to scale transforms appropriately, given an arbitrary axis.
*/
void ScapulothoracicJoint::scale(const ScaleSet& aScaleSet)
{
    Vec3 scaleFactors(1.0);

    // Joint knows how to scale locations of the joint in parent and on the body
    Joint::scale(aScaleSet);

    // SCALING TO DO WITH THE PARENT BODY -----
    // Joint kinematics are scaled by the scale factors for the
    // parent body, so get those body's factors
    const string& parentName = getParentBody().getName();
    for (int i=0; i<aScaleSet.getSize(); i++) {
        const Scale &scale = aScaleSet.get(i);
        if (scale.getSegmentName()==parentName) {
            scale.getScaleFactors(scaleFactors);
            break;
        }
    }

    for(int i=0; i<3; i++){ 
        // Scale the size of the mobilizer
        upd_thoracic_ellipsoid_radii_x_y_z()[i] *= scaleFactors[i];
    }
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void ScapulothoracicJoint::addToSystem(SimTK::MultibodySystem& system) const
{
    const SimTK::Vec3& orientation = get_orientation();
    const SimTK::Vec3& location = get_location();
    const SimTK::Vec3& orientationInParent = get_orientation_in_parent();
    const SimTK::Vec3& locationInParent = get_location_in_parent();
    
    // Transform for ellipsoid joint frame in intermediate Scapula massless body frame
    // such that intermediate frame is aligned with scapula joint frame with respect
    // to the scapula body frame as  user specified by _location and _orientation 
    //Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, 
    //	_orientation[2]+SimTK::Pi/2, ZAxis);
    //SimTK::Transform ellipsoidJointFrameInIntermediate(rotation, _location);
    Rotation rotation(BodyRotationSequence, 0,XAxis, 0,YAxis, Pi/2, ZAxis);
    SimTK::Transform ellipsoidJointFrameInIntermediate(rotation, Vec3(0.0));

    // Transform for Ellipsoid in parent body (Thorax)
    // Note: Ellipsoid rotated Pi/2 w.r.t. parent (i.e. Thorax) so that abduction and elevation are positive
    Rotation parentRotation(BodyRotationSequence, orientationInParent[0], XAxis, 
        orientationInParent[1], YAxis, orientationInParent[2]+SimTK::Pi/2, ZAxis);
    SimTK::Transform ellipsoidJointFrameInThorax(parentRotation, locationInParent);

    // Workaround for new API with const functions
    ScapulothoracicJoint* mutableThis = const_cast<ScapulothoracicJoint*>(this);
    
    // CREATE MOBILIZED BODY
    // Ellipsoid is rotated Pi/2 for desired rotations, but user's still wants to define Ellipsoid shape w.r.t thorax
    // Swap ellipsoidRadii X,Y,Z in Thorax body frame to Y, X, Z in rotated joint frame in parent
    Vec3 ellipsoidRadii(get_thoracic_ellipsoid_radii_x_y_z()[1], 
        get_thoracic_ellipsoid_radii_x_y_z()[0], 
        get_thoracic_ellipsoid_radii_x_y_z()[2]);
    MobilizedBody::Ellipsoid
        simtkMasslessBody(mutableThis->updModel().updMatterSubsystem().updMobilizedBody(getParentBody().getIndex()),
        ellipsoidJointFrameInThorax, SimTK::Body::Massless(), 
        ellipsoidJointFrameInIntermediate, 
        ellipsoidRadii);

    // get unit vector version of direction in the scapula joint frame of the Ellipsoid 
    // where the joint Z-axis is normal to the ellipsoid surface, X-axis is in the 
    // direction of abduction and Y is elevation in the neutral position
    // winging is orthogonal to upward rotation (about Z) with axis in XY-plane
    // winging direction for 0 is aligned with intermediate frame Y and rotates
    // counterclockwise with increasing angles.
    const double& wingDirection = get_scapula_winging_axis_direction();
    SimTK::UnitVec3 dir(-sin(wingDirection), cos(wingDirection), 0);

    // Find rotation that aligns z-axis of pin mobilizer frame to winging axis
    // This is in the scapula-ellipsoid (massless body) frame
    SimTK::Rotation wingOrientationInIntermediateFrame(dir, ZAxis);

    // origin of the winging axis w.r.t to the scapula joint frame of the ellipsoid
    SimTK::Vec3 wingOriginInIntermediateFrame(
        get_scapula_winging_axis_origin(0), get_scapula_winging_axis_origin(1), 0);
    // winging joint transform in the scapula ellipsoid joint frame
    SimTK::Transform wingingInIntermediateFrame(
        wingOrientationInIntermediateFrame, 
        wingOriginInIntermediateFrame);

    MobilizedBody::Pin simtkBody(simtkMasslessBody, 
        wingingInIntermediateFrame, SimTK::Body::Massless(), 
        wingingInIntermediateFrame);

    // Define the scapular joint frame in w.r.t to the Scapula body frame
    Rotation rotation2(BodyRotationSequence, orientation[0], XAxis, 
        orientation[1], YAxis, orientation[2], ZAxis);
    SimTK::Transform jointInScapula(rotation2, location);
    MobilizedBody::Weld simtkBody2(simtkBody, Transform(), 
        SimTK::Body::Rigid(mutableThis->updBody().getMassProperties()), 
        jointInScapula);

    setMobilizedBodyIndex(&mutableThis->updBody(), 
        simtkBody2.getMobilizedBodyIndex());

    // Each coordinate needs to know it's body index and mobility index.
    const CoordinateSet& coordinateSet = get_CoordinateSet();

    // Link the coordinates for this Joint to the underlying mobilities
    for(int i =0; i < _numMobilities; ++i){
        Coordinate &coord = coordinateSet[i];
        // Translations enabled by Translation mobilizer, first, but appear second in coordinate set
        setCoordinateMobilizedBodyIndex(&coord, ((i < 3) ?
            (simtkMasslessBody.getMobilizedBodyIndex()) : 
                (simtkBody.getMobilizedBodyIndex())));
        // The mobility index is the same as the order in which the coordinate appears in the coordinate set.
        setCoordinateMobilizerQIndex(&coord, (i < 3 ? i : i-3));
    }
}

void ScapulothoracicJoint::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void ScapulothoracicJoint::setPropertiesFromState(const SimTK::State& state)
{
    Super::setPropertiesFromState(state);
}