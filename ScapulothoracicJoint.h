#ifndef OPENSIM_SCAPULOTHORACIC_JOINT_H_
#define OPENSIM_SCAPULOTHORACIC_JOINT_H_
/* -------------------------------------------------------------------------- *
*                    OpenSim:  ScapulothoracicJoint.h                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2015 Stanford University and the Authors                *
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

// INCLUDE
#include <string>
// Header to define plugin (DLL) interface
#include "osimPluginDLL.h"
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a 4dof ScapulothoracicJoint.
 * Motion of the scapula is described by an ellipsoid surface fixed
 * to the thorax upon which the joint frame of scapula rides. The motion on
 * the surface is governed by 2 dofs: abduction-adduction and elevation-
 * depression, which are comparable to longitude and latitude angles used to
 * locate a point (scapula joint frame origin) on the surface of a globe 
 * (thoracic surface). These coordinates are followed by upward rotation (the
 *  3rd dof) of the scapula about its joint Z-axis normal to the ellipsoid
 * surface. The 4th dof is a rotation about a winging" axis defined by a point
 * and axis direction in the scapula XY-plane (tangent to ellipsoid surface).
 *
 * @author Ajay Seth
 */
class OSIMPLUGIN_API ScapulothoracicJoint : public Joint  {
OpenSim_DECLARE_CONCRETE_OBJECT(ScapulothoracicJoint, Joint);

    static const int _numMobilities = 4;
//=============================================================================
// DATA
//=============================================================================
protected:

    /** Ellipsoid radii mobilizer frame as a Vec3(rX, rY, rZ) */
    PropertyDblVec3 _ellipsoidRadiiProp;
    SimTK::Vec3 &_ellipsoidRadii;

    /** Winging axis origin in the scapula plane tangent to the thoracic surface. */
    PropertyDblVec2 _wingOriginProp;
    SimTK::Vec2 &_wingOrigin;

    /** Winging axis orientation (in radians) of the scapula plane tangent to the thoracic surface.*/
    PropertyDbl _wingDirectionProp;
    double &_wingDirection;


//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    ScapulothoracicJoint();
    ScapulothoracicJoint(const ScapulothoracicJoint &aJoint);

    // convenience constructor
    ScapulothoracicJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
                OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
                SimTK::Vec3 ellipsoidRadii, SimTK::Vec2 wingingOrigin, double wingingDirection, bool reverse=false);

    virtual ~ScapulothoracicJoint();
    virtual Object* copy() const;
    ScapulothoracicJoint& operator=(const ScapulothoracicJoint &aJoint);
    void copyData(const ScapulothoracicJoint &aJoint);

    // SCALE
    virtual void scale(const ScaleSet& aScaleSet);

    // Model building
    virtual int numCoordinates() const {return _numMobilities; };

    // Accessors
    void setEllipsoidRadii(SimTK::Vec3 radii) { _ellipsoidRadii = radii; }
    const SimTK::Vec3 &getEllipsoidRadii() {return _ellipsoidRadii; }

    void setWingingOrigin(SimTK::Vec2 origin) { _wingOrigin = origin; }
    const SimTK::Vec2 &getWingingOrigin() {return _wingOrigin; } 

    void setWingingDirection(double direction) { _wingDirection = direction; }
    const double &getWingingDirection() {return _wingDirection; } 


protected:

    void connectToModel(Model& model);
    void addToSystem(SimTK::MultibodySystem& system) const;
    virtual void initStateFromProperties(SimTK::State& s) const;
    virtual void setPropertiesFromState(const SimTK::State& state);

private:
    void setNull();
    void setupProperties();
    void calcTransforms();


//=============================================================================
};	// END of class ScapulothoracicJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SCAPULOTHORACIC_JOINT_H_


