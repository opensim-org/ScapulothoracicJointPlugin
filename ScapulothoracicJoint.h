#ifndef OPENSIM_SCAPULOTHORACIC_JOINT_H_ 
#define OPENSIM_SCAPULOTHORACIC_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ScapulothoracicJoint.h                       *
 * -------------------------------------------------------------------------- *
 * ScapulothoracicJoint is offered as an addition to the OpenSim API          *
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
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a 4dof ScapulothoracicJoint.
 * Motion of the scapula is described by an ellipsoid surface fixed
 * to the thorax upon which the joint frame of scapul rides. The motion on
 * the surface is governed by 2 dofs: up-down and medio-lateral described
 * by latitude and longitudinal angles. Scapula rotation about the normal to
 * the ellipsoid surface is the 3rd dof. The 4th dof is a rotation about a 
 * "winging" axis defined by a point and axis direction in the scapula frame.
 *
 * @author Ajay Seth
 * @version 1
 */
class OSIMPLUGIN_API ScapulothoracicJoint : public Joint  {
OpenSim_DECLARE_CONCRETE_OBJECT(ScapulothoracicJoint, Joint);

    static const int _numMobilities = 4;

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations 
    The serializable properties associated with a ScapulothoracicJoint.
    /**@{**/
    OpenSim_DECLARE_PROPERTY(thoracic_ellipsoid_radii_x_y_z, SimTK::Vec3, 
        "Radii of the thoracic surface ellipsoid a Vec3(rX, rY, rZ).");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(scapula_winging_axis_origin, double, 2,
        "Winging axis origin (x,y coordinates) in the scapula plane "
        "(tangent to the thoracic surface).");

    OpenSim_DECLARE_PROPERTY(scapula_winging_axis_direction, double, 
        "Winging axis orientation (in radians) in the scapula plane.");
    /**@}**/


//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /** Default contructor */
    ScapulothoracicJoint();

    // default destructor, copy constructor, copy assignment

    /** Convenience constructor */
    ScapulothoracicJoint(const std::string &name, 
        OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
        OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
        SimTK::Vec3 ellipsoidRadii, SimTK::Vec2 wingingOrigin, double wingingDirection,
        bool reverse=false);


    // SCALE
    void scale(const ScaleSet& aScaleSet) OVERRIDE_11;

    // Model building
    int numCoordinates() const OVERRIDE_11 {return _numMobilities; };


protected:
    /** ModelComponent Interface */
    void connectToModel(Model& model) OVERRIDE_11;
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;

private:
    void constructProperties();

//=============================================================================
};	// END of class ScapulothoracicJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SCAPULOTHORACIC_JOINT_H_


