// ActWeightingVector.cpp
// Author:  Kat Steele 3/2014 for Static Optimization with Actuator Weight Matrix
/*
 * Copyright (c)  2014, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "ActWeightingVector.h"

//=============================================================================
// NAMESPACES
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ActWeightingVector::ActWeightingVector() :
	//_apply(_applyProp.getValueBool()),
   _awv(_awvProp.getValueDblArray())
{
	//_apply = true;
	_awv = (0);
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
ActWeightingVector::ActWeightingVector(const ActWeightingVector &aActWeightingVector) :
   Object(aActWeightingVector),
	//_apply(_applyProp.getValueBool()),
   _awv(_awvProp.getValueDblArray())
{
	//_apply = aActuatorWeightTask._apply;
	_awv = aActWeightingVector._awv;
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ActWeightingVector::setupProperties()
{
	//_applyProp.setComment("Whether or not this task will be used during static optimization."); 
	//_applyProp.setName("apply");
	//_propertySet.append(&_applyProp);

	_awvProp.setComment("Array describing one actuator weighting vector. Length should equal number of actuators in model."); 
	_awvProp.setName("array");
	_propertySet.append(&_awvProp);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ActWeightingVector& ActWeightingVector::operator=(const ActWeightingVector &aActWeightingVector)
{
	Object::operator=(aActWeightingVector);
	//_apply = aActuatorWeightTask._apply;
	_awv = aActWeightingVector._awv;
	return *this;
}