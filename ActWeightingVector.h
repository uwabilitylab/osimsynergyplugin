#ifndef __ActWeightingVector_h__
#define __ActWeightingVector_h__
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

#include "osimPluginDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDblArray.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @author Kat Steele
 * @version 1.0
 * Modified from IKTask to add actuator weights to static optimization. Kat 3/25/14
 */

class OSIMPLUGIN_API ActWeightingVector : public Object
{
OpenSim_DECLARE_CONCRETE_OBJECT(ActWeightingVector, Object);
protected:
	// whether or not this task will be used
	//PropertyBool _applyProp;
	//bool &_apply;

	PropertyDblArray _awvProp;
	Array<double> &_awv;

public:
	ActWeightingVector();
	ActWeightingVector(const ActWeightingVector &aActWeightingVector);
	// virtual Object* copy() const {return new ActWeightingVector(*this);}

#ifndef SWIG
	ActWeightingVector& operator=(const ActWeightingVector &aActWeightingVector);
#endif

	//bool getApply() const { return _apply; }
	//void setApply(bool aApply) { _apply = aApply; }

	Array<double> getActWeightingVector() { return _awv; }
	void setActWeightingVector(Array<double> ActWeightingVector) { _awv = ActWeightingVector; }

private:
	void setupProperties();
//=============================================================================
};	// END of class ActWeightingVector
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ActWeightingVector_h__
