#ifndef _StaticOptimization2_h_
#define _StaticOptimization2_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StaticOptimization2.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt with additions for matrix weights			  *
 *            from Kat M. Steele											  *
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
#include "osimPluginDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <SimTKcommon.h>
#include "ActWeightingMatrix.h"									// KAT


//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class ForceSet;
class Actuator;

/**
 * This class implements static optimization to compute Muscle Forces and 
 * activations. 
 *
 * @author Jeff Reinbolt
 */
class OSIMPLUGIN_API StaticOptimization2 : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(StaticOptimization2, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:
	int _numCoordinateActuators;
protected:
	/** Use force set from model. */
	PropertyBool _useModelForceSetProp;
	bool &_useModelForceSet;

	PropertyDbl _activationExponentProp;
	double &_activationExponent;

	PropertyBool _useMusclePhysiologyProp;
	bool	&_useMusclePhysiology;

	//PropertyInt _nSynergiesProp;					// KAT
	//int &_nSynergies;								// KAT

	SimTK::Matrix _actWeightingMatrix;				// KAT
	int _ifAWM;	

	OpenSim::Array<std::string> _activ_labels;			// KAT
	OpenSim::Array<std::string> _force_labels;

	PropertyObj _awmProp;							// KAT
	ActWeightingMatrix &_awm;						// KAT

	PropertyDbl _convergenceCriterionProp;
	double &_convergenceCriterion;

	PropertyInt _maximumIterationsProp;
	int &_maximumIterations;

	Storage *_activationStorage;
	Storage *_forceStorage;
	GCVSplineSet _statesSplineSet;

	Array<int> _accelerationIndices;

	SimTK::Vector _parameters;

	bool _ownsForceSet;
	ForceSet* _forceSet;

	double _numericalDerivativeStepSize;
	std::string _optimizerAlgorithm;
	int _printLevel;
	//double _optimizationConvergenceTolerance;
	//int _maxIterations;

	Model *_modelWorkingCopy;

//=============================================================================
// METHODS
//=============================================================================
public:
	StaticOptimization2(Model *aModel=0);
	StaticOptimization2(const StaticOptimization2 &aObject);
	virtual ~StaticOptimization2();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	StaticOptimization2& operator=(const StaticOptimization2 &aStaticOptimization2);
#endif
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructActivColumnLabels(); // KAT
	void constructForceColumnLabels(); // KAT
	void setActivColumnLabels(const OpenSim::Array<std::string> &aLabels); //KAT
	const OpenSim::Array<std::string> getActivColumnLabels() const; //KAT
	void setForceColumnLabels(const OpenSim::Array<std::string> &aLabels); //KAT
	const OpenSim::Array<std::string> getForceColumnLabels() const; //KAT
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getActivationStorage();
	Storage* getForceStorage();

	bool getUseModelForceSet() { return _useModelForceSet; }
	void setUseModelForceSet(bool aUseModelActuatorSet) { _useModelForceSet = aUseModelActuatorSet; }

	virtual void setModel(Model& aModel);
	void setActivationExponent(const double aExponent) { _activationExponent=aExponent; }
	double getActivationExponent() const { return _activationExponent; }
	void setUseMusclePhysiology(const bool useIt) { _useMusclePhysiology=useIt; }
	bool getUseMusclePhysiology() const { return _useMusclePhysiology; }
	void setifAWM(int ns) { _ifAWM=ns; }											//KAT
	//int getNSynergies() const {return _nSynergies; }												//KAT
	void setConvergenceCriterion(const double tolerance) { _convergenceCriterion = tolerance; }
	double getConvergenceCriterion() { return _convergenceCriterion; }
	void setMaxIterations( const int maxIt) { _maximumIterations = maxIt; }
	int getMaxIterations() {return _maximumIterations; }

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
        begin(SimTK::State& s );
    virtual int
        step(const SimTK::State& s, int setNumber );
	virtual int														//KAT
		recordActWeightingMatrix(const Set<Actuator>& model_actuators);		//KAT
    virtual int
        end(SimTK::State& s );
protected:
    virtual int
        record(const SimTK::State& s );
	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class StaticOptimization2

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __StaticOptimization2_h__
