#ifndef _StaticOptimization3_h_
#define _StaticOptimization3_h_
/* -------------------------------------------------------------------------- *
*                       OpenSim:  StaticOptimization3.h                       *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Jeffrey A. Reinbolt with additions for matrix weights           *
*            from Kat M. Steele                                             *
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
#include <memory>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Analyses/ForceReporter.h>
#include "ActWeightingMatrix.h"									// KAT
#include <OpenSim/Common/PropertyObj.h>							// NICK

//=============================================================================
//=============================================================================
/**
*/
namespace OpenSim {

	class Model;
	class ForceSet;

	/**
	* This class implements static optimization to compute Muscle Forces and
	* activations.
	*
	* @author Jeff Reinbolt
	*/
	class OSIMPLUGIN_API StaticOptimization3 : public Analysis {
		OpenSim_DECLARE_CONCRETE_OBJECT(StaticOptimization3, Analysis);

		//=============================================================================
		// DATA
		//=============================================================================
	private:
		int _numCoordinateActuators;

		std::unique_ptr<ForceReporter> _forceReporter;

	protected:
		/** Use force set from model. */
		PropertyBool _useModelForceSetProp;
		bool &_useModelForceSet;

		PropertyDbl _activationExponentProp;
		double &_activationExponent;

		PropertyBool _useMusclePhysiologyProp;
		bool    &_useMusclePhysiology;

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

		Model *_modelWorkingCopy;

		//=============================================================================
		// METHODS
		//=============================================================================
	public:
		StaticOptimization3(Model *aModel = 0);
		StaticOptimization3(const StaticOptimization3 &aObject);
		virtual ~StaticOptimization3();

		//--------------------------------------------------------------------------
		// OPERATORS
		//--------------------------------------------------------------------------
#ifndef SWIG
		StaticOptimization3& operator=(const StaticOptimization3 &aStaticOptimization3);
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
		void constructColumnLabels();
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

		void setModel(Model& aModel) override;
		void setActivationExponent(const double aExponent) { _activationExponent = aExponent; }
		double getActivationExponent() const { return _activationExponent; }
		void setUseMusclePhysiology(const bool useIt) { _useMusclePhysiology = useIt; }
		bool getUseMusclePhysiology() const { return _useMusclePhysiology; }
		void setifAWM(int ns) { _ifAWM = ns; }											//KAT
		//int getNSynergies() const {return _nSynergies; }												//KAT
		void setConvergenceCriterion(const double tolerance) { _convergenceCriterion = tolerance; }
		double getConvergenceCriterion() { return _convergenceCriterion; }
		void setMaxIterations(const int maxIt) { _maximumIterations = maxIt; }
		int getMaxIterations() { return _maximumIterations; }
		//--------------------------------------------------------------------------
		// ANALYSIS
		//--------------------------------------------------------------------------
		int
			begin(const SimTK::State& s) override;
		int
			step(const SimTK::State& s, int setNumber) override;
		virtual int														//KAT
			recordActWeightingMatrix(ForceSet *const forceSet);		//KAT
		int
			end(const SimTK::State& s) override;
	protected:
		virtual int
			record(const SimTK::State& s);
		//--------------------------------------------------------------------------
		// IO
		//--------------------------------------------------------------------------
	public:
		int
			printResults(const std::string &aBaseName, const std::string &aDir = "",
				double aDT = -1.0, const std::string &aExtension = ".sto") override;

		//=============================================================================
	};  // END of class StaticOptimization3

}; //namespace
   //=============================================================================
   //=============================================================================


#endif // #ifndef __StaticOptimization3_h__