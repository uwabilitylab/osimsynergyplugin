/* -------------------------------------------------------------------------- *
 *                      OpenSim:  StaticOptimization2.cpp                      *
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
#include <iostream>
#include <string>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <SimTKmath.h>
#include <SimTKlapack.h>
#include "StaticOptimization2.h"									//KAT
#include "StaticOptimizationTarget2.h"								//KAT
#include "ActWeightingMatrix.h"										//KAT
#include "ActWeightingVector.h"										//KAT
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
StaticOptimization2::~StaticOptimization2()
{
	deleteStorage();
	delete _modelWorkingCopy;
	if(_ownsForceSet) delete _forceSet;
}
//_____________________________________________________________________________
/**
 */
StaticOptimization2::StaticOptimization2(Model *aModel) :
	Analysis(aModel),
	_useModelForceSet(_useModelForceSetProp.getValueBool()),
	_activationExponent(_activationExponentProp.getValueDbl()),
	_useMusclePhysiology(_useMusclePhysiologyProp.getValueBool()),
	//_nSynergies(_nSynergiesProp.getValueInt()),								//KAT
	_awmProp(PropertyObj("", ActWeightingMatrix())),							//KAT
	_awm((ActWeightingMatrix&)_awmProp.getValueObj()),				//KAT
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maximumIterations(_maximumIterationsProp.getValueInt()),
	_modelWorkingCopy(NULL),
	_numCoordinateActuators(0)
{
	setNull();

	if(aModel) setModel(*aModel);
	else allocateStorage();
}
// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
StaticOptimization2::StaticOptimization2(const StaticOptimization2 &aStaticOptimization2):
	Analysis(aStaticOptimization2),
	_useModelForceSet(_useModelForceSetProp.getValueBool()),
	_activationExponent(_activationExponentProp.getValueDbl()),
	_useMusclePhysiology(_useMusclePhysiologyProp.getValueBool()),
	//_nSynergies(_nSynergiesProp.getValueInt()),										// KAT
	_awmProp(PropertyObj("", ActWeightingMatrix())),									// KAT
	_awm((ActWeightingMatrix&)_awmProp.getValueObj()),						// KAT
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maximumIterations(_maximumIterationsProp.getValueInt()),
	_modelWorkingCopy(NULL),
	_numCoordinateActuators(aStaticOptimization2._numCoordinateActuators)
{
	setNull();
	// COPY TYPE AND NAME
	*this = aStaticOptimization2;
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
StaticOptimization2& StaticOptimization2::
operator=(const StaticOptimization2 &aStaticOptimization2)
{
	// BASE CLASS
	Analysis::operator=(aStaticOptimization2);

	_modelWorkingCopy = aStaticOptimization2._modelWorkingCopy;
	_numCoordinateActuators = aStaticOptimization2._numCoordinateActuators;
	_useModelForceSet = aStaticOptimization2._useModelForceSet;
	_activationExponent=aStaticOptimization2._activationExponent;
	//_nSynergies=aStaticOptimization2._nSynergies;								// KAT
	_awm=aStaticOptimization2._awm;												// KAT
	_convergenceCriterion=aStaticOptimization2._convergenceCriterion;
	_maximumIterations=aStaticOptimization2._maximumIterations;
	_useMusclePhysiology=aStaticOptimization2._useMusclePhysiology;
	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void StaticOptimization2::setNull()
{
	setAuthors("Jeffrey A. Reinbolt & Kat M. Steele");
	setupProperties();

	// OTHER VARIABLES
	_useModelForceSet = true;
	_activationStorage = NULL;
	_forceStorage = NULL;
	_ownsForceSet = false;
	_forceSet = NULL;
	_activationExponent=2;
	//_nSynergies = 0;												// KAT
	_useMusclePhysiology=true;
	_numCoordinateActuators = 0;
	_convergenceCriterion = 1e-4;
	_maximumIterations = 100;

	setName("StaticOptimization2");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void StaticOptimization2::
setupProperties()
{
	_useModelForceSetProp.setComment("If true, the model's own force set will be used in the static optimization computation.  "
													"Otherwise, inverse dynamics for coordinate actuators will be computed for all unconstrained degrees of freedom.");
	_useModelForceSetProp.setName("use_model_force_set");
	_propertySet.append(&_useModelForceSetProp);

	_activationExponentProp.setComment(
		"A double indicating the exponent to raise activations to when solving static optimization.  ");
	_activationExponentProp.setName("activation_exponent");
	_propertySet.append(&_activationExponentProp);

	/*_nSynergiesProp.setComment(																				// KAT
		"An integer specifying the number of synergies included in synergy_set.");
	_nSynergiesProp.setName("num_synergies");
	_propertySet.append(&_nSynergiesProp);*/

	_awmProp.setComment("Set used to specify actuator weight matrix for synergy optimization."); 
	_awmProp.setName("ActWeightingMatrix"); 
	_propertySet.append(&_awmProp); 

	_useMusclePhysiologyProp.setComment(
		"If true muscle force-length curve is observed while running optimization.");
	_useMusclePhysiologyProp.setName("use_muscle_physiology");
	_propertySet.append(&_useMusclePhysiologyProp);

	_convergenceCriterionProp.setComment(
		"Value used to determine when the optimization solution has converged");
	_convergenceCriterionProp.setName("optimizer_convergence_criterion");
	_propertySet.append(&_convergenceCriterionProp);

	_maximumIterationsProp.setComment(
		"An integer for setting the maximum number of iterations the optimizer can use at each time.  ");
	_maximumIterationsProp.setName("optimizer_max_iterations");
	_propertySet.append(&_maximumIterationsProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the static optimization files.
 */
void StaticOptimization2::
constructDescription()
{
	string descrip = "This file contains static optimization results.\n\n";
	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the static optimization files.
 * Activation includes actuators and activation of actuator weighting groups (if applied)
 * Force includes actuators
 */
void StaticOptimization2::
constructActivColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	if(_model) {
		// Actuators
		for (int i=0; i < _forceSet->getSize(); i++) {
			Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(i));
			if(act) labels.append(_forceSet->get(i).getName());
		}
		// Actuator Weighting Matrix (if provided)
		for (int i=0; i< _awm.getSize(); i++) {
			ActWeightingVector *awm_element = dynamic_cast<ActWeightingVector*>(&_awm.get(i));
			labels.append(awm_element->getName());
		}
	}
	setActivColumnLabels(labels);
}
void StaticOptimization2::
constructForceColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	// Actuators
	if(_model) {
		for (int i=0; i < _forceSet->getSize(); i++) {
			Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(i));
			if(act) labels.append(_forceSet->get(i).getName());
		}
	}
	setForceColumnLabels(labels);
}
void StaticOptimization2::		// KAT: Since we are using different column labels need separate column labels from generic analysis
setActivColumnLabels(const OpenSim::Array<string> &aLabels)
{
	_activ_labels = aLabels;
}
void StaticOptimization2::
setForceColumnLabels(const OpenSim::Array<string> &aLabels)
{
	_force_labels = aLabels;
}
const OpenSim::Array<string> StaticOptimization2::
getActivColumnLabels() const
{
	return _activ_labels;
}
const OpenSim::Array<string> StaticOptimization2::
getForceColumnLabels() const
{
	return _force_labels;
}

//_____________________________________________________________________________
/**
 * Allocate storage for the static optimization.
 */
void StaticOptimization2::
allocateStorage()
{
	constructActivColumnLabels(); // KAT
	constructForceColumnLabels(); // KAT

	_activationStorage = new Storage(1000,"Static Optimization");
	_activationStorage->setDescription(getDescription());
	_activationStorage->setColumnLabels(getActivColumnLabels()); //KAT: Specific column labels

	_forceStorage = new Storage(1000,"Static Optimization");
	_forceStorage->setDescription(getDescription());
	_forceStorage->setColumnLabels(getForceColumnLabels());

}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void StaticOptimization2::
deleteStorage()
{
	delete _activationStorage; _activationStorage = NULL;
	delete _forceStorage; _forceStorage = NULL;
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the static optimization is to be computed.
 *
 * @param aModel Model pointer
 */
void StaticOptimization2::
setModel(Model& aModel)
{
	Analysis::setModel(aModel);
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the activation storage.
 *
 * @return Activation storage.
 */
Storage* StaticOptimization2::
getActivationStorage()
{
	return(_activationStorage);
}
//_____________________________________________________________________________
/**
 * Get the force storage.
 *
 * @return Force storage.
 */
Storage* StaticOptimization2::
getForceStorage()
{
	return(_forceStorage);
}

//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capacities run out.
 */
void StaticOptimization2::
setStorageCapacityIncrements(int aIncrement)
{
	_activationStorage->setCapacityIncrement(aIncrement);
	_forceStorage->setCapacityIncrement(aIncrement);
}

//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the results.
 */
int StaticOptimization2::
record(const SimTK::State& s)
{
	if(!_modelWorkingCopy) return -1;

	// Set model to whatever defaults have been updated to from the last iteration
    SimTK::State& sWorkingCopy = _modelWorkingCopy->updWorkingState();
	sWorkingCopy.setTime(s.getTime());
	_modelWorkingCopy->initStateWithoutRecreatingSystem(sWorkingCopy); 

	// update Q's and U's
	sWorkingCopy.setQ(s.getQ());
	sWorkingCopy.setU(s.getU());

	_modelWorkingCopy->getMultibodySystem().realize(sWorkingCopy, SimTK::Stage::Velocity);
	//_modelWorkingCopy->equilibrateMuscles(sWorkingCopy);

    const Set<Actuator>& fs = _modelWorkingCopy->getActuators();

	int na = fs.getSize();							// Number of actuators
	int size_awm = _awm.getSize();				// Size of actuator weighting matrix		// KAT
	int nacc = _accelerationIndices.getSize();		// Number of coordinates

	// Set number of parameters for optimization												// KAT
	int np = _actWeightingMatrix.ncol();

	// Get number of muscle 
	int nm = _modelWorkingCopy->getMuscles().getSize();

	// IPOPT
	_numericalDerivativeStepSize = 0.0001;
	_optimizerAlgorithm = "ipopt";
	_printLevel = 0;
	//_optimizationConvergenceTolerance = 1e-004;
	//_maxIterations = 2000;

	// Optimization target
	_modelWorkingCopy->setAllControllersEnabled(false);
	StaticOptimizationTarget2 target(sWorkingCopy,_modelWorkingCopy,np,nacc,_useMusclePhysiology);  // KAT: For SynergyOptimization na = np (number of parameters)
	target.setStatesStore(_statesStore);															
	target.setStatesSplineSet(_statesSplineSet);
	target.setActivationExponent(_activationExponent);
	target.setDX(_numericalDerivativeStepSize);
	target.setifAWM(size_awm);																// KAT: Need to determine if/when these get set
	target.setActWeightingMatrix(_actWeightingMatrix); 

	// Pick optimizer algorithm
	SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
	//SimTK::OptimizerAlgorithm algorithm = SimTK::CFSQP;

	// Optimizer
	SimTK::Optimizer *optimizer = new SimTK::Optimizer(target, algorithm);

	// Optimizer options
	//cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
	optimizer->setDiagnosticsLevel(_printLevel);
	//cout<<"Setting optimizer convergence criterion to "<<_convergenceCriterion<<".\n";
	optimizer->setConvergenceTolerance(_convergenceCriterion);
	//cout<<"Setting optimizer maximum iterations to "<<_maximumIterations<<".\n";
	optimizer->setMaxIterations(_maximumIterations);
	optimizer->useNumericalGradient(false);
	optimizer->useNumericalJacobian(false);
	if(algorithm == SimTK::InteriorPoint) {
		// Some IPOPT-specific settings
		optimizer->setLimitedMemoryHistory(500); // works well for our small systems
		optimizer->setAdvancedBoolOption("warm_start",true);
		optimizer->setAdvancedRealOption("obj_scaling_factor",1);
		optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",1);
	}

	// Parameter bounds
	SimTK::Vector lowerBounds(np), upperBounds(np);
	if(size_awm<=0){ // Bounds from actuators if no actuator weighting matrix
		for(int i=0,j=0;i<fs.getSize();i++) {
			ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fs.get(i));
			lowerBounds(j) = act->getMinControl();
			upperBounds(j) = act->getMaxControl();
			j++;
		}
	}else{
		for(int i=0;i<np;i++) {								// KAT: Need to add if statement and variables to define upper/lowerBounds
			// Check if actuator is muscle (0 - 1 activation constraints)
			double s_sum = 0.0;
			double s_max = 0.0;
			for(int j=0;j<nm;j++) { // For muscles ONLY
				s_sum = s_sum + _actWeightingMatrix(j,i);
				if(_actWeightingMatrix(j,i) > s_max){s_max = _actWeightingMatrix(j,i);}
			}
			//std::cout<<s_sum<<endl;
			//std::cout<<i<<endl;
			if(s_sum==0){ // No actuators that are muscles have weights
				lowerBounds(i) = -10000;  // Default for reserve actuators
				upperBounds(i) = 10000;
			}else{ // Muscles that have max activation of 1
				lowerBounds(i) = 0;  
				upperBounds(i) = 1/s_max;
			}
		}
		//std::cout<<lowerBounds<<endl;
		//std::cout<<upperBounds<<endl;
		
	}
	
	target.setParameterLimits(lowerBounds, upperBounds);

	_parameters = 0; // Set initial guess to zeros

	// Static optimization
	_modelWorkingCopy->getMultibodySystem().realize(sWorkingCopy,SimTK::Stage::Velocity);
	target.prepareToOptimize(sWorkingCopy, &_parameters[0]);

	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	try {
		target.setCurrentState( &sWorkingCopy );
		optimizer->optimize(_parameters);
	}
	catch (const SimTK::Exception::Base& ex) {
		cout << ex.getMessage() << endl;
		cout << "OPTIMIZATION FAILED..." << endl;
		cout << endl;
		cout << "StaticOptimization2.record:  WARN- The optimizer could not find a solution at time = " << s.getTime() << endl;
		cout << endl;

		// WARNING messages about a weak model only if not using actuator weight matrix					// KAT
		if(size_awm<=0){
			double tolBounds = 1e-1;																		
			bool weakModel = false;																											
			string msgWeak = "The model appears too weak for static optimization.\nTry increasing the strength and/or range of the following force(s):\n";
			for(int a=0;a<na;a++) {
				Actuator* act = dynamic_cast<Actuator*>(&_forceSet->get(a));
				if( act ) {
					Muscle*  mus = dynamic_cast<Muscle*>(&_forceSet->get(a));
 					if(mus==NULL) {
			    		if(_parameters(a) < (lowerBounds(a)+tolBounds)) {
			    			msgWeak += "   ";
			    			msgWeak += act->getName();
			    			msgWeak += " approaching lower bound of ";
			    			ostringstream oLower;
			    			oLower << lowerBounds(a);
			    			msgWeak += oLower.str();
			    			msgWeak += "\n";
			    			weakModel = true;
			    		} else if(_parameters(a) > (upperBounds(a)-tolBounds)) {
			    			msgWeak += "   ";
			    			msgWeak += act->getName();
			    			msgWeak += " approaching upper bound of ";
			    			ostringstream oUpper;
			    			oUpper << upperBounds(a);
			    			msgWeak += oUpper.str();
			    			msgWeak += "\n";
			    			weakModel = true;
			    		} 
					} else {
			    		if(_parameters(a) > (upperBounds(a)-tolBounds)) {
			    			msgWeak += "   ";
			    			msgWeak += mus->getName();
			    			msgWeak += " approaching upper bound of ";
			    			ostringstream o;
			    			o << upperBounds(a);
			    			msgWeak += o.str();
			    			msgWeak += "\n";
			    			weakModel = true;
			    		}
					}
				}
			}
			if(weakModel) cout << msgWeak << endl;

			if(!weakModel) {
				double tolConstraints = 1e-6;
				bool incompleteModel = false;
				string msgIncomplete = "The model appears unsuitable for static optimization.\nTry appending the model with additional force(s) or locking joint(s) to reduce the following acceleration constraint violation(s):\n";
				SimTK::Vector constraints;
				target.constraintFunc(_parameters,true,constraints);
				const CoordinateSet& coordSet = _modelWorkingCopy->getCoordinateSet();
				for(int acc=0;acc<nacc;acc++) {
					if(fabs(constraints(acc)) > tolConstraints) {
						const Coordinate& coord = coordSet.get(_accelerationIndices[acc]);
						msgIncomplete += "   ";
						msgIncomplete += coord.getName();
						msgIncomplete += ": constraint violation = ";
						ostringstream o;
						o << constraints(acc);
						msgIncomplete += o.str();
						msgIncomplete += "\n";
						incompleteModel = true;
					}
				}
				if(incompleteModel) cout << msgIncomplete << endl;
			}
		}
	}

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//cout << "optimizer time = " << (duration*1.0e3) << " milliseconds" << endl;

	target.printPerformance(sWorkingCopy, &_parameters[0]);

	/* Calculate activations based upon current parameters */						// KAT: Only need if using activation weighting matrix
	SimTK::Matrix activ(na,1);
	activ = _actWeightingMatrix*_parameters;
	if(size_awm > 0){	
		/* Make sure activations are within actuator limits */												// KAT: Limits on activations based on control constraints
		for(int i=0; i<na; i++) {
			ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fs.get(i));
			if(activ(i,0) < act->getMinControl()) {
				activ(i,0) = act->getMinControl();}
			if(activ(i,0) > act->getMaxControl()) {
				activ(i,0) = act->getMaxControl();}
		}
	}

	//Update defaults for use in the next step

	const Set<Actuator>& actuators = _modelWorkingCopy->getActuators();
	for(int k=0; k < actuators.getSize(); ++k){
		ActivationFiberLengthMuscle *mus = dynamic_cast<ActivationFiberLengthMuscle*>(&actuators[k]);
		if(mus){
			mus->setDefaultActivation(activ(k,0)); // Set activations, not parameters (actuator weighting matrix)
			// Don't send up red flags when the def
			//mus->setObjectIsUpToDateWithProperties();
		}
	}

	//Write forces and activations to file															//KAT: Only execute when using synergies and need to add column headers
	SimTK::Vector forces(na);
	if(size_awm<=0){ // If no actuator weight matrix, just activations
		_activationStorage->append(sWorkingCopy.getTime(),na,&_parameters[0]);
		target.getActuation(const_cast<SimTK::State&>(sWorkingCopy), _parameters,forces);
		_forceStorage->append(sWorkingCopy.getTime(),na,&forces[0]);
	}else{		// activations + activation of weighted groups
		SimTK::Vector toStoreAct(actuators.getSize()+_parameters.size()); 
		for(int k=0; k < actuators.getSize(); k++){
			toStoreAct(k) = activ(k,0);
		}
		for(int k=0; k < _parameters.size(); k++){
			toStoreAct(actuators.getSize()+k) = _parameters(k);
		}
		_activationStorage->append(sWorkingCopy.getTime(),size_awm+na,&toStoreAct[0]);
		SimTK::Vector toStoreForces = activ.col(0);	
		target.getActuation(const_cast<SimTK::State&>(sWorkingCopy), toStoreForces, forces); 
		_forceStorage->append(sWorkingCopy.getTime(),na,&forces[0]);
	}

	return 0;
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration 
 *
 * @param s Current state .
 *
 * @return -1 on error, 0 otherwise.
 */
int StaticOptimization2::
begin(SimTK::State& s )
{
	if(!proceed()) return(0);

	// Make a working copy of the model
	delete _modelWorkingCopy;
	_modelWorkingCopy = _model->clone();
	_modelWorkingCopy->initSystem();

	// Replace model force set with only generalized forces
	if(_model) {
        SimTK::State& sWorkingCopyTemp = _modelWorkingCopy->updWorkingState();
		// Update the _forceSet we'll be computing inverse dynamics for
		if(_ownsForceSet) delete _forceSet;
		if(_useModelForceSet) {
			// Set pointer to model's internal force set
			_forceSet = &_modelWorkingCopy->updForceSet();
			_ownsForceSet = false;
		} else {
			ForceSet& as = _modelWorkingCopy->updForceSet();
			// Keep a copy of forces that are not muscles to restore them back.
			ForceSet* saveForces = as.clone();
			// Generate an force set consisting of a coordinate actuator for every unconstrained degree of freedom
			_forceSet = CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(sWorkingCopyTemp,*_modelWorkingCopy,1,false);
			_ownsForceSet = false;
			_modelWorkingCopy->setAllControllersEnabled(false);
			_numCoordinateActuators = _forceSet->getSize();
			// Copy whatever forces that are not muscles back into the model
			
			for(int i=0; i<saveForces->getSize(); i++){
				const Force& f=saveForces->get(i);
				if ((dynamic_cast<const Muscle*>(&saveForces->get(i)))==NULL)
					as.append(saveForces->get(i).clone());
			}
		}

		SimTK::State& sWorkingCopy = _modelWorkingCopy->initSystem();

		// Set modeiling options for Actuators to be overriden
		for(int i=0,j=0; i<_forceSet->getSize(); i++) {
			ScalarActuator* act = dynamic_cast<ScalarActuator*>(&_forceSet->get(i));
			if( act ) {
				act->overrideActuation(sWorkingCopy,true);
			}
		}

		sWorkingCopy.setQ(s.getQ());
		sWorkingCopy.setU(s.getU());
		sWorkingCopy.setZ(s.getZ());
		_modelWorkingCopy->getMultibodySystem().realize(s,SimTK::Stage::Velocity);
		_modelWorkingCopy->equilibrateMuscles(sWorkingCopy);
		// Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
		_accelerationIndices.setSize(0);
		const CoordinateSet& coordSet = _model->getCoordinateSet();
		for(int i=0; i<coordSet.getSize(); i++) {
			const Coordinate& coord = coordSet.get(i);
			if(!coord.isConstrained(sWorkingCopy)) {
				_accelerationIndices.append(i);
			}
		}

		const Set<Actuator>& fs = _modelWorkingCopy->getActuators();
		int na = fs.getSize();
		int nacc = _accelerationIndices.getSize();
		int size_awm = _awm.getSize();				// Size of actuator weighting matrix		// KAT

		// RECORD ACTUATOR WEIGHT MATRIX FROM FILE OR USE IDENTITY MATRIX			// KAT
		if(size_awm<=0){
			_actWeightingMatrix.resize(na,na);
			_actWeightingMatrix.diag() = 1; // Identity matrix
		}else{
			recordActWeightingMatrix(fs);
		}

		// Set number of parameters for optimization												// KAT
		int np = _actWeightingMatrix.ncol();

		if(na < nacc) 
			throw(Exception("StaticOptimization2: ERROR- overconstrained system -- need at least as many forces as there are degrees of freedom.\n"));

		_parameters.resize(np);														// KAT: Changed from na to np 
		_parameters = 0;
	}

	_statesSplineSet=GCVSplineSet(5,_statesStore);


	// DESCRIPTION AND LABELS
	constructDescription();
	// constructActivColumnLabels(); // KAT
	// constructForceColumnLabels(); // KAT

	deleteStorage();
	allocateStorage();

	// RESET STORAGE
	_activationStorage->reset(s.getTime());
	_forceStorage->reset(s.getTime());

	// RECORD
	int status = 0;
	if(_activationStorage->getSize()<=0 && _forceStorage->getSize()<=0) {
		status = record(s);
		const Set<Actuator>& fs = _modelWorkingCopy->getActuators();
		for(int k=0;k<fs.getSize();k++) {
			ScalarActuator* act = dynamic_cast<ScalarActuator*>(&fs.get(k));
			cout << "Bounds for " << act->getName() << ": " << act->getMinControl()<< " to "<< act->getMaxControl() << endl;
		}
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s Current state .
 *
 * @return -1 on error, 0 otherwise.
 */
int StaticOptimization2::
step(const SimTK::State& s, int stepNumber )
{
	if(!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * KAT: This method is called during the analysis to record the synergies
 * specified in SynergySet.
 *
 * input: actuator set of model (model_actuators)
 * output: _actWeightingMatrix
 */
int StaticOptimization2::
recordActWeightingMatrix(const Set<Actuator>& model_actuators)
{
	//Set vector size to number of actuators
	_actWeightingMatrix.resize(model_actuators.getSize(),_awm.getSize());

	/* Check to make sure num_synergies = number of synergies specified
	if(_awm.getSize() != _nSynergies) {
		throw(Exception("StaticOptimization2: ERROR- number of synergies provided is different from number specified (num_synergies)\n"));
	} */

	//Add vectors from xml to weighting matrix
	for (int i=0; i< _awm.getSize(); i++)
	{
		ActWeightingVector *awm_element = dynamic_cast<ActWeightingVector*>(&_awm.get(i));
		Array<double> awv = awm_element->getActWeightingVector(); 
		// TO DO: Check if unit length
		for (int j=0; j < model_actuators.getSize(); j++)
		{
			_actWeightingMatrix(j,i) = awv[j];
		}
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * @param s Current state 
 *
 * @return -1 on error, 0 otherwise.
 */
int StaticOptimization2::
end( SimTK::State& s )
{
	if(!proceed()) return(0);

	record(s);

	return(0);
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int StaticOptimization2::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACTIVATIONS
	Storage::printResult(_activationStorage,aBaseName+"_"+getName()+"_activation",aDir,aDT,aExtension);

	// FORCES
	Storage::printResult(_forceStorage,aBaseName+"_"+getName()+"_force",aDir,aDT,aExtension);

	// Make a ControlSet out of activations for use in forward dynamics
	ControlSet cs(*_activationStorage);
	std::string path = (aDir=="") ? "." : aDir;
	std::string name = path + "/" + aBaseName+"_"+getName()+"_controls.xml";
	cs.print(name);
	return(0);
}
