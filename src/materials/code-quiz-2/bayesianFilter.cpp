//============================================================================
// Name        : bayesianFilter.cpp
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//constructor:
bayesianFilter::bayesianFilter() {

    //TODO add is_initialized to header file
    //set initialization to false:
    // NOTE: helps us set up the initial believe state
	//is_initialized_ = 

    //TODO add control_std to header file
	//set standard deviation of control to 1.0f:
	//control_std = 

	//define size of believe, same size as map
	bel_x.resize(100,0);
	
	//TODO add bel_x_init to header file
	// NOTE: helps us not overwrite believe during
	// the motion calculation
	//bel_x_init

}

//de-constructor:
bayesianFilter::~bayesianFilter() {

}

void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
        						             const map &map_1d,
                                         help_functions &helpers){

	
	
	/******************************************************************************
	 *  motion model and observation update
	******************************************************************************/
	std::cout <<"-->motion model for state x ! \n" << std::endl;

	//get current observations and control information:
	MeasurementPackage::control_s     controls = measurements.control_s_;
	MeasurementPackage::observation_s observations = measurements.observation_s_;



		/**************************************************************************
		 *  posterior for motion model
		**************************************************************************/

        // motion posterior:
        // used to set up the convlution
		float posterior_motion = 0.0f;

		
};

