#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//constructor:
bayesianFilter::bayesianFilter() {


	//set initialization to false:
	is_initialized_ = false;

	//set standard deviation of control:
	control_std     = 1.0f;

	//set standard deviation of observations:
	observation_std = 1.0f;

	//define size of different state vectors:
	bel_x.resize(100,0);
	bel_x_init.resize(100,0);

}

//de-constructor:
bayesianFilter::~bayesianFilter() {

}

bool sort_decreasing(float a, float b) {return a > b;}

void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
                                         const map &map_1d,
                                         help_functions &helpers){

	/******************************************************************************
	 *  Set init belief of state vector:
	 ******************************************************************************/
	if(!is_initialized_){

		//run over map:
		for (unsigned int l=0; l< map_1d.landmark_list.size(); ++l){

			//define landmark:
			map::single_landmark_s landmark_temp;
			//get landmark from map:
			landmark_temp = map_1d.landmark_list[l];

			//check, if landmark position is in the range of state vector x:
			if(landmark_temp.x_f > 0 && landmark_temp.x_f < bel_x_init.size() ){

				//cast float to int:
				int position_x = int(landmark_temp.x_f) ;
				//set belief to 1:
				bel_x_init[position_x]   = 1.0f;
				bel_x_init[position_x-1] = 1.0f;
				bel_x_init[position_x+1] = 1.0f;

			} //end if
		}//end for

    //normalize belief at time 0:
    bel_x_init = helpers.normalize_vector(bel_x_init);

    //set initial flag to true:
    is_initialized_ = true ;

	}//end if


	/******************************************************************************
	 *  motion model and observation update
   ******************************************************************************/
	std::cout <<"-->motion model for state x ! \n" << std::endl;

	//get current observations and control information:
	MeasurementPackage::control_s     controls = measurements.control_s_;
	MeasurementPackage::observation_s observations = measurements.observation_s_;

	//run over the whole state (index represents the pose in x!):
	for (unsigned int i=0; i< bel_x.size(); ++i){


		float pose_i = float(i) ;
		/**************************************************************************
		 *  posterior for motion model
     **************************************************************************/

		// motion posterior:
		float posterior_motion = 0.0f;

		//loop over state space x_t-1 (convolution):
		for (unsigned int j=0; j< bel_x.size(); ++j){
			float pose_j = float(j) ;


			float distance_ij = pose_i-pose_j;

			//transition probabilities:
			float transition_prob = helpers.normpdf(distance_ij,
                                              controls.delta_x_f,
                                              control_std) ;
			//motion model:
			posterior_motion += transition_prob*bel_x_init[j];
		}

		/**************************************************************************
		 *  observation update:
     **************************************************************************/

    // Split the landmarks into the left_landmarks, and the right_landmarks
    std::vector<float> left_landmarks, right_landmarks;
    for (uint l = 0; l < map_1d.landmark_list.size(); ++l) {
      if (map_1d.landmark_list[l].x_f < pose_i) {
        left_landmarks.push_back(map_1d.landmark_list[l].x_f);
      } else {
        right_landmarks.push_back(map_1d.landmark_list[l].x_f);
      }
    }

    //sort(left_landmarks.begin(), left_landmarks.end(), sort_decreasing);
    sort(right_landmarks.begin(), right_landmarks.end());

    //cout << "Done: splitting landmarks: " << i << endl;

    // Split the observations into negative_observations, and positive_observations
    std::vector<float> negative_observations, positive_observations;
    for (uint z = 0; z < observations.distance_f.size(); ++z) {
      if (observations.distance_f[z] < 0) {
        negative_observations.push_back(observations.distance_f[z]);
      } else {
        positive_observations.push_back(observations.distance_f[z]);
      }
    }
    //sort(negative_observations.begin(), negative_observations.end(), sort_decreasing);
    sort(positive_observations.begin(), positive_observations.end());
    //cout << "Done: splitting observations: " << i << endl;

    //define observation posterior:
		float posterior_obs = 1.0f ;
    // work on the negative_observations
    float theorectical_distance = 0;
    // for (uint z = 0; z < negative_observations.size(); ++z) {
    //   // Pair the observations and landmarks, to compute the theoretical distance to the landmarks paired.
    //   theorectical_distance = left_landmarks[z] - pose_i;
    //   // Use the theoretical_distance as the mean to compute the observation posterior
    //   posterior_obs *= helpers.normpdf(negative_observations[z],
    //                                    theorectical_distance,
    //                                    observation_std);
    // }
    //cout << "Done: negative observations posterior: " << i << endl;

    for (uint z = 0; z < positive_observations.size(); ++z) {
      // Pair the observations and landmarks, to compute the theoretical distance to the landmarks paired.
      theorectical_distance = right_landmarks[z] - pose_i;
      // Use the theoretical_distance as the mean to compute the observation posterior
      posterior_obs *= helpers.normpdf(positive_observations[z],
                                       theorectical_distance,
                                       observation_std);
    }
    //cout << "Done: positive observations posterior: " << i << endl;

		/**************************************************************************
		 *  finalize bayesian localization filter:
		 *************************************************************************/

		//update = observation_update* motion_model
		bel_x[i] = posterior_obs*posterior_motion ;

	};

	//normalize:
	bel_x = helpers.normalize_vector(bel_x);

	//set bel_x to belief_init:
	bel_x_init = bel_x;
};
