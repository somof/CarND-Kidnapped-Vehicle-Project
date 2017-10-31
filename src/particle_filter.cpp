/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "float.h"
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
   // 初期化の際だけ、prediction()の代りに呼ばれる

   // Set the number of particles.
   num_particles = 100;
   particles.resize(num_particles);

   // creates a normal (Gaussian) distribution
   std::default_random_engine       gen;
   std::normal_distribution<double> dist_x(x, std[0]);
   std::normal_distribution<double> dist_y(y, std[1]);
   std::normal_distribution<double> dist_theta(theta, std[2]);

   // Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
   // Add random Gaussian noise to each particle.
   for (auto & p: particles) {
      p.x      = dist_x(gen);
      p.y      = dist_y(gen);
      p.theta  = dist_theta(gen);
      p.weight = 1;
   }

   is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
   // 運動モデルから、1単位時間先を予測する

   // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
   //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   //  http://www.cplusplus.com/reference/random/default_random_engine/

   std::default_random_engine       gen;
   std::normal_distribution<double> N_x(0, std_pos[0]);
   std::normal_distribution<double> N_y(0, std_pos[1]);
   std::normal_distribution<double> N_theta(0, std_pos[2]);

   // Prevent Zero Division
   if (yaw_rate <= FLT_MIN) {
      yaw_rate += FLT_MIN;
   }

   // Add measurements to each particle
   for (auto & p: particles) {
      // 運動モデルから算出した予測式
      p.x     += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y     += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta += yaw_rate * delta_t;

      // Add random Gaussian noise.
      p.x     += N_x(gen);
      p.y     += N_y(gen);
      p.theta += N_theta(gen);
   }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
   // 観測点毎に、最も近い予測点を検索する → 尤度が高い点とする

   // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
   //   implement this method and use it as a helper during the updateWeights phase.

   // Find the predicted measurement that is closest to each observed measurement
   // assign the observed measurement to this particular landmark.
   for (auto & observed_meas: observations) {
      auto min_distance = std::numeric_limits<double>::max();
      for (const auto & predicted_meas: predicted) {
         auto distance = dist(observed_meas.x, observed_meas.y, predicted_meas.x, predicted_meas.y);
         if (distance < min_distance) {
            min_distance     = distance;
            observed_meas.id = predicted_meas.id;
         }
      }
   }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
   // 重みを更新する

   // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
   //   according to the MAP'S coordinate system. You will need to transform between the two systems.
   //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
   //   The following is a good resource for the theory:
   //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   //   and the following is a good resource for the actual equation to implement (look at equation
   //   3.33
   //   http://planning.cs.uiuc.edu/node99.html

   // Update the weights of each particle using a mult-variate Gaussian distribution.
   // more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   for (auto & p: particles) {

      // センサの感度範囲になるランドマークを集める
      vector<LandmarkObs> predictions;
      for (const auto landmark: map_landmarks.landmark_list) {
        if (dist(p.x, p.y, landmark.x_f, landmark.y_f) < sensor_range) {
           predictions.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
        }
      }
      // sensor range limitation caused no effect...
      // for (const auto landmark: map_landmarks.landmark_list) {
      //    predictions.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
      // }


      // 14. Converting Landmark Observations
      // 観測点を地図の座標系に変換する
      vector<LandmarkObs> observations_map;
      double cos_theta = cos(p.theta);
      double sin_theta = sin(p.theta);
      for (const auto obs: observations) {
         observations_map.push_back({0, // id will be set at dataAssociation()
                  p.x + obs.x * cos_theta - obs.y * sin_theta,
                  p.y + obs.x * sin_theta + obs.y * cos_theta});
      }

      // 9. Data Association: Nearest Neighbor
      // 観測点毎に、最も近いランドマークの予測点を検索する → 尤度が高い点とする
      dataAssociation(predictions, observations_map);

      // 11. Update Step
      // 18. Calculating the Particle's Final Weight
      // 各particleのweight(尤度)を計算し直す
      p.weight = 1.0;
      for (const auto obs_map: observations_map) {
         Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_map.id - 1); // landmark ID start from 1
         const auto exponent     = - (pow(obs_map.x - landmark.x_f, 2.) / pow(std_landmark[0], 2.) / 2. +
                                      pow(obs_map.y - landmark.y_f, 2.) / pow(std_landmark[1], 2.) / 2.);
         const auto denominator  = 2. * M_PI * std_landmark[0] * std_landmark[1];
         p.weight               *= exp(exponent) / denominator;;
      }

      weights.push_back(p.weight);
   }
}

void ParticleFilter::resample() {
   // いくつかあるリサンプル方法のうちの一つを選択
   // - 単純ランダム抽出法（simple random sampling）
   // - 系統抽出法（systematic sampling）
   // - 層別抽出法（stratified sampling）
   // - Kullbuck-Leibler divergence(KLD sampling)

   // refer to http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   // discrete_distribution(InputIterator firstW, InputIterator lastW);
   // 確率列 weights のイテレータ範囲を指定するコンストラクタ

   // weightsの値に基いて値を分布させて、確率列から選択された0から始まるインデックスを返す
   std::random_device rd;
   std::mt19937       gen(rd());
   std::discrete_distribution<> dist(weights.begin(), weights.end());
   weights.clear();

   // Resample particles with replacement with probability proportional to their weight.
   vector<Particle> resampled_particles;
   resampled_particles.resize(num_particles);
   for (int i = 0; i < num_particles; i++) {
      int index = dist(gen);
      resampled_particles[i] = particles[index];
   }
   particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
   //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
   // associations: The landmark id that goes along with each listed association
   // sense_x: the associations x mapping already converted to world coordinates
   // sense_y: the associations y mapping already converted to world coordinates

   //Clear the previous associations
   particle.associations.clear();
   particle.sense_x.clear();
   particle.sense_y.clear();

   particle.associations= associations;
   particle.sense_x = sense_x;
   particle.sense_y = sense_y;

   return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
   vector<int> v = best.associations;
   stringstream ss;
   copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
   string s = ss.str();
   s = s.substr(0, s.length()-1);  // get rid of the trailing space
   return s;
}
string ParticleFilter::getSenseX(Particle best)
{
   vector<double> v = best.sense_x;
   stringstream ss;
   copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
   string s = ss.str();
   s = s.substr(0, s.length()-1);  // get rid of the trailing space
   return s;
}
string ParticleFilter::getSenseY(Particle best)
{
   vector<double> v = best.sense_y;
   stringstream ss;
   copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
   string s = ss.str();
   s = s.substr(0, s.length()-1);  // get rid of the trailing space
   return s;
}
