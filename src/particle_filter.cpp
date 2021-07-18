/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::numeric_limits;
using std::uniform_int_distribution;
using std::uniform_real_distribution;

using std::ostream_iterator;
using std::stringstream;
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
if(is_initialized){
  return;
}
  num_particles = 100;  // TODO: Set the number of particles

  // Set the Standard deviations of position and heading 
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

 // set initial position and heading with normal distributions
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);


  //initialize all particiles 
  for (int i = 0; i < num_particles; ++i) {
    Particle particle;
    particle.x=dist_x(gen);
    particle.y=dist_y(gen);
    particle.theta=dist_theta(gen);
    particle.weight=1;
    particles.push_back(particle);

  }
  is_initialized=true;
  }

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */


  // standard deviations
  double std_x=std_pos[0]; 
  double std_y=std_pos[1];
  double std_theta=std_pos[2];


  // for normal distribuation we have 
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);
  
// update particles locations 
 for (int i = 0; i < num_particles; ++i) {
    double theta = particles[i].theta;
// I added a small number to the yaw rate to avoid dividing by zero 
      particles[i].x += velocity / (yaw_rate+0.000001) * (sin(theta + yaw_rate * delta_t) - sin(theta)) + dist_x(gen);
      particles[i].y += velocity / (yaw_rate+0.000001) * (cos(theta) - cos(theta + yaw_rate * delta_t)) +dist_y(gen);
      particles[i].theta += yaw_rate * delta_t + dist_theta(gen);


  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  //storing the number of predicitions and observations
  unsigned int nPredictions = predicted.size();
  unsigned int nObservations = observations.size();

  for (unsigned int i=0;i<nObservations;i++){

    // the min distance is init as a large number
    double Dist_min=numeric_limits<double>::max(); 
    
    int mapId=-1; 

    for (unsigned j=0; j<nPredictions;j++){

      double xDist = observations[i].x - predicted[j].x;
      double yDist = observations[i].y - predicted[j].y;

      // find the total distance
      double distance = sqrt(xDist * xDist + yDist * yDist);
      if(distance <Dist_min){
        Dist_min=distance; 
        mapId=predicted[j].id;
      }

    }
    observations[i].id=mapId;

  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */


  //  Standard deviations of landmark positions
  double stdLandmarkX = std_landmark[0];
  double stdLandmarkY = std_landmark[1];


   // for each particle we check closest landmarks and update weights
  for (int i = 0; i < num_particles; i++) {
    
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
 
 //  landmarks in the particles' sensor range
    vector<LandmarkObs> inRangeLandmarks;

    double sensor_range2 = sensor_range * sensor_range;

    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      float landmarkX = map_landmarks.landmark_list[j].x_f;
      float landmarkY = map_landmarks.landmark_list[j].y_f;
      int id = map_landmarks.landmark_list[j].id_i;
      double dX = x - landmarkX;
      double dY = y - landmarkY;
     
     
      if ( dX*dX + dY*dY <= sensor_range2 ) {
        inRangeLandmarks.push_back(LandmarkObs{ id, landmarkX, landmarkY });
      }


// Observation coordinates transformation from particle coordinate to map coordinates
    vector<LandmarkObs> Observations_in_map;
 for (unsigned int j = 0; j < observations.size(); j++) {

      double mappedX = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double mappedY = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      
      //Recording the new transformed position 
      Observations_in_map.push_back(LandmarkObs{ observations[j].id, mappedX, mappedY });
    
    }

    //  indext Update for the nearest landmark 
    dataAssociation(inRangeLandmarks, Observations_in_map);

    // Weights reset 
    particles[i].weight = 1.0;

    // Calculating weights for each mapped observation
     for (unsigned int j = 0; j < Observations_in_map.size(); j++) {
      double observationX = Observations_in_map[j].x;
      double observationY = Observations_in_map[j].y;

      // get x,y of the nearest landmark
      int landmarkId = Observations_in_map[j].id;
      double landmarkX, landmarkY;
      unsigned int k = 0;
      unsigned int nLandmarks = inRangeLandmarks.size();
      bool found = false;
      while (!found && k < nLandmarks) {
        if (inRangeLandmarks[k].id == landmarkId) {
          found = true;
          landmarkX = inRangeLandmarks[k].x;
          landmarkY = inRangeLandmarks[k].y;
        }
        k++;
      }

      // Calculating weight.
      double dX = observationX - landmarkX;
      double dY = observationY - landmarkY;

      // Multivariate-Gaussian Probability
      double weight = (1/(2*M_PI*stdLandmarkX*stdLandmarkY)) * exp(-(dX*dX/(2*stdLandmarkX*stdLandmarkX) + (dY*dY/(2*stdLandmarkY*stdLandmarkY))));
      if (weight == 0) {
        // avoid weight of zero
        particles[i].weight *= 0.0001;
      } else {
        particles[i].weight *= weight;
      }
    }
    }

}}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
 // initial index is retrieved by random.
  uniform_int_distribution<int> distInt(0, num_particles - 1);
  int index = distInt(gen);

  double beta = 0.0;

  // Getting the weights max. 
  vector<double> weights;
  double maxWeight = numeric_limits<double>::min();
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if (particles[i].weight > maxWeight) {
      maxWeight = particles[i].weight;
    }
  }

  // Creating uniform real distributions
  uniform_real_distribution<double> distDouble(0.0, maxWeight);

  // Finding the Resampled Particles
  vector<Particle> resampledParticles;
  for (int i = 0; i < num_particles; i++) {
    beta += distDouble(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
  }

  particles = resampledParticles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}