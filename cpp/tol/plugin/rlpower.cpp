//
// Created by Milan Jelisavcic on 28/03/16.
//

#include "rlpower.h"
//#include "sensor.h"
//#include "actuator.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include <algorithm>
#include <stdexcept>
#include <cstdlib>
#include <map>
#include <string>
#include <sstream>
#include <cmath>

#include <random>
#include <iostream>
#include <fstream>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

namespace tol {

RLPower::RLPower(std::string modelName, EvaluatorPtr evaluator, std::vector< revolve::gazebo::MotorPtr >& actuators, std::vector< revolve::gazebo::SensorPtr >& sensors) :
        nActuators_(actuators.size()),
        nSensors_(sensors.size()),
        start_eval_time_(0),
        generation_counter_(0),
        cycle_start_time_(-1),
        evaluator_(evaluator) {

    source_y_size = RLPower::INITIAL_SPLINE_SIZE;
    noise_sigma_ = RLPower::SIGMA_START_VALUE;

    std::cout << "RLPower::RLPower()" << std::endl;
    // Create transport node
//     node_.reset(new ::gazebo::transport::Node());
//     node_->Init();

    // Listen to network modification requests
//     alterSub_ = node_->Subscribe("~/"+modelName+"/modify_neural_network",
//                                  &RLPower::modify, this);

    step_rate_ = RLPower::MAX_SPLINE_SAMPLES / source_y_size;


    std::random_device rd;
    std::mt19937 mt(rd());
    std::normal_distribution<double> dist(-1, 1);

    // Init first random controller
    current_policy_ = std::make_shared<Policy>(nActuators_);
    for (int i = 0; i < nActuators_; i++) {
        std::cout << " Init first random controller! round "<< i+1 << std::endl;
        //Spline &spline = current_policy_->at(i);
        Spline spline(source_y_size);
        for (int j = 0; j < source_y_size; j++) {
        std::cout << " #### first random controller! round "<< i+1 << " part " << j+1 << std::endl;
            spline[j] = dist(mt);
            std::cout << "  rand: "<< spline[j] << std::endl;
        }
        current_policy_->at(i) = spline;
    }

    // Init of empty cache
    interpolation_cache_ = std::make_shared<Policy>(nActuators_);
    for (int i = 0; i < nActuators_; i++) {
        std::cout << " Reserve interpolation_cache_ part " << i+1 << std::endl;
        interpolation_cache_->at(i).resize(INTERPOLATION_CACHE_SIZE, 0);
    }

    this->generateCache();
    evaluator_->start();
    std::cout<<"RLPower::RLPower() # constructor end\n"<<std::endl;
}

RLPower::~RLPower() {
    // `boost::shared_ptr< Policy >` should take care of memory management for us
}

double RLPower::getFitness()
{
        //Calculate fitness for current policy
        return evaluator_->fitness();
}

void RLPower::generatePolicy()
{
    // Calculate fitness for current policy
    double curr_fitness = this->getFitness();

    // Insert ranked policy in list
    std::cout << "Generation " << generation_counter_ << " fitness " << curr_fitness << std::endl;
    this->writeCurrent(curr_fitness);
    ranked_policies_.push_back(PolicySave(curr_fitness, current_policy_));
//     std::sort(ranked_policies_.begin(), ranked_policies_.end(), [](const PolicySave &a, const PolicySave &b) {
//         return b > a;
//     });
    std::sort(ranked_policies_.begin(), ranked_policies_.end(), std::greater<PolicySave>());

    std::cout << "ranked_policies_: ";
    for (const PolicySave &p : ranked_policies_) {
        std::cout << "," << p.fitness_;
    }
    std::cout << std::endl;

    // Remove worst policies
    while (ranked_policies_.size() > RLPower::MAX_RANKED_POLICIES) {
//         auto last = std::prev(ranked_policies_.end());
//         ranked_policies_.erase(last);
        ranked_policies_.pop_back();
    }

    if (generation_counter_ == (RLPower::MAX_EVALUATIONS - 1))
        this->writeLast();

    // Set variables for new policy
    generation_counter_++;

    if (generation_counter_ == RLPower::MAX_EVALUATIONS - 1)
        std::cout << "Finish!!!" << std::endl;

    // increase spline points if it is time
    if (generation_counter_ % RLPower::UPDATE_STEP == 0) {
        source_y_size++;

        // LOG code
        step_rate_ = RLPower::MAX_SPLINE_SAMPLES / source_y_size;
        std::cout << "New samplingSize_=" << source_y_size << ", and stepRate_=" << step_rate_ << std::endl;

        // current policy
        Policy policy_copy(current_policy_->size());
        for (int i=0; i<nActuators_; i++) {
            Spline &spline = current_policy_->at(i);
            policy_copy[i] = Spline(spline.begin(), spline.end());

            spline.resize(source_y_size);
        }


        interpolateCubic(&policy_copy, current_policy_.get());


        //for every ranked policy
        for(auto policy_it = ranked_policies_.begin(); policy_it != ranked_policies_.end(); policy_it++) {
            PolicyPtr policy = policy_it->policy_;

            for (int j=0; j<nActuators_; j++) {
                Spline &spline = policy->at(j);
                policy_copy[j] = Spline(spline.begin(), spline.end());
                spline.resize(source_y_size);
            }

            interpolateCubic(&policy_copy, policy.get());
            }

    }


    // Actual policy generation
    std::random_device rd;
    std::mt19937 mt(rd());
    std::normal_distribution<double> dist(0, noise_sigma_);
    noise_sigma_ *= SIGMA_DECAY_SQUARED;

    double total_fitness = 0;
    for (auto it = ranked_policies_.begin(); it != ranked_policies_.end(); it++) {
        // first is fitness
        total_fitness += it->fitness_;
    }

    // for actuator
    for (int i = 0; i < nActuators_; i++) {
        // for column
        for (int j = 0; j < source_y_size; j++) {

            // modifier ...
            double spline_point = 0;
            for (auto it = ranked_policies_.begin(); it != ranked_policies_.end(); it++) {
                // first → fitness
                // second → policy
                spline_point += ((it->policy_->at(i)[j] - (*current_policy_)[i][j])) * it->fitness_;
            }
            spline_point /= total_fitness;

            // ... + noise + current
            spline_point += dist(mt) + (*current_policy_)[i][j];

            (*current_policy_)[i][j] = spline_point;
        }
    }

    // cache update
    this->generateCache();
}

void print_spline(const RLPower::Spline &spline) {
    std::cout << "[ ";
    for (auto spline_it = spline.begin(); spline_it != spline.end(); spline_it++) {
        if (spline_it != spline.begin()) {
            std::cout << ", ";
        }
        std::cout << (*spline_it);
    }
    std::cout << " ]" << std::endl;
}

void print_policy(RLPower::Policy * const policy) {
    for (auto policy_it = policy->begin(); policy_it != policy->end(); policy_it++) {
        std::cout << " spline size = " << policy_it->size() << " - ";
        print_spline(*policy_it);
    }
}

void RLPower::generateCache()
{
//     print_policy(current_policy_.get());
//     print_policy(interpolation_cache_.get());
    this->interpolateCubic(current_policy_.get(), interpolation_cache_.get());
//     print_policy(current_policy_.get());
//     print_policy(interpolation_cache_.get());
}

void RLPower::generateOutput(const double time, double* output_vector)
{
    if (cycle_start_time_ < 0) {
        cycle_start_time_ = time;
    }

    // get correct X value (between 0 and CICLE_LENGTH)
    double x = time - cycle_start_time_;
    while (x >= RLPower::CICLE_LENGTH) {
        cycle_start_time_ += RLPower::CICLE_LENGTH;
        x = time - cycle_start_time_;
    }

    // adjust X on the cache coordinate space
    x = (x/CICLE_LENGTH) * INTERPOLATION_CACHE_SIZE;
    // generate previous and next values
    int x_a = ((int)x) % INTERPOLATION_CACHE_SIZE;
    int x_b = (x_a+1) % INTERPOLATION_CACHE_SIZE;

    // linear interpolation for every actuator
    for (int i=0; i<nActuators_; i++) {
        double y_a = interpolation_cache_->at(i)[x_a];
        double y_b = interpolation_cache_->at(i)[x_b];

        output_vector[i] = y_a +
            ((y_b - y_a) * (x - x_a) / (x_b - x_a));
    }
}


void RLPower::update(const std::vector< revolve::gazebo::MotorPtr >& actuators, const std::vector< revolve::gazebo::SensorPtr >& sensors, double t, double step) {
//     boost::mutex::scoped_lock lock(networkMutex_);

    // Evaluate policy on certain time limit
    if ((t-start_eval_time_) > RLPower::FREQUENCY_RATE && generation_counter_ < RLPower::MAX_EVALUATIONS) {
        std::cout << "################# GENERATING NEW POLICY !!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        this->generatePolicy();
        start_eval_time_ = t;
        evaluator_->start();
    }

    // generate outputs
    double output_vector[nActuators_];
    this->generateOutput(t, output_vector);

    // Send new signals to the actuators
    unsigned int p = 0;
    for (auto actuator: actuators) {
        actuator->update(&output_vector[p], step);
        p += actuator->outputs();
    }
}

void RLPower::interpolateCubic(Policy * const source_y, Policy *destination_y)
{
    const unsigned int source_y_size = (*source_y)[0].size();
    const unsigned int destination_y_size = (*destination_y)[0].size();

   const unsigned int N = source_y_size;
    double *x = new double[N+1];
    double *y = new double[N+1];
    double *x_new = new double[destination_y_size];

    gsl_interp_accel *acc = gsl_interp_accel_alloc ();
    const gsl_interp_type *t = gsl_interp_cspline_periodic;
    gsl_spline *spline = gsl_spline_alloc (t, N+1);

    // init x
    double step_size = ((double)CICLE_LENGTH) / source_y_size;
    double x_ = 0;
    for (int i = 0; i < N+1; i++) {
        x[i] = x_;
        x_ += step_size;
    }

    // init x_new
    step_size = ((double)CICLE_LENGTH) / destination_y_size;
    x_ = 0;
    for (int i = 0; i <= destination_y_size; i++) {
        x_new[i] = x_;
        x_ += step_size;
    }


    for (int j = 0; j< nActuators_; j++) {
        Spline &source_y_line = source_y->at(j);
        Spline &destination_y_line = destination_y->at(j);

        // init y
        // TODO use memcpy
        for (int i = 0; i < N; i++) {
            y[i] = source_y_line[i];
        }

        // make last equal to first
        y[N] = y[0];

        gsl_spline_init(spline, x, y, N+1);

//         std::cout << "[ ";
//         for (int i=0; i< destination_y_size; i++) {
//             std::cout << destination_y_line[i] << ", ";
//         }
//         std::cout << " ]"<< std::endl;

        for (int i = 0; i < destination_y_size; i++) {
            destination_y_line[i] = gsl_spline_eval (spline, x_new[i], acc);
        }

//         std::cout << "[ ";
//         for (int i=0; i< destination_y_size; i++) {
//             std::cout << destination_y_line[i] << ", ";
//         }
//         std::cout << " ]"<< std::endl;

    }

        std::cout << "######################################################### BEFORE" << std::endl;

    gsl_spline_free (spline);
        std::cout << "######################################################### 0" << std::endl;
    gsl_interp_accel_free (acc);
        std::cout << "######################################################### 1" << std::endl;

//     delete[] x_new;
        std::cout << "######################################################### 2" << std::endl;
//     delete[] y;
        std::cout << "######################################################### 3" << std::endl;
//     delete[] x;
        std::cout << "######################################################### END" << std::endl;
}


void RLPower::printCurrent() {
    for (int i = 0; i < RLPower::MAX_SPLINE_SAMPLES; i++) {
        for (int j = 0; j < nActuators_; j++) {
            std::cout << current_policy_->at(j)[i] << " ";
        }
        std::cout << std::endl;
    }
}

void RLPower::writeCurrent(double current_fitness) {
    std::ofstream outputFile;
    std::string uri = "~/output/spider_";
    outputFile.open(uri + ".csv", std::ios::app | std::ios::out | std::ios::ate);
    // std::to_string(currEval_) -------^
    // outputFile << "id,fitness,steps,policy" << std::endl;
    outputFile << generation_counter_ << "," << current_fitness << "," << step_rate_ << ",";
    for (int i = 0; i < RLPower::MAX_SPLINE_SAMPLES; i++) {
        for (int j = 0; j < nActuators_; j = j + step_rate_) {
            outputFile << current_policy_->at(j)[i] << ":";
        }
    }
    outputFile << std::endl;
    outputFile.close();
}

void RLPower::writeLast() {
    std::ofstream outputFile;
    std::string uri = "~/output/spider_last_";
    outputFile.open(uri + ".csv", std::ios::app);
    // std::to_string(currEval_) -------^
    outputFile << "id,fitness,steps,policy" << std::endl;
    for (auto iterator = ranked_policies_.begin(); iterator != ranked_policies_.end(); iterator++) {
        outputFile << iterator->fitness_ << "," << step_rate_ << ",";
        for (int i = 0; i < RLPower::MAX_SPLINE_SAMPLES; i++) {
            for (int j = 0; j < nActuators_; j++) {
                outputFile << iterator->policy_->at(j)[i] << ":";
            }
        }
        outputFile << std::endl;
    }
    outputFile << std::endl;
    outputFile.close();
}

} /* namespace tol */
