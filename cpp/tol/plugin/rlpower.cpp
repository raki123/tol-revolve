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

    RLPower::RLPower(std::string modelName, EvaluatorPtr evaluator, std::vector <revolve::gazebo::MotorPtr> &actuators,
                     std::vector <revolve::gazebo::SensorPtr> &sensors) :
            nActuators_(actuators.size()),
            nSensors_(sensors.size()),
            start_eval_time_(0),
            generation_counter_(0),
            evaluator_(evaluator),
            cycle_start_time_(-1),
            robot_name_(modelName) {

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
        std::normal_distribution<double> dist(0, this->noise_sigma_);

        // Init first random controller
        current_policy_ = std::make_shared<Policy>(nActuators_);
        for (unsigned int i = 0; i < nActuators_; i++) {
            Spline spline(source_y_size);
            for (unsigned int j = 0; j < source_y_size; j++) {
                spline[j] = dist(mt);
            }
            current_policy_->at(i) = spline;
        }

        // Init of empty cache
        interpolation_cache_ = std::make_shared<Policy>(nActuators_);
        for (unsigned int i = 0; i < nActuators_; i++) {
            interpolation_cache_->at(i).resize(INTERPOLATION_CACHE_SIZE, 0);
        }

        this->generateCache();
        evaluator_->start();
    }

    RLPower::~RLPower() {
        // `boost::shared_ptr< Policy >` should take care of memory management for us
    }

    void RLPower::update(const std::vector <revolve::gazebo::MotorPtr> &actuators,
                         const std::vector <revolve::gazebo::SensorPtr> &sensors, double t, double step) {
//     boost::mutex::scoped_lock lock(networkMutex_);

        // Evaluate policy on certain time limit
        if ((t - start_eval_time_) > RLPower::FREQUENCY_RATE && generation_counter_ < RLPower::MAX_EVALUATIONS) {
            this->generatePolicy();
            start_eval_time_ = t;
            evaluator_->start();
        }

        // generate outputs
        double *output_vector = new double[nActuators_];
        this->generateOutput(t, output_vector);

        // Send new signals to the actuators
        unsigned int p = 0;
        for (auto actuator: actuators) {
            actuator->update(&output_vector[p], step);
            p += actuator->outputs();
        }

        delete[] output_vector;
    }

    void RLPower::generatePolicy() {
        // Calculate fitness for current policy
        double curr_fitness = this->getFitness();

        // Insert ranked policy in list
        ranked_policies_.insert({curr_fitness, current_policy_});

        // Remove worst policies
        while (ranked_policies_.size() > RLPower::MAX_RANKED_POLICIES) {
            auto last = std::prev(ranked_policies_.end());
            ranked_policies_.erase(last);
        }

        std::cout << robot_name_ << ":" << generation_counter_ << " ranked_policies_:";

        for (auto const &it : ranked_policies_) {
            double fitness = it.first;
            std::cout << " " << fitness;
        }
        std::cout << std::endl;

        this->writeCurrent(0);
        this->writeLast(curr_fitness);

        generation_counter_++;
        if (generation_counter_ == RLPower::MAX_EVALUATIONS) {
            std::cout << "Finish!!!" << std::endl;
            exit(0);
        }

        // increase spline points if it is time
        if (generation_counter_ % RLPower::UPDATE_STEP == 0) {
            source_y_size++;

            // LOG code
            step_rate_ = RLPower::MAX_SPLINE_SAMPLES / source_y_size;
            std::cout << "New samplingSize_=" << source_y_size << ", and stepRate_=" << step_rate_ << std::endl;

            // current policy
            Policy policy_copy(current_policy_->size());
            for (unsigned int i = 0; i < nActuators_; i++) {
                Spline &spline = current_policy_->at(i);
                policy_copy[i] = Spline(spline.begin(), spline.end());

                spline.resize(source_y_size);
            }

            this->interpolateCubic(&policy_copy, current_policy_.get());

            //for every ranked policy
            for (auto &it : ranked_policies_) {
                PolicyPtr policy = it.second;

                for (unsigned int j = 0; j < nActuators_; j++) {
                    Spline &spline = policy->at(j);
                    policy_copy[j] = Spline(spline.begin(), spline.end());
                    spline.resize(source_y_size);
                }
                this->interpolateCubic(&policy_copy, policy.get());
            }

        }

        // Actual policy generation
        std::random_device rd;
        std::mt19937 mt(rd());
        std::normal_distribution<double> dist(0, noise_sigma_);
        noise_sigma_ *= SIGMA_DECAY_SQUARED;

        if (ranked_policies_.size() < RLPower::MAX_RANKED_POLICIES) {
            // Init random controller
            for (unsigned int i = 0; i < nActuators_; i++) {
                for (unsigned int j = 0; j < source_y_size; j++) {
                    (*current_policy_)[i][j] = dist(mt);
                }
            }
        } else {
            double total_fitness = 0;
            for (auto const &it : ranked_policies_) {
                double fitness = it.first;
                total_fitness += fitness;
            }

            // for actuator
            for (unsigned int i = 0; i < nActuators_; i++) {
                // for column
                for (unsigned int j = 0; j < source_y_size; j++) {

                    // modifier ...
                    double spline_point = 0;
                    for (auto const &it : ranked_policies_) {
                        double fitness = it.first;
                        PolicyPtr policy = it.second;

                        spline_point += ((policy->at(i)[j] - (*current_policy_)[i][j])) * fitness;
                    }
                    spline_point /= total_fitness;

                    // ... + noise + current
                    spline_point += dist(mt) + (*current_policy_)[i][j];

                    (*current_policy_)[i][j] = spline_point;
                }
            }
        }

        // cache update
        this->generateCache();
    }

    double RLPower::getFitness() {
        //Calculate fitness for current policy
        return evaluator_->fitness() / RLPower::FREQUENCY_RATE;
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

    void print_policy(RLPower::Policy *const policy) {
        for (auto policy_it = policy->begin(); policy_it != policy->end(); policy_it++) {
            std::cout << " spline size = " << policy_it->size() << " - ";
            print_spline(*policy_it);
        }
    }

    void RLPower::generateCache() {
        this->interpolateCubic(current_policy_.get(), interpolation_cache_.get());
    }

    void RLPower::generateOutput(const double time, double *output_vector) {
        if (cycle_start_time_ < 0) {
            cycle_start_time_ = time;
        }

        // get correct X value (between 0 and CYCLE_LENGTH)
        double x = time - cycle_start_time_;
        while (x >= RLPower::CYCLE_LENGTH) {
            cycle_start_time_ += RLPower::CYCLE_LENGTH;
            x = time - cycle_start_time_;
        }

        // adjust X on the cache coordinate space
        x = (x / CYCLE_LENGTH) * INTERPOLATION_CACHE_SIZE;
        // generate previous and next values
        int x_a = ((int) x) % INTERPOLATION_CACHE_SIZE;
        int x_b = (x_a + 1) % INTERPOLATION_CACHE_SIZE;

        // linear interpolation for every actuator
        for (unsigned int i = 0; i < nActuators_; i++) {
            double y_a = interpolation_cache_->at(i)[x_a];
            double y_b = interpolation_cache_->at(i)[x_b];

            output_vector[i] = y_a +
                               ((y_b - y_a) * (x - x_a) / (x_b - x_a));
        }
    }

    void RLPower::interpolateCubic(Policy *const source_y, Policy *destination_y) {
        const unsigned int source_y_size = (*source_y)[0].size();
        const unsigned int destination_y_size = (*destination_y)[0].size();

        const unsigned int N = source_y_size + 1;
        double *x = new double[N];
        double *y = new double[N];
        double *x_new = new double[destination_y_size];

        gsl_interp_accel *acc = gsl_interp_accel_alloc();
        const gsl_interp_type *t = gsl_interp_cspline_periodic;
        gsl_spline *spline = gsl_spline_alloc(t, N);

        // init x
        double step_size = CYCLE_LENGTH / source_y_size;
        for (unsigned int i = 0; i < N; i++) {
            x[i] = step_size * i;
        }

        // init x_new
        step_size = CYCLE_LENGTH / destination_y_size;
        for (unsigned int i = 0; i < destination_y_size; i++) {
            x_new[i] = step_size * i;
        }

        for (unsigned int j = 0; j < nActuators_; j++) {
            Spline &source_y_line = source_y->at(j);
            Spline &destination_y_line = destination_y->at(j);

            // init y
            // TODO use memcpy
            for (unsigned int i = 0; i < source_y_size; i++) {
                y[i] = source_y_line[i];
            }

            // make last equal to first
            y[N - 1] = y[0];

            gsl_spline_init(spline, x, y, N);

            for (unsigned int i = 0; i < destination_y_size; i++) {
                destination_y_line[i] = gsl_spline_eval(spline, x_new[i], acc);
            }
        }

        gsl_spline_free(spline);
        gsl_interp_accel_free(acc);

        delete[] x_new;
        delete[] y;
        delete[] x;
    }


    void RLPower::printCurrent() {
        for (unsigned int i = 0; i < RLPower::MAX_SPLINE_SAMPLES; i++) {
            for (unsigned int j = 0; j < nActuators_; j++) {
                std::cout << current_policy_->at(j)[i] << " ";
            }
            std::cout << std::endl;
        }
    }

    void RLPower::writeCurrent(double current_fitness) {
        std::ofstream outputFile;
        outputFile.open(robot_name_ + ".log", std::ios::app | std::ios::out | std::ios::ate);
        outputFile << "- generation: " << generation_counter_ << std::endl;
        outputFile << "  velocities:" << std::endl;
        for (auto const &it : ranked_policies_) {
            double fitness = it.first;
            outputFile << "  - " << fitness << std::endl;
        }
        outputFile.close();
    }

    void RLPower::writeLast(double fitness) {
        std::ofstream outputFile;
        outputFile.open(robot_name_ + ".policy", std::ios::app | std::ios::out | std::ios::ate);
        outputFile << "-----" << std::endl;
        outputFile << "- evaluation : " << generation_counter_ << std::endl;
        outputFile << "- velocity : " << fitness << std::endl;
        outputFile << "  - steps:" << source_y_size << std::endl;
        outputFile << "  - policy:" << std::endl;
        for (unsigned int i = 0; i < nActuators_; i++) {
            outputFile << "    - spline:" << std::endl;
            for (unsigned int j = 0; j < RLPower::MAX_SPLINE_SAMPLES; j += step_rate_) {
                outputFile << "      - " << current_policy_->at(i)[j] << std::endl;
            }
        }
        outputFile.close();
    }

} /* namespace tol */
