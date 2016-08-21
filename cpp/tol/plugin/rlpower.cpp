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

    RLPower::RLPower(std::string modelName,
                     sdf::ElementPtr brain,
                     EvaluatorPtr evaluator,
                     std::vector <revolve::gazebo::MotorPtr> &actuators,
                     std::vector <revolve::gazebo::SensorPtr> &sensors) :
            cycle_start_time_(-1),
            evaluator_(evaluator),
            generation_counter_(0),
            nActuators_(actuators.size()),
            nSensors_(sensors.size()),
            start_eval_time_(0),
            robot_name_(modelName) {

//        // Create transport node
//        node_.reset(new ::gazebo::transport::Node());
//        node_->Init();
//
//        // Listen to network modification requests
//        alterSub_ = node_->Subscribe("~/" + modelName + "/modify_neural_network",
//                                     &RLPower::modify, this);

        // Read out brain configuration attributes
        algorithm_type_ = brain->HasAttribute("type") ? brain->GetAttribute("type")->GetAsString() : "A";
        std::cout << std::endl <<"Initialising RLPower, type " << algorithm_type_ << std::endl << std::endl;

        evaluation_rate_ = brain->HasAttribute("evaluation_rate") ?
                           std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
                           RLPower::EVALUATION_RATE;
        intepolation_spline_size_ = brain->HasAttribute("intepolation_spline_size") ?
                                    std::stoul(brain->GetAttribute("intepolation_spline_size")->GetAsString()) :
                                    RLPower::INTERPOLATION_CACHE_SIZE;
        max_evaluations_ = brain->HasAttribute("max_evaluations") ?
                           std::stoul(brain->GetAttribute("max_evaluations")->GetAsString()) :
                           RLPower::MAX_EVALUATIONS;
        max_ranked_policies_ = brain->HasAttribute("max_ranked_policies") ?
                               std::stoul(brain->GetAttribute("max_ranked_policies")->GetAsString()) :
                               RLPower::MAX_RANKED_POLICIES;
        noise_sigma_ = brain->HasAttribute("init_sigma") ?
                       std::stod(brain->GetAttribute("init_sigma")->GetAsString()) :
                       RLPower::SIGMA_START_VALUE;
        sigma_tau_deviation_ = brain->HasAttribute("sigma_tau_deviation") ?
                               std::stod(brain->GetAttribute("sigma_tau_deviation")->GetAsString()) :
                               RLPower::SIGMA_TAU_DEVIATION;
        source_y_size = brain->HasAttribute("init_spline_size") ?
                        std::stoul(brain->GetAttribute("init_spline_size")->GetAsString()) :
                        RLPower::INITIAL_SPLINE_SIZE;
        update_step_ = brain->HasAttribute("update_step") ?
                       std::stoul(brain->GetAttribute("update_step")->GetAsString()) :
                       RLPower::UPDATE_STEP;
        step_rate_ = intepolation_spline_size_ / source_y_size;

        // Generate first random policy
        this->generateInitPolicy();

        // Start the evaluator
        evaluator_->start();
    }

    RLPower::~RLPower() {
        // `boost::shared_ptr< Policy >` should take care of memory management for us
    }

    void RLPower::update(const std::vector <revolve::gazebo::MotorPtr> &actuators,
                         const std::vector <revolve::gazebo::SensorPtr> &sensors,
                         double t,
                         double step) {
//        boost::mutex::scoped_lock lock(networkMutex_);

        // Evaluate policy on certain time limit
        if ((t - start_eval_time_) > evaluation_rate_ && generation_counter_ < max_evaluations_) {
            this->updatePolicy();
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

    void RLPower::generateInitPolicy() {
        std::random_device rd;
        std::mt19937 mt(rd());
        std::normal_distribution<double> dist(0, this->noise_sigma_);

        // Init first random controller
        if (!current_policy_)
            current_policy_ = std::make_shared<Policy>(nActuators_);

        for (unsigned int i = 0; i < nActuators_; i++) {
            Spline spline(source_y_size);
            for (unsigned int j = 0; j < source_y_size; j++) {
                spline[j] = dist(mt);
            }
            current_policy_->at(i) = spline;
        }

        // Init of empty cache
        if (!interpolation_cache_)
            interpolation_cache_ = std::make_shared<Policy>(nActuators_);

        for (unsigned int i = 0; i < nActuators_; i++) {
            interpolation_cache_->at(i).resize(intepolation_spline_size_, 0);
        }

        this->generateCache();
    }

    void RLPower::generateCache() {
        this->interpolateCubic(current_policy_.get(), interpolation_cache_.get());
    }

    void RLPower::updatePolicy() {
        // Calculate fitness for current policy
        double curr_fitness = this->getFitness();

        // Insert ranked policy in list
        PolicyPtr policy_copy = std::make_shared<Policy>(nActuators_);
        for (unsigned int i = 0; i < nActuators_; i++) {
            Spline &spline = current_policy_->at(i);
            policy_copy->at(i) = Spline(spline.begin(), spline.end());

            spline.resize(source_y_size);
        }
        ranked_policies_.insert({curr_fitness, policy_copy});

        // Remove worst policies
        while (ranked_policies_.size() > max_ranked_policies_) {
            auto last = std::prev(ranked_policies_.end());
            ranked_policies_.erase(last);
        }

        // Print-out current status to the terminal
        std::cout << robot_name_ << ":" << generation_counter_ << " ranked_policies_:";
        for (auto const &it : ranked_policies_) {
            double fitness = it.first;
            std::cout << " " << fitness;
        }
        std::cout << std::endl;

        // Write fitness and genomes log to output files
        this->writeCurrent();
        this->writeElite();

        // Update generation counter and check is it finished
        generation_counter_++;
        if (generation_counter_ == max_evaluations_) {
            std::cout << "Finish!!!" << std::endl;
            std::exit(0);
        }

        // Increase spline points if it is a time
        if (generation_counter_ % update_step_ == 0)
            this->increaseSplinePoints();

        /// Actual policy generation

        /// Determine which mutation operator to use
        /// Default, for algorithms A and B, is used standard normal distribution with decaying sigma
        /// For algorithms C and D, is used normal distribution with self-adaptive sigma
        std::random_device rd;
        std::mt19937 mt(rd());

        if (algorithm_type_ == "C" || algorithm_type_ == "D") {
            // uncorrelated mutation with one step size
            std::mt19937 sigma_mt(rd());
            std::normal_distribution<double> sigma_dist(0, sigma_tau_deviation_);
            noise_sigma_ = noise_sigma_ * exp(sigma_dist(sigma_mt));
        } else {
            noise_sigma_ *= SIGMA_DECAY_SQUARED;
        }
        std::normal_distribution<double> dist(0, noise_sigma_);

        /// Determine which selection operator to use
        ///
        if (ranked_policies_.size() < max_ranked_policies_) {
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

    void RLPower::interpolateCubic(Policy *const source_y,
                                   Policy *destination_y) {
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

    void RLPower::increaseSplinePoints() {
        source_y_size++;

        // LOG code
        step_rate_ = intepolation_spline_size_ / source_y_size;
        std::cout << "New samplingSize_=" << source_y_size << ", and stepRate_=" << step_rate_ << std::endl;

        // Copy current policy for resizing
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

    void RLPower::generateOutput(const double time,
                                 double *output_vector) {
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
        x = (x / CYCLE_LENGTH) * intepolation_spline_size_;
        // generate previous and next values
        int x_a = ((int) x) % intepolation_spline_size_;
        int x_b = (x_a + 1) % intepolation_spline_size_;

        // linear interpolation for every actuator
        for (unsigned int i = 0; i < nActuators_; i++) {
            double y_a = interpolation_cache_->at(i)[x_a];
            double y_b = interpolation_cache_->at(i)[x_b];

            output_vector[i] = y_a +
                               ((y_b - y_a) * (x - x_a) / (x_b - x_a));
        }
    }

    double RLPower::getFitness() {
        //Calculate fitness for current policy
        return evaluator_->fitness();
    }

    void RLPower::printCurrent() {
        for (unsigned int i = 0; i < intepolation_spline_size_; i++) {
            for (unsigned int j = 0; j < nActuators_; j++) {
                std::cout << current_policy_->at(j)[i] << " ";
            }
            std::cout << std::endl;
        }
    }

    void RLPower::writeCurrent() {
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

    void RLPower::writeElite() {
        std::ofstream outputFile;
        outputFile.open(robot_name_ + ".policy", std::ios::app | std::ios::out | std::ios::ate);
        outputFile << "- evaluation: " << generation_counter_ << std::endl;
        outputFile << "  steps: " << source_y_size << std::endl;
        outputFile << "  population:" << std::endl;
        for (auto const &it : ranked_policies_) {
            double fitness = it.first;
            PolicyPtr policy = it.second;
            outputFile << "   - velocity: " << fitness << std::endl;
            outputFile << "     policy:" << std::endl;
            for (unsigned int i = 0; i < policy->size(); i++) {
                Spline &spline = policy->at(i);
                for (unsigned int j = 0; j < spline.size(); j++) {
                    outputFile << "      - " << spline.at(j) << std::endl;
                }
            }
        }
        outputFile.close();
    }

} /* namespace tol */
