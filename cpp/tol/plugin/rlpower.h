//
// Created by Milan Jelisavcic on 28/03/16.
//

#ifndef REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H
#define REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H

#include "evaluator.h"
#include "revolve/gazebo/brain/Brain.h"

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>

namespace tol {

    class RLPower : public revolve::gazebo::Brain {

    public:
        typedef std::vector<double> Spline;
        typedef std::vector <Spline> Policy;
        typedef std::shared_ptr <Policy> PolicyPtr;

        typedef const std::shared_ptr<revolve::msgs::ModifyNeuralNetwork const> ConstModifyNeuralNetworkPtr;

        /**
         * The RLPower constructor reads out configuration file, deretmines which algorithm type to apply and
         * initialises new policy.
         * @param modelName: name of robot
         * @param brain: configuration file
         * @param evaluator: pointer to fitness evaluatior
         * @param actuators: vector list of robot's actuators
         * @param sensors: vector list of robot's sensors
         * @return pointer to the RLPower class object
         */
        RLPower(std::string modelName,
                sdf::ElementPtr brain,
                tol::EvaluatorPtr evaluator,
                std::vector <revolve::gazebo::MotorPtr> &actuators,
                std::vector <revolve::gazebo::SensorPtr> &sensors);

        virtual ~RLPower();

        /**
         * Method for updating sensors readings, actuators positions, ranked list of policies and generating new policy
         * @param actuators: vector list of robot's actuators
         * @param sensors: vector list of robot's sensors
         * @param t:
         * @param step:
         */
        virtual void update(const std::vector <revolve::gazebo::MotorPtr> &actuators,
                            const std::vector <revolve::gazebo::SensorPtr> &sensors,
                            double t,
                            double step);

    protected:
//        /**
//         * Request handler to modify the neural network
//         */
//        void modify(ConstModifyNeuralNetworkPtr &req);
//
//        boost::mutex networkMutex_; // Mutex for stepping / updating the network

        /**
         * Ranked list of used splines
         */
        class PolicySave {
        public:
            PolicyPtr policy_;
            double fitness_;

            PolicySave(double fitness, PolicyPtr &p) :
                    policy_(p),
                    fitness_(fitness) {}

            bool operator>(const PolicySave &ps) const {
                return this->fitness_ > ps.fitness_;
            }
        };

        const unsigned int MAX_EVALUATIONS = 1000; // max number of evaluations
        const unsigned int MAX_RANKED_POLICIES = 10; // max length of policies vector
        const unsigned int INTERPOLATION_CACHE_SIZE = 100; // number of data points for the interpolation cache
        const unsigned int INITIAL_SPLINE_SIZE = 3; // number of initially sampled spline points
        const unsigned int UPDATE_STEP = 100; // after # generations, it increases the number of spline points
        const double EVALUATION_RATE = 30.0; // evaluation time for each policy
        const double SIGMA_START_VALUE = 0.8; // starting value for sigma
        const double SIGMA_TAU_CORRECTION = 0.2;

        const double CYCLE_LENGTH = 5; // seconds
        const double SIGMA_DECAY_SQUARED = 0.98; // sigma decay

    private:
//        /**
//         * Transport node
//         */
//        ::gazebo::transport::NodePtr node_;
//
//        /**
//         * Network modification subscriber
//         */
//        ::gazebo::transport::SubscriberPtr alterSub_;

        /**
         * Generate new policy
         */
        void generateInitPolicy();

        /**
         * Generate cache policy
         */
        void generateCache();

        /**
         * Evaluate the current policy and generate new
         */
        void updatePolicy();

        /**
         * Generate interpolated spline based on number of sampled control points in 'source_y'
         * @param source_y: set of control points over which interpolation is generated
         * @param destination_y: set of interpolated control points (default 100 points)
         */
        void interpolateCubic(Policy *const source_y,
                              Policy *destination_y);

        /**
         * Increment number of sampling points for policy
         */
        void increaseSplinePoints();

        /**
         * Randomly select two policies and return the one with higher fitness
         * @return an iterator from 'ranked_policies_' map
         */
        std::map<double, RLPower::PolicyPtr>::iterator binarySelection();

        /**
         * Extracts the value of the current_policy in x=time using linear
         * interpolation
         *
         * Writes the output in output_vector
         */
        void generateOutput(const double time,
                            double *output_vector);

        /**
         * Retrieves fitness for the current policy
         * @return
         */
        double getFitness();

        /**
         * Writes all current splines to file
         */
        void printCurrent();

        /**
         * Writes current spline to file
         */
        void writeCurrent();

        /**
         * Writes best 10 splines to file
         */
        void writeElite();

        PolicyPtr current_policy_ = NULL; // Pointer to the current policy
        PolicyPtr interpolation_cache_ = NULL; // Pointer to the interpolated current_policy_ (default 100 points)
        EvaluatorPtr evaluator_ = NULL; // Pointer to the fitness evaluator

        unsigned int generation_counter_; // Number of current generation
        unsigned int intepolation_spline_size_; // Number of 'interpolation_cache_' sample points
        unsigned int max_ranked_policies_; // Maximal number of stored ranked policies
        unsigned int max_evaluations_; // Maximal number of evaluations
        unsigned int nActuators_; // Number of actuators
        unsigned int nSensors_; // Number of sensors
        unsigned int source_y_size; //
        unsigned int step_rate_; //
        unsigned int update_step_; // Number of evaluations after which sampling size increases

        double cycle_start_time_;
        double evaluation_rate_;
        double noise_sigma_; // Noise in the generatePolicy function
        double sigma_tau_correction_; // Tau deviation for self-adaptive sigma
        double start_eval_time_;

        std::string robot_name_; // Name of the robot
        std::string algorithm_type_; // Type of the used algorithm
        std::map<double, PolicyPtr, std::greater<double>> ranked_policies_; // Container for best ranked policies
    };

} /* namespace tol */

#endif //REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H

