//
// Created by Milan Jelisavcic on 28/03/16.
//

#ifndef REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H
#define REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H

#include "revolve/gazebo/brain/Brain.h"

#include <gazebo/gazebo.hh>
#include <revolve/msgs/neural_net.pb.h>

namespace tol {

    class RLPower : public revolve::gazebo::Brain {

    public:
        typedef std::vector<double> Spline;
        typedef std::vector <Spline> Policy;
        typedef std::shared_ptr <Policy> PolicyPtr;

        class Evaluator {
        public:
            virtual void start() = 0;

            virtual double fitness() = 0;
        };

        typedef std::shared_ptr <Evaluator> EvaluatorPtr;
//     typedef const std::shared_ptr<revolve::msgs::ModifyNeuralNetwork const> ConstModifyNeuralNetworkPtr;

        RLPower(std::string modelName,
                EvaluatorPtr evaluator,
                std::vector <revolve::gazebo::MotorPtr> &actuators,
                std::vector <revolve::gazebo::SensorPtr> &sensors);

        RLPower(std::string modelName,
                sdf::ElementPtr brain,
                EvaluatorPtr evaluator,
                std::vector <revolve::gazebo::MotorPtr> &actuators,
                std::vector <revolve::gazebo::SensorPtr> &sensors);

        virtual ~RLPower();

        /**
         * @param Motor list
         * @param Sensor list
         */
        virtual void update(const std::vector <revolve::gazebo::MotorPtr> &actuators,
                            const std::vector <revolve::gazebo::SensorPtr> &sensors,
                            double t,
                            double step);

    protected:

        double getFitness();

        void generatePolicy();

        /**
         * Request handler to modify the neural network
         */
//     void modify(ConstModifyNeuralNetworkPtr &req);

        // Mutex for stepping / updating the network
        //boost::mutex networkMutex_;

        /**
         * Holder of current policy
         */
        PolicyPtr current_policy_;

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

        std::map<double, PolicyPtr, std::greater<double>> ranked_policies_;

        unsigned int nActuators_;
        unsigned int nSensors_;

        double start_eval_time_;
        double max_evaluations_;

        unsigned int generation_counter_;
        //ignition::math::Pose3d currentPosition_;
        //xignition::math::Pose3d previousPosition_;
        EvaluatorPtr evaluator_;

        /**
         * Rank and matrix of currently used splines.
         * Same as @currentSpline_
         */
        unsigned int source_y_size;
        unsigned int step_rate_;
        unsigned int intepolation_spline_size_;
        unsigned int max_ranked_policies_;

        /**
         * Noise in the generatePolicy function
         */
        double noise_sigma_;

    private:
        void interpolateCubic(Policy *const source_y, Policy *destination_y);

        void generateCache();

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

        /**
         * Extracts the value of the current_policy in x=time using linear
         * interpolation
         *
         * Writes the output in output_vector
         */
        void generateOutput(const double time, double *output_vector);

        PolicyPtr interpolation_cache_;

        const unsigned int MAX_EVALUATIONS = 1000; // max number of evaluations
        const unsigned int MAX_RANKED_POLICIES = 10; // max length of policies vector
        const unsigned int MAX_SPLINE_SAMPLES = 100; // interpolation cache size
        const unsigned int UPDATE_STEP = 100; // after # generations, it increases the number of spline points
        const unsigned int INITIAL_SPLINE_SIZE = 3;
        const unsigned int FREQUENCY_RATE = 30; // seconds
        const double CYCLE_LENGTH = 5; // seconds
        const double SIGMA_START_VALUE = 0.8; // starting value for sigma
        const double SIGMA_DECAY_SQUARED = 0.98; // sigma decay
        const unsigned int INTERPOLATION_CACHE_SIZE = 100; // number of data points for the interpolation cache

        double cycle_start_time_;
        std::string robot_name_;

        /**
         * Transport node
         */
//     ::gazebo::transport::NodePtr node_;

        /**
         * Network modification subscriber
         */
//     ::gazebo::transport::SubscriberPtr alterSub_;
    };

} /* namespace tol */

#endif //REVOLVE_GAZEBO_BRAIN_REINFORCEDLEARNING_H

