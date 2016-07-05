#ifndef SUPGBRAIN_H
#define SUPGBRAIN_H

#include "revolve/gazebo/brain/Brain.h"
#include "supg/supgneuron.h"
#include "neat/asyncneat.h"

class SUPGBrain : public revolve::gazebo::Brain
{
public:
    class Evaluator {
    public:
        virtual void start() = 0;
        virtual double fitness() = 0;
    };

    typedef std::shared_ptr< Evaluator > EvaluatorPtr;
    SUPGBrain(EvaluatorPtr evaluator, std::vector< std::vector< float > > neuron_coordinates,  std::vector< revolve::gazebo::MotorPtr >& motors, const std::vector< revolve::gazebo::SensorPtr >& sensors);
    ~SUPGBrain();

    virtual void update(const std::vector< revolve::gazebo::MotorPtr >& motors, const std::vector< revolve::gazebo::SensorPtr >& sensors, double t, double step) override;

private:
    double getFitness();
    void nextBrain();

    std::unique_ptr<AsyncNeat> neat;
    EvaluatorPtr evaluator;
    double start_eval_time;
    unsigned int generation_counter;
    std::shared_ptr< NeatEvaluation > current_evalaution;

    unsigned int n_inputs, n_outputs;
    std::vector< std::vector< float > > neuron_coordinates;
    std::vector< std::unique_ptr< SUPGNeuron > > neurons;

    const unsigned int MAX_EVALUATIONS = 25000; // max number of evaluations
    const double FREQUENCY_RATE = 30; // seconds
    const double CYCLE_LENGTH = 5; // seconds
};

#endif // SUPGBRAIN_H
