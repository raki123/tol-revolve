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

    /**
     * Number of evaluations before the program quits. Usefull to do long run
     * tests. If negative (default value), it will never stop.
     *
     * Takes value from env variable SUPG_MAX_EVALUATIONS.
     * Default value -1
     */
    const long MAX_EVALUATIONS; //= -1; // max number of evaluations
    /**
     * How long should an evaluation lasts (in seconds)
     *
     * Takes value from env variable SUPG_FREQUENCY_RATE
     * Default value 30 seconds
     */
    const double FREQUENCY_RATE; //= 30; // seconds
    /**
     * How long should the supg timer cicle be (in seconds)
     *
     * Takes value from env variable SUPG_CYCLE_LENGTH
     * Default value 5 seconds
     */
    const double CYCLE_LENGTH; // = 5; // seconds

    static long GetMAX_EVALUATIONSenv();
    static double GetFREQUENCY_RATEenv();
    static double GetCYCLE_LENGTHenv();
    static const char* getVARenv(const char* var_name);
};

#endif // SUPGBRAIN_H
