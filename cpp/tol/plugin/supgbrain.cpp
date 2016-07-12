#include "supgbrain.h"
#include "revolve/gazebo/sensors/VirtualSensor.h"
#include "revolve/gazebo/motors/Motor.h"
#include "neat/asyncneat.h"
#include <sstream>
#include <stdexcept>
#include <limits>
#include <iomanip>
#include <string>
#include <ctime>

const char* SUPGBrain::getVARenv(const char* var_name)
{
    const char* env_p = std::getenv(var_name);
    if(env_p) {
        std::cout << "ENV " << var_name << " is: " << env_p << std::endl;
    } else {
        std::cout << "ENV " << var_name << " not found, using default value: ";
    }
    return env_p;
}

long SUPGBrain::GetMAX_EVALUATIONSenv()
{
    if(const char* env_p = SUPGBrain::getVARenv("SUPG_MAX_EVALUATIONS")) {
        //TODO catch exception
        long value = std::stol(env_p);
        return value;
    }
    std::cout << -1 << std::endl;
    return -1;
}


double SUPGBrain::GetFREQUENCY_RATEenv()
{
    if(const char* env_p = SUPGBrain::getVARenv("SUPG_FREQUENCY_RATE")) {
        //TODO catch exception
        double value = std::stod(env_p);
        return value;
    }
    std::cout << 30 << std::endl;
    return 30;
}

double SUPGBrain::GetCYCLE_LENGTHenv()
{
    if(const char* env_p = SUPGBrain::getVARenv("SUPG_CYCLE_LENGTH")) {
        //TODO catch exception
        double value = std::stod(env_p);
        return value;
    }
    std::cout << 5 << std::endl;
    return 5;
}


SUPGBrain::SUPGBrain(EvaluatorPtr evaluator, std::vector< std::vector< float > > neuron_coordinates, std::vector< revolve::gazebo::MotorPtr >& motors, const std::vector< revolve::gazebo::SensorPtr >& sensors)
  : evaluator(evaluator)
  , start_eval_time(std::numeric_limits< double >::lowest())
  , generation_counter(0)
  , neuron_coordinates(neuron_coordinates)
  , MAX_EVALUATIONS(GetMAX_EVALUATIONSenv())
  , FREQUENCY_RATE(GetFREQUENCY_RATEenv())
  , CYCLE_LENGTH(GetCYCLE_LENGTHenv())
{
    if (motors.size() != neuron_coordinates.size()) {
        std::stringstream ss;
        ss << "motor size [" << motors.size() << "] and neuron coordinates size [" << neuron_coordinates.size() << "] are different!";
        throw std::invalid_argument( ss.str());
    }

    unsigned int p = 0;
    std::cout<<"sensor->sensorId()"<<std::endl;
    for (auto sensor : sensors) {
        std::cout << "sensor: " << sensor->sensorId() << "(inputs: " << sensor->inputs() << ")" << std::endl;
        p += sensor->inputs();
    }
    std::cout<<"END sensor->sensorId()"<<std::endl;
    n_inputs = p;

    p = 0;
    for (auto motor : motors) {
        p += motor->outputs();
    }
    n_outputs = p;

    std::unique_ptr< AsyncNeat > neat(new AsyncNeat(
        SUPGNeuron::GetDimensionInput(n_inputs, neuron_coordinates[0].size()),
        SUPGNeuron::GetDimensionOutput(n_outputs),
        std::time(0)
    ));
    this->neat = std::move(neat);
}

SUPGBrain::~SUPGBrain()
{

}


void SUPGBrain::update(const std::vector< revolve::gazebo::MotorPtr >& motors, const std::vector< revolve::gazebo::SensorPtr >& sensors, double t, double step)
{

    // Evaluate policy on certain time limit
    if ((t-start_eval_time) > SUPGBrain::FREQUENCY_RATE) {

        // check if to stop the experiment. Negative value for MAX_EVALUATIONS will never stop the experiment
        if (SUPGBrain::MAX_EVALUATIONS > 0 && generation_counter > SUPGBrain::MAX_EVALUATIONS) {
            std::cout << "Max Evaluations (" << SUPGBrain::MAX_EVALUATIONS << ") reached. stopping now." << std::endl;
            std::exit(0);
        }
        generation_counter++;
        std::cout << "################# EVALUATING NEW BRAIN !!!!!!!!!!!!!!!!!!!!!!!!! (generation " << generation_counter << " )" << std::endl;
        this->nextBrain();
        start_eval_time = t;
        evaluator->start();
    }

    assert(n_outputs == motors.size());

    // Read sensor data and feed the neural network
    double *inputs = new double[n_inputs];
    unsigned int p = 0;
    for (auto sensor : sensors) {
        sensor->read(&inputs[p]);
        p += sensor->inputs();
    }
    assert(p == n_inputs);

    // load sensors
    for (unsigned int i = 0; i < n_inputs; i++) {
        neurons[0]->load_sensor(i, inputs[i]);
    }

    // Activate network and save results
//     std::cout << "brain update:" << std::endl;
    double *outputs = new double[n_outputs];
    for (unsigned int i = 0; i < neurons.size(); i++) {
//         std::cout << "neuron["<<i<<"]: "<<"("<<neurons[i]->coordinates[0]<<","<<neurons[i]->coordinates[1]<<")" << std::endl;
        neurons[i]->activate(t);
        outputs[i] = neurons[i]->get_outputs()[0];
    }

    // send signals to motors
    p = 0;
//     std::cout<<"motors: [";
    for (auto motor: motors) {
        motor->update(&outputs[p], step);
        p += motor->outputs();
//         std::cout << motor->motorId() << " ";
    }
//     std::cout << "]" << std::endl;
    assert(p == n_outputs);

//     std::cout<<"outputs: [";// << std::setprecision(0);
//     for (int i=0; i<p; i++) {
//         std::cout << (outputs[i] > 0 ? '+' : '-');
//         if (i==p-1)
//             std::cout << "]" << std::endl;
//         else
//             std::cout << ", ";
//     }

    delete inputs;
}

double SUPGBrain::getFitness()
{
    //Calculate fitness for current policy
    double fitness = evaluator->fitness();
    std::cout << "Evaluating gait, fitness = " << fitness << std::endl;
    return fitness;
}

void SUPGBrain::nextBrain()
{
    bool init_supgs;
    int how_many_neurons;
    if (!current_evalaution) {
        // first evaluation
        init_supgs = true;
        how_many_neurons = neuron_coordinates.size();
    } else {
        // normal change of evaluation
        init_supgs = false;
        how_many_neurons = neurons.size();
        current_evalaution->finish(getFitness());
    }

    current_evalaution = neat->getEvaluation();
    NEAT::CpuNetwork *cppn = reinterpret_cast< NEAT::CpuNetwork* > (
        current_evalaution->getOrganism()->net.get()
    );

    for (unsigned int i=0; i< how_many_neurons; i++) {
        if (init_supgs)
            neurons.push_back(std::unique_ptr<SUPGNeuron>(new SUPGNeuron(cppn, neuron_coordinates[i], CYCLE_LENGTH)));
        else
            neurons[i]->setCppn(cppn);
    }
}
