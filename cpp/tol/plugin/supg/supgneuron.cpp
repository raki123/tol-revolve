#include "../supg/supgneuron.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <sstream>

#define NCYCLES 1

SUPGNeuron::SUPGNeuron(NEAT::CpuNetwork* cppn, std::vector< float > coordinates)
  : cppn(cppn)
  , start_timer(-1)
{
    set_coordinates(coordinates);
    NEAT::NetDims dims = cppn->get_dims();
    unsigned int n_input = dims.nnodes.input;
    unsigned int n_output = dims.nnodes.output;
    if (n_input != 1 + coordinates.size()) {
        std::stringstream ss;
        ss << "SUPGNeuron::SUPGNeuron() Invalid network input size or coordinate vector size (cppn.input_size == " << n_input << " | coordinates.size() == " << coordinates.size() << " )";
        throw std::invalid_argument(ss.str());
    }
    if (n_output != 2) {
        std::stringstream ss;
        ss << "SUPGNeuron::SUPGNeuron() Invalid network output size (cppn.output_size == " << n_output << " )";
        throw std::invalid_argument(ss.str());
    }
}

SUPGNeuron::SUPGNeuron(const SUPGNeuron& other, std::vector< float > coordinates)
  : SUPGNeuron(other.cppn, coordinates)
{}

void SUPGNeuron::reset(float global_time)
{
    start_timer = global_time;
}

//create value from cppn (limit value from 0 to 1)
void SUPGNeuron::init_timer(float global_time)
{
    cppn->load_sensor(Input::TIMER, 0);
    load_coordinates();

    cppn->activate(NCYCLES);

    NEAT::real_t* outputs = cppn->get_outputs();
    NEAT::real_t offset = outputs[Output::OFFSET];

    if (offset < 0 || offset > 1) {
        offset = 0;
    }

    reset(global_time);
    start_timer += offset;
}

void SUPGNeuron::set_coordinates(std::vector< float > coordinates)
{
    this->coordinates = coordinates;
}

void SUPGNeuron::load_coordinates()
{
    for(unsigned int i=0; i<coordinates.size(); i++) {
        cppn->load_sensor(
            Input::COORDINATE_OFFSET + i,
            coordinates[i]
        );
    }
}


NEAT::real_t SUPGNeuron::get_timer(float global_time)
{
    if (start_timer < 0) {
        init_timer(global_time);
    }

    NEAT::real_t timer = global_time - start_timer;

    // this difference should never be negative
    if (timer < 0) {
        std::cerr << "SUPGNeuron::get_timer() error, negative timer value" << std::endl;
        return 0;
    }

    // reset timer if value one was passed
    if (timer > 1) {
        NEAT::real_t int_part;
        timer = std::modf(timer, &int_part);
        start_timer += int_part;
    }

    return timer;
}


float SUPGNeuron::update(float global_time)
{
    NEAT::real_t timer = get_timer(global_time);

    cppn->load_sensor(Input::TIMER, timer);
    load_coordinates();
    cppn->activate(NCYCLES);
    NEAT::real_t* outputs = cppn->get_outputs();
    return outputs[Output::SUPG];
}

