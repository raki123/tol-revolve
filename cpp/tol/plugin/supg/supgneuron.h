#ifndef SUPGNEURON_H
#define SUPGNEURON_H

#include <vector>
#include <chrono>
#include "network/cpu/cpunetwork.h"

class SUPGNeuron
{
public:
    SUPGNeuron(NEAT::CpuNetwork *cppn, std::vector< float > coordinates, NEAT::real_t cicle_length);
    SUPGNeuron(const SUPGNeuron& other, std::vector< float > coordinates, NEAT::real_t cicle_length);
    virtual ~SUPGNeuron();

    static unsigned int GetDimensionInput(unsigned int n_inputs, unsigned int coordinates_size);
    static unsigned int GetDimensionOutput(unsigned int n_outputs);

    void reset(float global_time);
    void activate(float global_time);
    void load_sensor(size_t isensor, NEAT::real_t activation);
    void setCppn(NEAT::CpuNetwork* cppn);
    NEAT::real_t *get_outputs();

    NEAT::NetDims get_dims();

private:
    NEAT::CpuNetwork *cppn;
    NEAT::real_t start_timer;
    std::vector<float> coordinates;
    NEAT::real_t const timer_window;
    bool started_timer_flag;

    /**
     * create values from cppn (limit value from 0 to 1)
     */
    void init_timer(float global_time);
    void set_coordinates(std::vector< float > coordinates);
    void load_coordinates();
    NEAT::real_t get_timer(float global_time);

    enum Input {
        TIMER = 0,
        COORDINATE_OFFSET = 1
    };
    const unsigned int supg_internal_inputs; // 1 + coordinate.size()
    enum Output {
        OFFSET = 0,
        RESET_TRIGGER = 1
    };
    const unsigned int supg_internal_outputs; // 2
};

#endif // SUPGNEURON_H
