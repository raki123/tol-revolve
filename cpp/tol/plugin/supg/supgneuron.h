#ifndef SUPGNEURON_H
#define SUPGNEURON_H

#include <vector>
#include <chrono>
#include "network/cpu/cpunetwork.h"

class SUPGNeuron
{
public:
    SUPGNeuron(NEAT::CpuNetwork *cppn, std::vector< float > coordinates);
    SUPGNeuron(const SUPGNeuron& other, std::vector< float > coordinates);
    virtual ~SUPGNeuron();

    void reset(float global_time);
    float update(float global_time);

private:
    NEAT::CpuNetwork *cppn;
    NEAT::real_t start_timer;
    std::vector<float> coordinates;

    /**
     * create values from cppn (limit value from 0 to 1)
     */
    void init_timer(float global_time);
    void set_coordinates(std::vector< float > coordinates);
    void load_coordinates();
    NEAT::real_t get_timer(float global_time);

    enum Output {
        SUPG = 0,
        OFFSET  = 1
    };
    enum Input {
        TIMER = 0,
        COORDINATE_OFFSET = 1
    };
};

#endif // SUPGNEURON_H
