#include "testmultinnspeciesneat.h"

#include "../asyncneat.h"
#include <limits>
#include "network/cpu/cpunetwork.h"
#include <vector>
#include <cmath>
#include <iostream>

TestMultiNNSpeciesNeat::TestMultiNNSpeciesNeat()
{

}

TestMultiNNSpeciesNeat::~TestMultiNNSpeciesNeat()
{

}


bool TestMultiNNSpeciesNeat::test()
{
    if (!testXOR())
        return false;

    return true;
}

// 0 0 -> 0
// 0 1 -> 1
// 1 0 -> 1
// 1 1 -> 0
bool TestMultiNNSpeciesNeat::testXOR()
{
    AsyncNeat::Init();
    AsyncNeat::SetSearchType(NEAT::GeneticSearchType::BLENDED);
    AsyncNeat::SetPopulationType(NEAT::PopulationType::MULTI_NN_SPECIES);
    AsyncNeat::SetPopulationSize(10);
    AsyncNeat neat(2, 1, 1);
    float success_margin_error = 0.0001;

    bool success = false;
    float min_error = std::numeric_limits< float >().max();
    std::vector<float>         inputs0 = { 0, 0, 1, 1};
    std::vector<float>         inputs1 = { 0, 1, 0, 1};
    std::vector<float> expectedOutputs = { 0, 1, 1, 0};

    int gen;
    for (gen = 1; gen < MAX_EVALUATIONS; gen++) {
        std::shared_ptr< NeatEvaluation > eval = neat.getEvaluation();
        const NEAT::Organism* organism = eval->getOrganism();
        NEAT::CpuNetwork* net = reinterpret_cast< NEAT::CpuNetwork*> (organism->net.get());

        float error = 0;
//         std::cout << std::endl;
        for(unsigned int test=0; test<inputs0.size(); test++) {
            net->load_sensor(0, inputs0[test]);
            net->load_sensor(1, inputs1[test]);

            net->activate(1);
            NEAT::real_t* outputs = net->get_outputs();
            error += std::abs(outputs[0] - expectedOutputs[test]);

//             std::cout << "#" << gen << "# " << inputs0[test] << ' ' << inputs1[test] << " -> " << outputs[0] << std::endl;
        }

        if (min_error > error) {
            min_error = error;
        }

        eval->finish(1/error);

        if (min_error < success_margin_error) {
            std::cout << "\nAfter " << gen << " tries, a successful organism was found with an error of " << min_error << std::endl;
            std::cout << "The organism fitness is " << neat.getFittest()->getOrganism()->eval.fitness << std::endl;
            success = true;
            break;
        }
    }

    neat.CleanUp();
    return success;
}



int main(int argc, char* argv[]) {
    TestMultiNNSpeciesNeat t;
    return t.test() ? 0 : 1;
}
