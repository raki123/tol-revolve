#include "neatevaluation.h"

NeatEvaluation::NeatEvaluation(NEAT::Organism *organism)
    : organism(organism)
    , finished_callback(nullptr)
    , fitness(-1)
{}

void NeatEvaluation::finish(float fitness)
{
    organism->eval.fitness = fitness;

    if (!finished_callback) {
        std::cerr << "NeatEvaluation::finish() error, finish callback not setted!" << std::endl;
        return;
    }

    finished_callback(fitness);
}
