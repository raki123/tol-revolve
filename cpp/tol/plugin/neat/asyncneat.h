#ifndef NEAT_H
#define NEAT_H

#include "neatevaluation.h"
#include "organism.h"
#include "population.h"
#include <list>
#include <memory>


/**
 * This is an asynchronous NEAT implemetation, it could be hard to understand
 *
 * The main loop cycle must be implemented outside (for more flexibility in real
 * time).
 *
 * To use this NEAT implemetation, get evaluations to complete using getEvaluation().
 * Then complete each evaluation calling Evaluation.finish(). If not all evaluations
 * are finished, the getEvaluation could fail.
 *
 * Multiple evaluation can be tested at the same time, is not a problem.
 *
 * It conflicts with the classical implemeation of accuNEAT since there are some
 * global objects to be used, be carefull.
 */
class AsyncNeat
{
public:
    AsyncNeat(unsigned int n_inputs, unsigned int n_outputs);
    virtual ~AsyncNeat();

    /**
     * If it returns nullptr, wait until all evaluations are finished to get
     * the next generation
     */
    std::shared_ptr<NeatEvaluation> getEvaluation();

    /**
     * to be called before any AsyncNeat object can be used
     */
    static void Init() {
        NEAT::env->genome_manager = NEAT::GenomeManager::create();
    };

    /**
     * to be called after all AsyncNeat object are not in use anymore
     */
    static void CleanUp() {
        delete NEAT::env->genome_manager;
        NEAT::env->genome_manager = nullptr;
    };

    static void SetSearchType(NEAT::GeneticSearchType type) {
        NEAT::env->search_type = type;
    }

    static void SetPopulationSize(unsigned int n) {
        NEAT::env->pop_size = n;
    }

    std::shared_ptr<NeatEvaluation> getFittest() const {
        return fittest;
    }

private:
    unsigned int n_inputs;
    unsigned int n_outputs;
    unsigned int generation;
    int rng_seed;

    NEAT::Population *population;
    std::list<std::shared_ptr<NeatEvaluation>> evaluatingList;
    std::list<std::shared_ptr<NeatEvaluation>> evaluatingQueue;

    std::shared_ptr<NeatEvaluation> fittest;
    float fittest_fitness;

    void singleEvalutionFinished(std::shared_ptr<NeatEvaluation> evaluation, float fitness);
    void next_generation();
    void refill_evaluation_queue();
};

#endif // NEAT_H
