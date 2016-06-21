#include "asyncneat.h"

#include "neat.h"
#include "genomemanager.h"
#include "species/speciesorganism.h"
#include <iostream>
#include <limits>
#include <stdexcept>

#define DEFAULT_RNG_SEED 1

AsyncNeat::AsyncNeat(unsigned int n_inputs, unsigned int n_outputs)
  : n_inputs(n_inputs)
  , n_outputs(n_outputs)
  , generation(1)
  , rng_seed(DEFAULT_RNG_SEED)
  , fittest(nullptr)
  , fittest_fitness(std::numeric_limits<float>().min())
{
    if (NEAT::env->genome_manager == nullptr) {
        throw std::invalid_argument("genome manager not initialized, "
        "please run AsyncNeat::Init() function to initialize the genome manager"
        " [remember also to call AsyncNeat::CleanUp() after finised using all"
        " AsyncNEAT objects]");
    }

    NEAT::rng_t rng{rng_seed};
    NEAT::rng_t rng_exp(rng.integer());
    std::vector<std::unique_ptr<NEAT::Genome>> genomes =
        NEAT::env->genome_manager->create_seed_generation(NEAT::env->pop_size,
                                                            rng_exp,
                                                            1,
                                                            n_inputs,
                                                            n_outputs,
                                                            n_inputs);
    //Spawn the Population
    population = NEAT::Population::create(rng_exp, genomes);
    refill_evaluation_queue();
}

AsyncNeat::~AsyncNeat()
{
    delete population;
}


std::shared_ptr<NeatEvaluation> AsyncNeat::getEvaluation()
{
    if (this->evaluatingQueue.empty()) {
        std::cerr << "Neat::getEvaluation() evaluation queue is empty" << std::endl;
        return nullptr;
    }

    std::shared_ptr<NeatEvaluation> new_evaluation = this->evaluatingQueue.front();
    new_evaluation->add_finished_callback([this, new_evaluation] (float fitness) {
        this->evaluatingList.remove(new_evaluation);
        this->singleEvalutionFinished(new_evaluation, fitness);
    });

    this->evaluatingQueue.pop_front();
    this->evaluatingList.push_back(new_evaluation);

    return new_evaluation;
}

void AsyncNeat::next_generation()
{
    generation++;
    population->next_generation();
    refill_evaluation_queue();
}

void AsyncNeat::refill_evaluation_queue()
{
    size_t n_organism = population->size();
    for(size_t i = 0; i < n_organism; i++) {
        evaluatingQueue.push_back(
            std::make_shared<NeatEvaluation>(
                population->get(i)
            )
        );
    }
}


void AsyncNeat::singleEvalutionFinished(std::shared_ptr< NeatEvaluation > evaluation, float fitness)
{
    if (fitness > this->fittest_fitness) {
        this->fittest = evaluation;
    }

    // need to wait for all generational evaluations to finish before creating a new generation
    // (because of spieces shared fitness)
    if (evaluatingQueue.empty() && evaluatingList.empty()) {
        next_generation();
    }

}
