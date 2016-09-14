#include "testcustomgenomemanager.h"

#include "../asyncneat.h"
#include <limits>
#include "network/cpu/cpunetwork.h"
#include "innovgenome/innovgenomemanager.h"
#include "innovgenome/innovgenome.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>

TestCustomGenomeManager::TestCustomGenomeManager()
{

}

TestCustomGenomeManager::~TestCustomGenomeManager()
{

}

bool TestCustomGenomeManager::test()
{
    if (!testXOR())
        return false;

    return true;
}

static NEAT::InnovGenome *to_innov(NEAT::Genome &g) {
    return dynamic_cast<NEAT::InnovGenome *>(&g);
}

class TestGenomeManager : public NEAT::InnovGenomeManager {
public:
    TestGenomeManager() {

    }

    virtual std::vector<std::unique_ptr<NEAT::Genome>> create_seed_generation(size_t ngenomes,
                                                                      NEAT::rng_t rng,
                                                                      size_t ntraits,
                                                                      size_t ninputs,
                                                                      size_t noutputs,
                                                                      size_t nhidden)
    {
        NEAT::InnovGenome start_genome(rng,
                                ntraits,
                                ninputs,
                                noutputs,
                                nhidden);


        const int node_id_bias = 1;
        const int node_id_input = node_id_bias + 1;
        const int node_id_output = node_id_input + ninputs;
        const int node_id_hidden = node_id_output + noutputs;

        start_genome.add_link(start_genome.links,
                            NEAT::InnovLinkGene(rng.element(start_genome.traits).trait_id,
                                                rng.prob(),
                                                node_id_input + 0,
                                                node_id_output + 0,
                                                false,
                                                start_genome.get_last_gene_innovnum(),
                                                0.0)
                            );

        std::vector<std::unique_ptr<NEAT::Genome>> genomes;
        {
            NEAT::rng_t _rng = rng;
            for(int i = 0; i < NEAT::env->pop_size; i++) {
                NEAT::InnovGenome *g = new NEAT::InnovGenome();
                start_genome.duplicate_into(g);
                g->rng.seed(_rng.integer());
                g->mutate_link_weights(1.0,1.0,NEAT::COLDGAUSSIAN);
                g->randomize_traits();

                genomes.emplace_back(std::unique_ptr<NEAT::Genome>(g));
            }
        }

        {
            NEAT::InnovGenome *g = to_innov(*genomes.back());

            //Keep a record of the innovation and node number we are on
            innovations.init(g->get_last_node_id(),
                            g->get_last_gene_innovnum());
        }

        return genomes;
    }
};

// 0 0 -> 0
// 0 1 -> 1
// 1 0 -> 1
// 1 1 -> 0
bool TestCustomGenomeManager::testXOR()
{
    AsyncNeat::Init(std::unique_ptr<NEAT::GenomeManager>(new TestGenomeManager()));
    AsyncNeat::SetSearchType(NEAT::GeneticSearchType::BLENDED);
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
    TestCustomGenomeManager t;
    return t.test() ? 0 : 1;
}
