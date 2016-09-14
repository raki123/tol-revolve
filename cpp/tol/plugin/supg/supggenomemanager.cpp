/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2016  Matteo De Carlo <matteo.dek@covolunablu.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "supggenomemanager.h"
#include "supgneuron.h"
#include "innovgenome/innovgenome.h"

#include <iostream>

SUPGGenomeManager::SUPGGenomeManager()
{

}

static NEAT::InnovGenome *to_innov(NEAT::Genome &g) {
    return dynamic_cast<NEAT::InnovGenome *>(&g);
}

std::vector<std::unique_ptr<NEAT::Genome>> SUPGGenomeManager::create_seed_generation(size_t ngenomes,
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

        // ADD FIRST LINK ALREADY
//         for (int output_id = 0; output_id<noutputs-2; output_id++) {
//         std::cout << "link: " << node_id_input + SUPGNeuron::TIMER << " → " << node_id_output + 2 + output_id << std::endl;
        start_genome.add_link(start_genome.links,
                                NEAT::InnovLinkGene(rng.element(start_genome.traits).trait_id,
                                                    rng.prob(),
                                                    node_id_input + SUPGNeuron::TIMER,
                                                    node_id_output + 2,
                                                    false,
                                                    start_genome.get_last_gene_innovnum(),
                                                    0.0)
                                );
//         }
        // FINISHED MODIFICATION

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
