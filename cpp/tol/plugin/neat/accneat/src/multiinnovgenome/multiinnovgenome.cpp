#include "multiinnovgenome.h"

using namespace NEAT;

MultiInnovGenome::MultiInnovGenome(std::__cxx11::list< std::unique_ptr< InnovGenome > >& genome_list)
{
    innov_genome_list = new std::vector<std::unique_ptr<InnovGenome>>();

     for (auto g = genome_list.begin(); g != genome_list.end(); g++) {
         innov_genome_list->push_back(std::move(*g));
     }
     innov_genome_list->shrink_to_fit();
}

MultiInnovGenome::~MultiInnovGenome()
{
    delete innov_genome_list;
}


Genome& MultiInnovGenome::operator=(const Genome& other)
{
    return *this = dynamic_cast<const MultiInnovGenome &>(other);
}

void MultiInnovGenome::verify()
{
#ifdef NDEBUG
    return;
#else
    for (auto g = innov_genome_list->begin(); g != innov_genome_list->end(); g++) {
        (*g)->verify();
    }
#endif
}

Genome::Stats MultiInnovGenome::get_stats() {
    int nodes_size = 0,
        links_size = 0;
    for (auto g = innov_genome_list->begin(); g != innov_genome_list->end(); g++) {
        nodes_size += (*g)->nodes.size();
        links_size += (*g)->links.size();
    }

    return {nodes_size, links_size};
}

void MultiInnovGenome::print(std::ostream& out)
{
    for (auto g = innov_genome_list->begin(); g != innov_genome_list->end(); g++) {
        (*g)->print(out);
    }
}

void MultiInnovGenome::init_phenotype(Network& net)
{
    throw std::invalid_argument("init_phenotype not supported like this");
}



