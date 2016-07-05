#ifndef MULTIINNOVGENOME_H
#define MULTIINNOVGENOME_H

#include "genome.h"
#include "innovgenome/innovgenome.h"
#include <memory>
#include <list>
#include <vector>

namespace NEAT {

class MultiInnovGenome : public Genome
{
public:
    MultiInnovGenome(std::list<std::unique_ptr<InnovGenome>> &genome_list);
    virtual ~MultiInnovGenome() override;

    virtual Genome &operator=(const Genome &other) override;

    virtual void init_phenotype(class Network &net) override;

    virtual void print(std::ostream &out) override;
    virtual void verify() override;

    virtual Stats get_stats() override;

private:
    std::vector<std::unique_ptr< InnovGenome >> *innov_genome_list;
};

}

#endif // MULTIINNOVGENOME_H
