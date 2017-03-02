#ifndef TOL_PLUGIN_BODY_H_
#define TOL_PLUGIN_BODY_H_

#include <vector>
#include <map>
#include <string>
#include <tuple>

#include <yaml-cpp/yaml.h>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "brain/learner/cppneat/GeneticEncoding.h"

namespace tol
{

class Body
{
    struct BodyPart
    {
        int arity;
        std::string name;
        std::string type;
        int rotation;        //0 is standard, 1 is rotated to the right by 90 degrees, ...
        BodyPart *neighbours;    //the neighbours of this bodypart. neighbour in socket i is in neighbours[i]
        int coordinates[2];    //the coordinates of this bodypart
        CPPNEAT::NeuronGenePtr differential_oscillator[3];
    };
public:
    Body(std::string &yaml_path);

    ~Body();

    CPPNEAT::GeneticEncodingPtr
    get_coupled_cpg_network();

    std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>>
    get_input_output_map(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &sensors);

    CPPNEAT::GeneticEncodingPtr
    get_hyper_neat_network();

    std::map<std::string, std::tuple<int, int, int>>
    get_id_to_coordinate_map();

    //returns the coordinates of the actuators matching the order the actuators are given
    //coordinate of actuators[0] is in sorted_coordinates[0]
    std::vector<std::pair<int, int>>
    get_coordinates_sorted(const std::vector<revolve::gazebo::MotorPtr> &actuators);

    int
    getInnovNumber()
    { return innov_number + 1; };
private:
    //Body parsing
    std::map<std::string, int> part_arity;
    std::vector<BodyPart *> to_parse;
    BodyPart *core;
    std::vector<BodyPart *> delete_later;
    std::map<BodyPart *, YAML::Node> body_to_node;

    static void
    set_coordinates(int x,
                    int y,
                    BodyPart *part);

    static int
    calc_rotation(int arity,
                  int slot,
                  int parent_rotation);

    void
    make_empty(BodyPart *part);

    void
    from_yaml(BodyPart *part,
              YAML::Node &node);

    //Network
    int innov_number;
    std::vector<CPPNEAT::ConnectionGenePtr> connections;
    std::vector<CPPNEAT::NeuronGenePtr> neurons;
    std::vector<CPPNEAT::NeuronGenePtr> input_neurons;
    std::vector<CPPNEAT::NeuronGenePtr> output_neurons;
    std::map<CPPNEAT::NeuronGenePtr, std::tuple<int, int, int>> neuron_coordinates;


};
}

#endif // TOL_PLUGIN_BODY_H_
