#include "Body.h"

#include <iostream>

namespace tol {

Body::Body(std::string &yaml_path)
        :
        innov_number(0)
{
  part_arity["Core"] = 4;
  part_arity["CoreWithMics"] = 4;
  part_arity["FixedBrick"] = 4;
  part_arity["ActiveHinge"] = 2;
  part_arity["Hinge"] = 2;
  part_arity["ParametricBarJoint"] = 2;

  YAML::Node policy_file = YAML::LoadFile(yaml_path);

  if (policy_file.IsNull()) {
    std::cout << "Failed to load the policy file." << std::endl;
    return;
  }

  //read body structure from yaml file
  //includes adding the rotation of the parts and their coordinates
  core = new BodyPart();
  to_parse.push_back(core);
  body_to_node[core] = policy_file["body"];
  make_empty(core);
  core->rotation = 0;
  set_coordinates(0,
                  0,
                  core);
  BodyPart *cur_part;
  while (!to_parse.empty()) {
    cur_part = to_parse.back();
    to_parse.pop_back();
    from_yaml(cur_part,
              body_to_node[cur_part]);
  }

  //couple neighbouring differential oscillators
  //here all oscillators must already be initialised
  to_parse.push_back(core);
  while (!to_parse.empty()) {
    cur_part = to_parse.back();
    to_parse.pop_back();
    if (cur_part->type != "ActiveHinge") {
      bool neighbour_is_motor[4];
      for (int i = 0; i < 4; i++) {
        neighbour_is_motor[i] = cur_part->neighbours[i].type == "ActiveHinge";

      }
      //add connections between each of the first neurons of an oscillator
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          if (neighbour_is_motor[i] &&
              neighbour_is_motor[j]    //no need to add both connections other one will be added for exchanged i,j
              && i != j)                    //dont add selfreferential connection
          {
            CPPNEAT::ConnectionGenePtr first_to_first(new CPPNEAT::ConnectionGene(cur_part->neighbours[i].differential_oscillator[0]->getInnovNumber(),
                                                                                  cur_part->neighbours[j].differential_oscillator[0]->getInnovNumber(),
                                                                                  0,
                                                                                  ++innov_number,
                                                                                  true));
            connections.push_back(first_to_first);
          }
        }
      }
    }

    //add neighbours to queue
    //watch out for parent
    if (cur_part == core) {
      if (cur_part->neighbours[0].name != "empty") {
        to_parse.push_back(&(cur_part->neighbours[0]));
      }
    }
    for (int i = 1; i < 4; i++) {
      if (cur_part->neighbours[i].name != "empty") {
        to_parse.push_back(&(cur_part->neighbours[i]));
      }
    }
  }

  //add input neurons and add connections between them and neurons on the core
  for (int i = 0; i < 6; i++) {
    //add neuron
    std::map<std::string, double> empty;

    CPPNEAT::NeuronPtr neuron(new CPPNEAT::Neuron(core->name + "-in-" + std::to_string(i),
                                                  CPPNEAT::Neuron::INPUT_LAYER,
                                                  CPPNEAT::Neuron::INPUT,
                                                  empty));
    CPPNEAT::NeuronGenePtr neuron_gene(new CPPNEAT::NeuronGene(neuron,
                                                               ++innov_number,
                                                               true));
    neurons.push_back(neuron_gene);
    input_neurons.push_back(neuron_gene);
    std::tuple<int, int, int> coord(0,
                                    0,
                                    0);
    neuron_coordinates[neuron_gene] = coord;

    //connect to neighbouring oscillators
    for (int i = 0; i < 4; i++) {
      if (core->neighbours[i].type == "ActiveHinge") {
        CPPNEAT::ConnectionGenePtr input_to_first(new CPPNEAT::ConnectionGene(core->neighbours[i].differential_oscillator[0]->getInnovNumber(),
                                                                              neuron_gene->getInnovNumber(),
                                                                              0,
                                                                              ++innov_number,
                                                                              true));
        connections.push_back(input_to_first);
      }
    }
  }
}

Body::~Body()
{
  for (BodyPart *todel : delete_later) {
    delete[] todel;
  }
  delete core;
}

CPPNEAT::GeneticEncodingPtr
Body::get_coupled_cpg_network()
{
  CPPNEAT::GeneticEncodingPtr ret(new CPPNEAT::GeneticEncoding(neurons,
                                                               connections));
  return ret;
}

std::pair<std::map<int, size_t>, std::map<int, size_t>>
Body::get_input_output_map(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                           const std::vector<revolve::gazebo::SensorPtr> &sensors)
{
  size_t p = 0;
  std::map<int, size_t> output_map;
  // Admapd output neurons to motors:

  // map of numbers of output neurons for each body part
  std::map<std::string, size_t> outputCountMap;

  for (auto it = actuators.begin(); it != actuators.end(); ++it) {
    auto motor = *it;
    auto partId = motor->partId();

    if (!outputCountMap.count(partId)) {
      outputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = motor->outputs(); i < l; ++i) {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      outputCountMap[partId]++;

      size_t j;
      for (j = 0; j < output_neurons.size(); j++) {
        if (output_neurons[j]->neuron
                             ->neuron_id == neuronId.str()) {
          break;
        }
      }
      if (j == output_neurons.size()) {
        std::cerr << "Required output neuron " << neuronId.str() <<
                  " for motor could not be located"
                  << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      output_map[output_neurons[j]->getInnovNumber()] = p++;
    }
  }

  p = 0;
  std::map<int, size_t> input_map;
  // Map input neurons to sensors:

  // map of number of input neurons for each part:

  std::map<std::string, size_t> inputCountMap;

  for (auto it = sensors.begin(); it != sensors.end(); ++it) {
    auto sensor = *it;
    auto partId = sensor->partId();

    if (!inputCountMap.count(partId)) {
      inputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = sensor->inputs(); i < l; ++i) {
      std::stringstream neuronId;
      neuronId << partId << "-in-" << inputCountMap[partId];
      inputCountMap[partId]++;

      size_t j;
      for (j = 0; j < input_neurons.size(); j++) {
        if (input_neurons[j]->neuron
                            ->neuron_id == neuronId.str()) {
          break;
        }
      }
      if (j == input_neurons.size()) {
        std::cerr << "Required input neuron " << neuronId.str() <<
                  " for sensor could not be located"
                  << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      input_map[input_neurons[j]->getInnovNumber()] = p++;
    }
  }
  return std::make_pair(input_map,
                        output_map);
}

CPPNEAT::GeneticEncodingPtr
Body::get_hyper_neat_network()
{
  int innov_number = 1;
  CPPNEAT::GeneticEncodingPtr ret(new CPPNEAT::GeneticEncoding(true));
  //add inputs
  std::map<std::string, double> empty;
  for (int i = 0; i < 6; i++) {
    CPPNEAT::NeuronPtr neuron(new CPPNEAT::Neuron(
            "Input-" + std::to_string(i), //better names (like input x1 etc) might help
            CPPNEAT::Neuron::INPUT_LAYER,
            CPPNEAT::Neuron::INPUT,
            empty));
    CPPNEAT::NeuronGenePtr neuron_gene(new CPPNEAT::NeuronGene(neuron,
                                                               innov_number++,
                                                               true)); //means innovation number is i + 1
    ret->add_neuron_gene(neuron_gene,
                         0,
                         i == 0);
  }

  //add outputs
  empty["rv:bias"] = 0;
  empty["rv:gain"] = 0;
  CPPNEAT::NeuronPtr weight_neuron(new CPPNEAT::Neuron("weight",
                                                       CPPNEAT::Neuron::OUTPUT_LAYER,
                                                       CPPNEAT::Neuron::SIMPLE,
                                                       empty));
  CPPNEAT::NeuronGenePtr weight_neuron_gene(new CPPNEAT::NeuronGene(weight_neuron,
                                                                    innov_number++,
                                                                    true));
  ret->add_neuron_gene(weight_neuron_gene,
                       1,
                       true);
  CPPNEAT::NeuronPtr bias_neuron(new CPPNEAT::Neuron("rv:bias",
                                                     CPPNEAT::Neuron::OUTPUT_LAYER,
                                                     CPPNEAT::Neuron::SIMPLE,
                                                     empty));
  CPPNEAT::NeuronGenePtr bias_neuron_gene(new CPPNEAT::NeuronGene(bias_neuron,
                                                                  innov_number++,
                                                                  true));
  ret->add_neuron_gene(bias_neuron_gene,
                       1,
                       false);
  CPPNEAT::NeuronPtr gain_neuron(new CPPNEAT::Neuron("rv:gain",
                                                     CPPNEAT::Neuron::OUTPUT_LAYER,
                                                     CPPNEAT::Neuron::SIMPLE,
                                                     empty));
  CPPNEAT::NeuronGenePtr gain_neuron_gene(new CPPNEAT::NeuronGene(gain_neuron,
                                                                  innov_number++,
                                                                  true));
  ret->add_neuron_gene(gain_neuron_gene,
                       1,
                       false);

  //connect every input with every output
  for (int i = 0; i < 6; i++) {
    CPPNEAT::ConnectionGenePtr connection_to_weight(new CPPNEAT::ConnectionGene(weight_neuron_gene->getInnovNumber(),
                                                                                i + 1,
                                                                                0,
                                                                                innov_number++,
                                                                                true,
                                                                                ""));
    ret->add_connection_gene(connection_to_weight);

    CPPNEAT::ConnectionGenePtr connection_to_bias(new CPPNEAT::ConnectionGene(bias_neuron_gene->getInnovNumber(),
                                                                              i + 1,
                                                                              0,
                                                                              innov_number++,
                                                                              true,
                                                                              ""));
    ret->add_connection_gene(connection_to_bias);

    CPPNEAT::ConnectionGenePtr connection_to_gain(new CPPNEAT::ConnectionGene(gain_neuron_gene->getInnovNumber(),
                                                                              i + 1,
                                                                              0,
                                                                              innov_number++,
                                                                              true,
                                                                              ""));
    ret->add_connection_gene(connection_to_gain);
  }
  return ret;
}

std::map<std::string, std::tuple<int, int, int> >
Body::get_id_to_coordinate_map()
{
  std::map<std::string, std::tuple<int, int, int> > ret;
  for (std::pair<CPPNEAT::NeuronGenePtr, std::tuple<int, int, int>> pair : neuron_coordinates) {
    ret[pair.first
            ->neuron
            ->neuron_id] = pair.second;
  }
  return ret;
}

std::vector<std::pair<int, int> >
Body::get_coordinates_sorted(const std::vector<revolve::gazebo::MotorPtr> &actuators)
{
  std::vector<std::pair<int, int>> ret;

  // map of numbers of output neurons for each body part
  std::map<std::string, size_t> outputCountMap;

  for (auto it = actuators.begin(); it != actuators.end(); ++it) {
    auto motor = *it;
    auto partId = motor->partId();

    if (!outputCountMap.count(partId)) {
      outputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = motor->outputs(); i < l; ++i) {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      outputCountMap[partId]++;

      size_t j;
      for (j = 0; j < output_neurons.size(); j++) {
        if (output_neurons[j]->neuron
                             ->neuron_id == neuronId.str()) {
          break;
        }
      }
      if (j == output_neurons.size()) {
        std::cerr << "Required output neuron " << neuronId.str() <<
                  " for motor could not be located"
                  << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      ret.push_back(std::pair<int, int>(std::get<0>(neuron_coordinates[output_neurons[j]]),
                                        std::get<1>(neuron_coordinates[output_neurons[j]])));
    }
  }
  return ret;
}


void
Body::set_coordinates(int x,
                      int y,
                      BodyPart *part)
{
  part->coordinates[0] = x;
  part->coordinates[1] = y;
}

void
Body::make_empty(BodyPart *part)
{
  part->name = "empty";
  part->neighbours = new BodyPart[4];
  delete_later.push_back(part->neighbours);
  for (int i = 0; i < 4; i++) {
    part->neighbours[i].name = "empty";
  }
}

int
Body::calc_rotation(int arity,
                    int slot,
                    int parent_rotation)
{
  if (arity == 2) {
    /*  slot 0
     *  Block
     *  slot 1 */
    return ((slot == 0) ? parent_rotation + 2 : parent_rotation) % 4;
  } else {
    /*  		slot 0
     *  	slot 3	Block	slot 2
     *  		slot 1 */
    switch (slot) {
      case 0:
        return (parent_rotation + 2) % 4;
      case 1:
        return parent_rotation;
      case 2:
        return (parent_rotation + 3) % 4;
      case 3:
        return (parent_rotation + 1) % 4;
      default:
        std::cout << "unknown error in body parsing" << std::endl;
    }
  }
}

void
Body::from_yaml(BodyPart *part,
                YAML::Node &node) //only works for slot: 0 currently
{
  std::string curname = node["id"].as<std::string>();
  part->name = node["id"].as<std::string>();
  part->type = node[(std::string)("type")].as<std::string>();
  part->arity = part_arity[part->type];

  //add differential oscillator in case of the current part being an ActiveHinge
  //output is also added here
  //needs to be done before adding the parent as a neighbour to the children since the neighbours are remembered by value and not reference
  if (part->type == "ActiveHinge") {
    //add neurons
    std::map<std::string, double> empty;
    empty["rv:bias"] = 0;
    empty["rv:gain"] = 1;

    CPPNEAT::NeuronPtr first_neuron(new CPPNEAT::Neuron(part->name + "-hidden-0",
                                                        CPPNEAT::Neuron::HIDDEN_LAYER,
                                                        CPPNEAT::Neuron::DIFFERENTIAL_CPG,
                                                        empty));
    part->differential_oscillator[0] = CPPNEAT::NeuronGenePtr(new CPPNEAT::NeuronGene(first_neuron,
                                                                                      ++innov_number,
                                                                                      true));
    neurons.push_back(part->differential_oscillator[0]);
    std::tuple<int, int, int> first_coord(part->coordinates[0],
                                          part->coordinates[1],
                                          1);
    neuron_coordinates[part->differential_oscillator[0]] = first_coord;

    CPPNEAT::NeuronPtr second_neuron(new CPPNEAT::Neuron(part->name + "-hidden-1",
                                                         CPPNEAT::Neuron::HIDDEN_LAYER,
                                                         CPPNEAT::Neuron::DIFFERENTIAL_CPG,
                                                         empty));
    part->differential_oscillator[1] = CPPNEAT::NeuronGenePtr(new CPPNEAT::NeuronGene(second_neuron,
                                                                                      ++innov_number,
                                                                                      true));
    neurons.push_back(part->differential_oscillator[1]);
    std::tuple<int, int, int> second_coord(part->coordinates[0],
                                           part->coordinates[1],
                                           -1);
    neuron_coordinates[part->differential_oscillator[1]] = second_coord;

    CPPNEAT::NeuronPtr output_neuron(new CPPNEAT::Neuron(part->name + "-out-0",
                                                         CPPNEAT::Neuron::OUTPUT_LAYER,
                                                         CPPNEAT::Neuron::SIMPLE,
                                                         empty));
    part->differential_oscillator[2] = CPPNEAT::NeuronGenePtr(new CPPNEAT::NeuronGene(output_neuron,
                                                                                      ++innov_number,
                                                                                      true));
    neurons.push_back(part->differential_oscillator[2]);
    output_neurons.push_back(part->differential_oscillator[2]);
    std::tuple<int, int, int> output_coord(part->coordinates[0],
                                           part->coordinates[1],
                                           0);
    neuron_coordinates[part->differential_oscillator[2]] = output_coord;

    //add connections between only these neurons (for now)
    CPPNEAT::ConnectionGenePtr first_to_second(new CPPNEAT::ConnectionGene(part->differential_oscillator[1]->getInnovNumber(),
                                                                           part->differential_oscillator[0]->getInnovNumber(),
                                                                           0,
                                                                           ++innov_number,
                                                                           true));
    connections.push_back(first_to_second);

    CPPNEAT::ConnectionGenePtr second_to_first(new CPPNEAT::ConnectionGene(part->differential_oscillator[0]->getInnovNumber(),
                                                                           part->differential_oscillator[1]->getInnovNumber(),
                                                                           0,
                                                                           ++innov_number,
                                                                           true));
    connections.push_back(second_to_first);

    CPPNEAT::ConnectionGenePtr first_to_output(new CPPNEAT::ConnectionGene(part->differential_oscillator[2]->getInnovNumber(),
                                                                           part->differential_oscillator[0]->getInnovNumber(),
                                                                           0,
                                                                           ++innov_number,
                                                                           true));
    connections.push_back(first_to_output);
  }


  //parse body and add initialised children to parsing queue
  if (part->arity == 4) {
    YAML::Node children = node["children"];
    if (children.IsNull()) {
      return;
    }
    if (children[0].size() != 0 && part->type != "Core") { //child in parent socket
      std::cout << "ERROR: child in parent socket (currently only 0 is accepted as parent socket)" << std::endl;
    }
    for (int i = 0; i < 4; i++) {
      if (children[i].size() == 0) {
        continue;
      }
      BodyPart *child = &part->neighbours[i];
      make_empty(child);
      child->rotation = calc_rotation(4,
                                      i,
                                      part->rotation);
      switch (child->rotation) {
        case 0:
          set_coordinates(part->coordinates[0],
                          part->coordinates[1] - 1,
                          child);
          break;
        case 1:
          set_coordinates(part->coordinates[0] - 1,
                          part->coordinates[1],
                          child);
          break;
        case 2:
          set_coordinates(part->coordinates[0],
                          part->coordinates[1] + 1,
                          child);
          break;
        case 3:
          set_coordinates(part->coordinates[0] + 1,
                          part->coordinates[1],
                          child);
          break;
        default:
          std::cout << "unknown error in body parsing" << std::endl;
      }
// 			std::cout << child->coordinates[0] << " " << child->coordinates[1] << " " << children[i]["id"].as<std::string>() << " " << i << " " << child->rotation << std::endl;

      //add parent as neighbour of child
      child->neighbours[0] = *part;

      body_to_node[child] = children[i];
      to_parse.push_back(child);
    }
  } else {
    YAML::Node children = node["children"];
    if (children.IsNull()) {
      return;
    }
    if (children[0].size() != 0) { //child in parent socket
      std::cout << "ERROR: child in parent socket (currently only 0 is accepted as parent socket)" << std::endl;
    }
    for (int i = 1; i < 2; i++) {
      if (children[i].size() == 0) {
        continue;
      }
      BodyPart *child = &part->neighbours[i];
      make_empty(child);
      child->rotation = calc_rotation(2,
                                      i,
                                      part->rotation);
      switch (child->rotation) {
        case 0:
          set_coordinates(part->coordinates[0],
                          part->coordinates[1] - 1,
                          child);
          break;
        case 1:
          set_coordinates(part->coordinates[0] - 1,
                          part->coordinates[1],
                          child);
          break;
        case 2:
          set_coordinates(part->coordinates[0],
                          part->coordinates[1] + 1,
                          child);
          break;
        case 3:
          set_coordinates(part->coordinates[0] + 1,
                          part->coordinates[1],
                          child);
          break;
        default:
          std::cout << "unknown error in body parsing" << std::endl;
      }
// 			std::cout << child->coordinates[0] << " " << child->coordinates[1] << " " << children[i]["id"].as<std::string>() << " " << i << " " << child->rotation << std::endl;

      //add parent as neighbour of child
      child->neighbours[0] = *part;

      body_to_node[child] = children[i];
      to_parse.push_back(child);
    }
  }

}

}
