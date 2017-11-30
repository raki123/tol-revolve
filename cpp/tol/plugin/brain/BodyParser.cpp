/*
* Copyright (C) 2017 Vrije Universiteit Amsterdam
*
* Licensed under the Apache License, Version 2.0 (the "License");
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Description: TODO: <Add brief description about file purpose>
* Author: Rafael Kiesel, modified Milan Jelisavcic
*
*/

#include <map>
#include <string>
#include <utility>
#include <tuple>
#include <vector>

#include "brain/learner/cppneat/CPPNNeuron.h"

#include "BodyParser.h"

namespace rg = revolve::gazebo;

using namespace tol;

///////////////////////////////////////////////////
BodyParser::BodyParser(const std::string &_yamlPath)
        : innovation_number_(0)
{
  YAML::Node yaml_genome = YAML::LoadFile(_yamlPath);
  if (yaml_genome.IsNull())
  {
    std::cerr << "Failed to load a YAML 'genome' file." << std::endl;
    return;
  }
  this->initParser(yaml_genome);
//  this->init(yaml_genome);
}

///////////////////////////////////////////////////
void BodyParser::initParser(const YAML::Node &_yaml)
{
  this->arity_["Core"] = 4;
  this->arity_["CoreWithMics"] = 4;
  this->arity_["FixedBrick"] = 4;
  this->arity_["ActiveHinge"] = 2;
  this->arity_["Hinge"] = 2;
  this->arity_["ParametricBarJoint"] = 2;

  // read body structure from yaml file
  // includes adding the rotation of the parts and their coordinates
  this->bodyMap_ = new BodyPart();
  this->toParse_.push_back(this->bodyMap_);
  this->bodyToNode_[bodyMap_] = _yaml["body"];
  this->initPart(bodyMap_);

  BodyPart *current_part;
  while (not this->toParse_.empty())
  {
    current_part = this->toParse_.back();
    this->toParse_.pop_back();
    this->ParseYaml(current_part, this->bodyToNode_[current_part]);
  }

  // couple neighbouring differential oscillators
  // here all oscillators must already be initialised
  this->ConnectOscillators();
}

///////////////////////////////////////////////////
BodyParser::~BodyParser()
{
  for (BodyPart *todel : this->toDelete_)
  {
    delete[] todel;
  }
  delete this->bodyMap_;
}

///////////////////////////////////////////////////
CPPNEAT::GeneticEncodingPtr BodyParser::CoupledCpgNetwork()
{
  CPPNEAT::GeneticEncodingPtr ret(
          new CPPNEAT::GeneticEncoding(this->neurons_, this->connections_));
  return ret;
}

///////////////////////////////////////////////////
std::pair< std::map< int, size_t >, std::map< int, size_t>>
BodyParser::InputOutputMap(
        const std::vector< rg::MotorPtr > &_actuators,
        const std::vector< rg::SensorPtr > &_sensors)
{
  size_t p = 0;
  std::map< int, size_t > output_map;
  // Add map output neurons to motors:

  // map of numbers of output neurons for each body part
  std::map< std::string, size_t > outputCountMap;

  for (const auto &motor : _actuators)
  {
    auto partId = motor->partId();

    if (not outputCountMap.count(partId))
    {
      outputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = motor->outputs(); i < l; ++i)
    {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      ++outputCountMap[partId];

      size_t j;
      for (j = 0; j < this->outputNeurons_.size(); ++j)
      {
        if (this->outputNeurons_[j]->neuron->neuron_id == neuronId.str())
        {
          break;
        }
      }
//      if (j == outputNeurons_.size())
//      {
//        std::cerr << "Required output neuron " << neuronId.str()
//                  << " for motor could not be located" << std::endl;
//        throw std::runtime_error("Robot brain error");
//      }
      output_map[this->outputNeurons_[j]->InnovationNumber()] = p++;
    }
  }

  p = 0;
  std::map< int, size_t > input_map;
  // Map input neurons to sensors:

  // map of number of input neurons for each part:

  std::map< std::string, size_t > inputCountMap;

  for (const auto &sensor : _sensors)
  {
    auto partId = sensor->partId();

    if (not inputCountMap.count(partId))
    {
      inputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = sensor->inputs(); i < l; ++i)
    {
      std::stringstream neuronId;
      neuronId << partId << "-in-" << inputCountMap[partId];
      ++inputCountMap[partId];

      size_t j;
      for (j = 0; j < this->inputNeurons_.size(); ++j)
      {
        if (this->inputNeurons_[j]->neuron->neuron_id == neuronId.str())
        {
          break;
        }
      }
      if (j == this->inputNeurons_.size())
      {
        std::cerr << "Required input neuron " << neuronId.str()
                  << " for sensor could not be located" << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      input_map[this->inputNeurons_[j]->InnovationNumber()] = p++;
    }
  }
  return {input_map, output_map};
}

///////////////////////////////////////////////////
CPPNEAT::GeneticEncodingPtr BodyParser::CppnNetwork()
{
  size_t innovationNumber = 1;
  CPPNEAT::GeneticEncodingPtr cppn(new CPPNEAT::GeneticEncoding(true));
  // add inputs
  std::map< std::string, double > parameters;
  for (size_t i = 0; i < 6; ++i)
  {
    // better names (like input x1 etc) might help
    CPPNEAT::NeuronPtr neuron(new CPPNEAT::Neuron(
            "Input-" + std::to_string(i),
            CPPNEAT::Neuron::INPUT_LAYER,
            CPPNEAT::Neuron::INPUT,
            parameters));
    // means innovation number is i + 1
    CPPNEAT::NeuronGenePtr neuron_gene(new CPPNEAT::NeuronGene(
            neuron,
            innovationNumber++,
            true));
    cppn->AddNeuron(neuron_gene, 0, i == 0);
  }

  // add outputs
  parameters["rv:bias"] = 0;
  parameters["rv:gain"] = 0;
  CPPNEAT::NeuronPtr weightNeuron(new CPPNEAT::Neuron(
          "weight",
          CPPNEAT::Neuron::OUTPUT_LAYER,
          CPPNEAT::Neuron::SIMPLE,
          parameters));
  CPPNEAT::NeuronGenePtr weightNeuronGene(new CPPNEAT::NeuronGene(
          weightNeuron,
          innovationNumber++,
          true));
  cppn->AddNeuron(weightNeuronGene, 1, true);
  CPPNEAT::NeuronPtr biasNeuron(new CPPNEAT::Neuron(
          "rv:bias",
          CPPNEAT::Neuron::OUTPUT_LAYER,
          CPPNEAT::Neuron::SIMPLE,
          parameters));
  CPPNEAT::NeuronGenePtr biasNeuronGene(new CPPNEAT::NeuronGene(
          biasNeuron,
          innovationNumber++,
          true));
  cppn->AddNeuron(biasNeuronGene, 1, false);
  CPPNEAT::NeuronPtr gainNeuron(new CPPNEAT::Neuron(
          "rv:gain",
          CPPNEAT::Neuron::OUTPUT_LAYER,
          CPPNEAT::Neuron::SIMPLE,
          parameters));
  CPPNEAT::NeuronGenePtr gainNeuronGene(new CPPNEAT::NeuronGene(
          gainNeuron,
          innovationNumber++,
          true));
  cppn->AddNeuron(gainNeuronGene, 1, false);

  // connect every input with every output
  for (size_t i = 0; i < 6; ++i)
  {
    CPPNEAT::ConnectionGenePtr connectionToWeight(new CPPNEAT::ConnectionGene(
            weightNeuronGene->InnovationNumber(),
            i + 1,
            0,
            innovationNumber++,
            true, ""));
    cppn->AddConnection(connectionToWeight);

    CPPNEAT::ConnectionGenePtr connectionToBias(new CPPNEAT::ConnectionGene(
            biasNeuronGene->InnovationNumber(),
            i + 1,
            0,
            innovationNumber++,
            true, ""));
    cppn->AddConnection(connectionToBias);

    CPPNEAT::ConnectionGenePtr connectionToGain(new CPPNEAT::ConnectionGene(
            gainNeuronGene->InnovationNumber(),
            i + 1,
            0,
            innovationNumber++,
            true, ""));
    cppn->AddConnection(connectionToGain);
  }
  return cppn;
}

///////////////////////////////////////////////////
std::map< std::string, CoordsTriple > BodyParser::IdToCoordinatesMap()
{
  std::map< std::string, CoordsTriple > ret;
  for (auto pair : this->coordinates_)
  {
    ret[pair.first->neuron->neuron_id] = pair.second;
  }
  return ret;
}

///////////////////////////////////////////////////
std::vector< std::pair< int, int > > BodyParser::SortedCoordinates(
        const std::vector< rg::MotorPtr > &_actuators)
{
  std::vector< std::pair< int , int >> coordinates;

  // map of numbers of output neurons for each body part
  std::map< std::string, size_t > outputCountMap;

  for (const auto &motor : _actuators)
  {
    auto partId = motor->partId();

    if (not outputCountMap.count(partId))
    {
      outputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = motor->outputs(); i < l; ++i)
    {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      outputCountMap[partId]++;

      size_t j;
      for (j = 0; j < this->outputNeurons_.size(); ++j)
      {
        if (this->outputNeurons_[j]->neuron->neuron_id == neuronId.str())
        {
          break;
        }
      }
      if (j == this->outputNeurons_.size())
      {
        std::cerr << "Required output neuron " << neuronId.str()
                  << " for motor could not be located" << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      coordinates.push_back({
              std::get< 0 >(this->coordinates_[this->outputNeurons_[j]]),
              std::get< 1 >(this->coordinates_[this->outputNeurons_[j]])});
    }
  }
  return coordinates;
}

///////////////////////////////////////////////////
std::tuple< int, int > BodyParser::setCoordinates(
        const size_t _rotation,
        const int _init_x,
        const int _init_y)
{
  int x = _init_x;
  int y = _init_y;

  switch (_rotation)
  {
    case 0:
      x += 1;
      break;
    case 1:
      y += 1;
      break;
    case 2:
      x -= 1;
      break;
    case 3:
      y -= 1;
      break;
    default:
      std::cerr << "Wrong rotation calculated." << std::endl;
      std::exit(-1);
  }

  return std::make_tuple(x, y);
}

///////////////////////////////////////////////////
size_t BodyParser::calculateRotation(
        const size_t _arity,
        const size_t _slot,
        const size_t _parents_rotation)
{
  if (_arity == 2 or _arity == 4)
  {
    switch (_slot)
    {
      case 0:
        return (_parents_rotation + 2) % 4;
      case 1:
        return _parents_rotation;
      case 2:
        return (_parents_rotation + 3) % 4;
      case 3:
        return (_parents_rotation + 1) % 4;
      default:
        std::cerr << "Unsupported parents slot provided." << std::endl;
        std::exit(-1);
    }
  }
  else
  {
    std::cerr << "Unsupported module arity provided." << std::endl;
    std::exit(-1);
  }
}

///////////////////////////////////////////////////
void BodyParser::initPart(BodyPart *_part)
{
  _part->rotation = 0;
  _part->x = 0;
  _part->y = 0;

  _part->neighbours = new BodyPart[MAX_SLOTS];
  this->toDelete_.push_back(_part->neighbours);
  for (size_t i = 0; i < MAX_SLOTS; ++i)
  {
    _part->neighbours[i].name = "empty";
  }
}

///////////////////////////////////////////////////
CPPNEAT::NeuronGenePtr BodyParser::GenerateDifferentialNeuron(
        BodyPart *_module,
        size_t _position,
        std::string _name)
{
  std::map< std::string, double > params;
  {
    params["rv:bias"] = 0;
    params["rv:gain"] = 1;
  }
  int x_coordinate, y_coordinate, z_coordinate;
  CPPNEAT::Neuron::Layer layer;
  CPPNEAT::Neuron::Ntype type;

  switch (_position)
  {
    case 0:
      x_coordinate = _module->x;
      y_coordinate = _module->y;
      z_coordinate = 1;
      layer = CPPNEAT::Neuron::HIDDEN_LAYER;
      type = CPPNEAT::Neuron::DIFFERENTIAL_CPG;
      break;
    case 1:
      x_coordinate = _module->x;
      y_coordinate = _module->y;
      z_coordinate = -1;
      layer = CPPNEAT::Neuron::HIDDEN_LAYER;
      type = CPPNEAT::Neuron::DIFFERENTIAL_CPG;
      break;
    case 2:
      x_coordinate = _module->x;
      y_coordinate = _module->y;
      z_coordinate = 0;
      layer = CPPNEAT::Neuron::OUTPUT_LAYER;
      type = CPPNEAT::Neuron::SIMPLE;
      break;
    case 3:
      x_coordinate = 0;
      y_coordinate = 0;
      z_coordinate = 0;
      layer = CPPNEAT::Neuron::INPUT_LAYER;
      type = CPPNEAT::Neuron::INPUT;
      params.clear();
      break;
    default:
      std::cerr << "Unsupported neuron position provided." << std::endl;
      std::exit(1);
  }

  CPPNEAT::NeuronPtr neuron(new CPPNEAT::Neuron(_name, layer, type, params));
  CPPNEAT::NeuronGenePtr neuronGene = CPPNEAT::NeuronGenePtr(
          new CPPNEAT::NeuronGene(neuron, ++innovation_number_, true));

  if (_position == 0 or _position == 1)
  {
    _module->differential_oscillator[_position] = neuronGene;
  }
  else if (_position == 2)
  {
    _module->differential_oscillator[_position] = neuronGene;
    this->outputNeurons_.push_back(neuronGene);
  }
  else if (_position == 3)
  {
    this->inputNeurons_.push_back(neuronGene);
  }

  this->neurons_.push_back(neuronGene);

  CoordsTriple coordinate(x_coordinate, y_coordinate, z_coordinate);
  this->coordinates_[neuronGene] = coordinate;

  return neuronGene;
}

///////////////////////////////////////////////////
void BodyParser::GenerateConnection(
        CPPNEAT::NeuronGenePtr _from,
        CPPNEAT::NeuronGenePtr _to)
{
  double weight = 0.0;
  bool isActive = true;

  CPPNEAT::ConnectionGenePtr connection(new CPPNEAT::ConnectionGene(
          _to->InnovationNumber(),
          _from->InnovationNumber(),
          weight,
          ++innovation_number_,
          isActive));
  this->connections_.push_back(connection);
}

///////////////////////////////////////////////////
void BodyParser::GenerateOscillator(BodyPart *_module)
{
  if (_module->type == "ActiveHinge")
  {
    this->GenerateDifferentialNeuron(_module, 0, _module->name + "-hidden-0");
    this->GenerateDifferentialNeuron(_module, 1, _module->name + "-hidden-1");
    this->GenerateDifferentialNeuron(_module, 2, _module->name + "-out-0");

    // add connections between only these neurons (for now)
    this->GenerateConnection(_module->differential_oscillator[0],
                             _module->differential_oscillator[1]);
    this->GenerateConnection(_module->differential_oscillator[1],
                             _module->differential_oscillator[0]);
    this->GenerateConnection(_module->differential_oscillator[0],
                             _module->differential_oscillator[2]);
  }
}

///////////////////////////////////////////////////
void BodyParser::ConnectOscillators()
{
  BodyPart *module;
  this->toParse_.push_back(this->bodyMap_);
  while (not this->toParse_.empty())
  {
    module = this->toParse_.back();
    this->toParse_.pop_back();

    bool isNeighbourActuator[MAX_SLOTS];
    for (size_t i = 0; i < MAX_SLOTS; ++i)
    {
      isNeighbourActuator[i] = (module->neighbours[i].type == "ActiveHinge");
    }
    // add connections between each of the first neurons of an oscillator
    for (size_t i = 0; i < MAX_SLOTS; ++i)
    {
      for (size_t j = 0; j < MAX_SLOTS; ++j)
      {
        if (isNeighbourActuator[i] and isNeighbourActuator[j] and i not_eq j)
        {
          this->GenerateConnection(
                  module->neighbours[i].differential_oscillator[0],
                  module->neighbours[j].differential_oscillator[0]);
        }
      }
    }

    // add neighbours to queue and watch out for parent
    if (module == this->bodyMap_)
    {
      if (module->neighbours[0].name not_eq "empty")
      {
        this->toParse_.push_back(&(module->neighbours[0]));
      }
    }
    for (size_t i = 1; i < MAX_SLOTS; ++i)
    {
      if (module->neighbours[i].name not_eq "empty")
      {
        this->toParse_.push_back(&(module->neighbours[i]));
      }
    }
  }

  // Add input neurons and add connections between them and neurons on core
  for (size_t i = 0; i < 6; ++i)
  {
    // Add input neuron
    CPPNEAT::NeuronGenePtr neuronGene = this->GenerateDifferentialNeuron(
            this->bodyMap_,
            3,
            bodyMap_->name + "-in-" + std::to_string(i));

    // Connect to neighbouring oscillators
    for (size_t j = 0; j < MAX_SLOTS; ++j)
    {
      if (this->bodyMap_->neighbours[j].type == "ActiveHinge")
      {
        this->GenerateConnection(
                neuronGene,
                bodyMap_->neighbours[j].differential_oscillator[0]);
      }
    }
  }
}

///////////////////////////////////////////////////
// only works for slot: 0 currently
void BodyParser::ParseYaml(
        BodyPart *_module,
        YAML::Node &_yaml)
{
  _module->name = _yaml["id"].as< std::string >();
  _module->type = _yaml["type"].as< std::string >();
  _module->arity = arity_[_module->type];

  // add differential oscillator in case of the current part being an
  // ActiveHinge output is also added here needs to be done before adding the
  // parent as a neighbour to the children since the neighbours are remembered
  // by value and not reference
  this->GenerateOscillator(_module);

  if (_yaml["children"].IsDefined())
  {
    YAML::Node yaml_offsprings = _yaml["children"];
    for (size_t parents_slot = 0; parents_slot < MAX_SLOTS; ++parents_slot)
    {
      if (yaml_offsprings[parents_slot].IsDefined())
      {
        BodyPart *child = &_module->neighbours[parents_slot];
        this->initPart(child);
        // add parent as neighbour of child
        child->neighbours[0] = *_module;

        child->rotation = this->calculateRotation(
                (_module->arity == 4) ? 4 : 2,
                parents_slot,
                _module->rotation);

        int offsprings_x, offsprings_y;
        std::tie(offsprings_x, offsprings_y) =
                this->setCoordinates(child->rotation, _module->x, _module->y);
        child->x = offsprings_x;
        child->y = offsprings_y;

        this->bodyToNode_[child] = yaml_offsprings[parents_slot];
        this->toParse_.push_back(child);
      }
    }
  }
}
