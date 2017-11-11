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
* Author: TODO <Add proper author>
*
*/

#ifndef TOL_PLUGIN_BODY_H_
#define TOL_PLUGIN_BODY_H_

#include <vector>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "brain/learner/cppneat/CPPNNeuron.h"
#include "brain/learner/cppneat/GeneticEncoding.h"

namespace tol
{
  typedef std::tuple< int, int, int > CoordsTriple;

  const std::string CORE = "Core";

  const std::string A_HINGE = "ActiveHinge";

  const std::string P_HINGE = "PassiveHinge";

  const std::string BRICK = "FixedBrick";

  const size_t MAX_SLOTS = 4;

  class BodyParser
  {
    struct BodyPart
    {
      std::string name = "empty";
      std::string type = CORE;
      int x = 0;
      int y = 0;
      size_t id = 0;
      size_t arity = 4;
      size_t rotation = 0;          // 0 -> 0, 1 -> 90 degrees, etc.

      BodyPart *neighbours;  // the neighbours of this `bodypart`
      CPPNEAT::NeuronGenePtr differential_oscillator[3];
    };

    public:
    BodyParser(const std::string &_yamlPath);

    ~BodyParser();

    CPPNEAT::GeneticEncodingPtr CoupledCpgNetwork();

    std::pair< std::map< int, size_t >, std::map< int, size_t>>
    InputOutputMap(
            const std::vector< revolve::gazebo::MotorPtr > &_actuators,
            const std::vector< revolve::gazebo::SensorPtr > &_sensors);

    CPPNEAT::GeneticEncodingPtr CppnNetwork();

    std::map< std::string, CoordsTriple > IdToCoordinatesMap();

    /// \brief returns the coordinates of the actuators matching the order the
    /// actuators give coordinate of actuators[0] is in sorted_coordinates[0]
    std::vector< std::pair< int, int >> SortedCoordinates(
            const std::vector< revolve::gazebo::MotorPtr > &_actuators);

    size_t InnovationNumber()
    {
      return ++innovation_number_;
    };

    private:
    static std::tuple< int, int > setCoordinates(
            const size_t _rotation,
            const int _init_x,
            const int _init_y);

    static size_t calculateRotation(
            const size_t _arity,
            const size_t _slot,
            const size_t _parents_rotation);

    void initPart(BodyPart *_part);

    private:
    void ParseYaml(
            BodyPart *_module,
            YAML::Node &_yaml);

    void initParser(const YAML::Node &_yaml);

    // Network
    void GenerateOscillator(BodyPart *_module);

    CPPNEAT::NeuronGenePtr GenerateDifferentialNeuron(
            BodyPart *_module,
            size_t _position,
            std::string _name);

    void GenerateConnection(
            CPPNEAT::NeuronGenePtr _from,
            CPPNEAT::NeuronGenePtr _to);

    void ConnectOscillators();

    /// \brief innovation number
    size_t innovation_number_;

    std::vector< CPPNEAT::NeuronGenePtr > neurons_;

    std::vector< CPPNEAT::NeuronGenePtr > inputNeurons_;

    std::vector< CPPNEAT::NeuronGenePtr > outputNeurons_;

    std::vector< CPPNEAT::ConnectionGenePtr > connections_;

    std::map< CPPNEAT::NeuronGenePtr, CoordsTriple > coordinates_;

    // Body parsing
    std::map< std::string, size_t > arity_;
    std::vector< BodyPart * > toParse_;
    std::vector< BodyPart * > toDelete_;
    std::map< BodyPart *, YAML::Node > bodyToNode_;

    /// \brief A tree data structure representing a robot
    BodyPart *bodyMap_ = nullptr;
  };
}

#endif  //  TOL_PLUGIN_BODY_H_
