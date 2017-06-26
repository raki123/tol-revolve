#include <iostream>
#include <cstring>

#include "YamlBodyParser.h"

namespace tol {

BodyPart::BodyPart()
{
  std::memset(neighbours, 0, sizeof(neighbours)*MAX_SLOTS);
}

BodyPart::BodyPart(const std::string &name, const std::string &type, int x, int y, size_t rotation)
        : name(name)
        , type(type)
        , x(x)
        , y(y)
        , rotation(rotation)
{
  std::memset(neighbours, 0, sizeof(neighbours)*MAX_SLOTS);
}

BodyPart::~BodyPart()
{
  // Delete dynamically allocated parents slots
  for (size_t parents_slot = 0; parents_slot < MAX_SLOTS; ++parents_slot) {
    delete neighbours[parents_slot];
  }
}

YamlBodyParser::YamlBodyParser(const std::string filepath)
{
  /// Verify and open a YAML file on a given file path
  YAML::Node genome;
  if ( access( filepath.c_str(), F_OK ) != -1 ) {
    genome = YAML::LoadFile(filepath);

    if (genome.IsNull()) {
      std::cerr << "Failed to load a 'genome' file." << std::endl;
      std::exit(1);
    }

  } else {
    std::cerr << "Filename '"<< filepath << "' does not exist!" << std::endl;
    std::exit(1);
  }

  /// Start parsing YAML file to a tree data structure
  this->bodyMap_ = this->parseModule(nullptr, genome["body"], 0, 0, 0); // Select the 'body' section from the YAML file.

  /// Prepare matrices for defining coordinates and connections
  int range_x = this->max_x - this->min_x;
  int range_y = this->max_y - this->min_y;

  this->coordinates_.resize(this->n_actuators_);
  this->connections_.resize(this->n_actuators_);
  for (size_t i = 0; i < this->n_actuators_; ++i) {
    this->coordinates_[i].resize(2);
    this->connections_[i].resize(n_actuators_);
  }

  /// Extract CPG coordinates matrix from a tree data structure
  this->setNormalisedCoordinates(this->bodyMap_, range_x, range_y);

  /// Extract connections matrix from a tree data structure
  this->setConnections(this->bodyMap_);

}

YamlBodyParser::~YamlBodyParser() {
  delete this->bodyMap_;
}

///////////////////////////////////////////////////////////////////////////////
BodyPart * YamlBodyParser::parseModule(BodyPart *parent,
                                       const YAML::Node &offspring,
                                       const size_t rotation,
                                       int y,
                                       int x)
{
  BodyPart* module = nullptr;
  if (offspring.IsDefined()) {

    module = new BodyPart();
    module->name = offspring["name"].as<std::string>();
    module->type = offspring["type"].as<std::string>();
    module->rotation = rotation;
    module->x = x;
    module->y = y;

    if (this->max_x < x) this->max_x = x;
    if (this->max_y < y) this->max_y = y;
    if (this->min_x > x) this->min_x = x;
    if (this->min_y > y) this->min_y = y;

    if (A_HINGE == module->type) {
      this->n_actuators_ += 1;
      module->id = this->n_actuators_;
    }

    if (A_HINGE == module->type || P_HINGE == module->type)
      module->arity = 2;
    else if (CORE == module->type || BRICK == module->type)
      module->arity = 4;

    if (offspring["children"].IsDefined()) {
      module->neighbours[0] = parent;
      size_t parents_slot = (CORE == module->type) ? 0 : 1;
      for (; parents_slot < MAX_SLOTS; ++parents_slot) {

        // Calculate coordinate for an offspring module
        int offsprings_x, offsprings_y;
        size_t offsprings_rotation = this->calculateRotation(module->arity, parents_slot, module->rotation);
        std::tie(offsprings_x, offsprings_y) = this->setCoordinates(offsprings_rotation, x, y);

        // Traverse recursively through each of offspring modules
        module->neighbours[parents_slot] = this->parseModule(module,
                                                             offspring["children"][parents_slot],
                                                             offsprings_rotation,
                                                             offsprings_y,
                                                             offsprings_x);
      }
    }
  }
  return module;
}

///////////////////////////////////////////////////////////////////////////////
size_t YamlBodyParser::calculateRotation(const size_t arity, const size_t slot, const size_t parents_rotation) const
{
  if (arity == 2) {
    return ((slot == 0) ? parents_rotation + 2 : parents_rotation) % 4;
  } else if (arity == 4){
    switch (slot) {
      case 0:
        return (parents_rotation + 2) % 4;
      case 1:
        return parents_rotation;
      case 2:
        return (parents_rotation + 3) % 4;
      case 3:
        return (parents_rotation + 1) % 4;
      default:
        std::cerr << "Unsupported parents slot provided." << std::endl;
        std::exit(-1);
    }
  } else {
    std::cerr << "Unsupported module arity provided." << std::endl;
    std::exit(-1);
  }
}

///////////////////////////////////////////////////////////////////////////////
std::tuple<int, int> YamlBodyParser::setCoordinates(const size_t rotation, const int init_x, const int init_y)
{
  int x = init_x;
  int y = init_y;

  switch (rotation) {
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

///////////////////////////////////////////////////////////////////////////////
void YamlBodyParser::setNormalisedCoordinates(BodyPart *module, const int range_x, const int range_y)
{
  if (module != nullptr) {
    if (A_HINGE == module->type) {
      this->coordinates_[module->id - 1][0] = (float)(module->x * (1.0/range_x));
      this->coordinates_[module->id - 1][1] = (float)(module->y * (1.0/range_y));
    }

    size_t parents_slot = (CORE == module->type) ? 0 : 1;
    for (; parents_slot < MAX_SLOTS; ++parents_slot) {
      this->setNormalisedCoordinates(module->neighbours[parents_slot], range_x, range_y);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
void YamlBodyParser::setConnections(BodyPart *module)
{
  if (module != nullptr) {
    if (CORE == module->type || BRICK == module->type) {
      for (size_t i = 0; i < MAX_SLOTS; ++i) {
        if (module->neighbours[i] != nullptr && A_HINGE == module->neighbours[i]->type) {
          for (size_t j = 0; j < MAX_SLOTS; ++j) {
            if (i != j && module->neighbours[j] != nullptr && A_HINGE == module->neighbours[j]->type)
              this->connections_[module->neighbours[i]->id - 1][module->neighbours[j]->id - 1] = true;
          }
        }
      }
    }

    size_t parents_slot = (CORE == module->type) ? 0 : 1;
    for (; parents_slot < MAX_SLOTS; ++parents_slot) {
      this->setConnections(module->neighbours[parents_slot]);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
ConnectionMatrix YamlBodyParser::connections()
{
  return this->connections_;
}

///////////////////////////////////////////////////////////////////////////////
CoordinatesMatrix YamlBodyParser::coordinates()
{
  return this->coordinates_;
}

}
