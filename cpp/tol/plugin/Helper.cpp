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

#include <boost/make_shared.hpp>

#include "Helper.h"

using namespace tol;

const std::vector<revolve::brain::ActuatorPtr>
Helper::createWrapper(const std::vector<revolve::gazebo::MotorPtr> &original)
{
  std::vector<revolve::brain::ActuatorPtr> result;
  for (unsigned int i = 0; i < original.size(); i++) {
    result.push_back(boost::make_shared<tol::Actuator>(tol::Actuator(original[i])));
  }

  return result;
}

const std::vector<revolve::brain::SensorPtr>
Helper::createWrapper(const std::vector<revolve::gazebo::SensorPtr> &original)
{
  std::vector<revolve::brain::SensorPtr> result;
  for (unsigned int i = 0; i < original.size(); i++) {
    result.push_back(boost::make_shared<tol::Sensor>(tol::Sensor(original[i])));
  }

  return result;
}

Helper::RobotType Helper::parseRobotType(const std::string &value) {
    if (value.compare("spider9") == 0)
        return RobotType::spider9;
    if (value.compare("spider13") == 0)
        return RobotType::spider13;
    if (value.compare("spider17") == 0)
        return RobotType::spider17;

    if (value.compare("gecko7") == 0)
        return RobotType::gecko7;
    if (value.compare("gecko12") == 0)
        return RobotType::gecko12;
    if (value.compare("gecko17") == 0)
        return RobotType::gecko17;

    if (value.compare("snake5") == 0)
        return RobotType::snake5;
    if (value.compare("snake7") == 0)
        return RobotType::snake7;
    if (value.compare("snake9") == 0)
        return RobotType::snake9;

    if (value.compare("babyA") == 0)
        return RobotType::babyA;
    if (value.compare("babyB") == 0)
        return RobotType::babyB;
    if (value.compare("babyC") == 0)
        return RobotType::babyC;

    //default value
    std::cerr << "Impossible to parse robot type (" << value << ")\nThrowing exception!"<<std::endl;
    throw std::invalid_argument("robot type impossible to parse");
}

std::ostream &operator<<(std::ostream &os, tol::Helper::RobotType type) {
    switch (type) {
        case tol::Helper::RobotType::spider9 :
            os << "spider9";
            break;
        case tol::Helper::RobotType::spider13:
            os << "spider13";
            break;
        case tol::Helper::RobotType::spider17:
            os << "spider17";
            break;
        case tol::Helper::RobotType::gecko7  :
            os << "gecko7";
            break;
        case tol::Helper::RobotType::gecko12 :
            os << "gecko12";
            break;
        case tol::Helper::RobotType::gecko17 :
            os << "gecko17";
            break;
        case tol::Helper::RobotType::snake5  :
            os << "snake5";
            break;
        case tol::Helper::RobotType::snake7  :
            os << "snake7";
            break;
        case tol::Helper::RobotType::snake9  :
            os << "snake9";
            break;
        case tol::Helper::RobotType::babyA   :
            os << "babyA";
            break;
        case tol::Helper::RobotType::babyB   :
            os << "babyB";
            break;
        case tol::Helper::RobotType::babyC   :
            os << "babyC";
            break;
        default      :
            os.setstate(std::ios_base::failbit);
    }
    return os;
}
