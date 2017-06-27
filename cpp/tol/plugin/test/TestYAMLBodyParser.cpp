#include "TestYAMLBodyParser.h"

#define BOOST_TEST_MODULE TestYAMLBodyParser test
#define BOOST_TEST_DYN_LINK
#include <boost/test/included/unit_test.hpp>

#include "../YamlBodyParser.h"

void testRobot(const std::string &yaml_source,
               const std::vector <std::vector<bool>> &connections_target,
               const std::vector <std::vector<float>> &coordinates_target)
{
    tol::YamlBodyParser* parser = new tol::YamlBodyParser("../res/robots/spider9.yaml");

    // Check connections
    std::vector <std::vector<bool>> connections = parser->connections();
    BOOST_CHECK_EQUAL(connections.size(), connections_target.size());
    for (size_t i = 0; i< connections.size(); i++) {
//        std::cout << "connections[" << i << ']' << std::endl;
        BOOST_CHECK_EQUAL_COLLECTIONS(connections[i].begin(), connections[i].end(),
                                      connections_target[i].begin(), connections_target[i].end());
    }

    // Check coordinates
    std::vector <std::vector<float>> coordinates = parser->coordinates();
    BOOST_CHECK_EQUAL(coordinates.size(), coordinates_target.size());
    for (size_t i = 0; i< coordinates.size(); i++) {
//        std::cout << "coordinates[" << i << ']' << std::endl;
        BOOST_CHECK_EQUAL_COLLECTIONS(coordinates[i].begin(), coordinates[i].end(),
                                      coordinates_target[i].begin(), coordinates_target[i].end());
    }
}

BOOST_AUTO_TEST_CASE(yaml_body_parser_just_runs)
{
    tol::YamlBodyParser* parser = new tol::YamlBodyParser("../res/robots/spider9.yaml");
    std::vector <std::vector<bool>> connections = parser->connections();
    std::vector <std::vector<float>> coordinates = parser->coordinates();
    BOOST_TEST(true, "Test did pass!");
}

BOOST_AUTO_TEST_CASE(yaml_body_parser_spider)
{
    testRobot(tol::Spider9_yaml_source,
              {
                      {false, true, true,false, true,false, true,false},
                      { true,false,false,false,false,false,false,false},
                      { true,false,false, true, true,false, true,false},
                      {false,false, true,false,false,false,false,false},
                      { true,false, true,false,false, true, true,false},
                      {false,false,false,false, true,false,false,false},
                      { true,false, true,false, true,false,false, true},
                      {false,false,false,false,false,false, true,false}
              },
              {
                      {-0.125f,    0}, //{1,    0,    1},
                      {-0.375f,    0}, //{.5,   0,    -1},
                      // Leg10Joint Leg11Joint
                      {0.125f,     0}, //{-1,    0,    1},
                      {0.375f,     0}, //{-.5f,    0,    -1},
                      // Leg20Joint Leg21Joint
                      {0,    -0.125f}, //{0,    1,    1},
                      {0,    -0.375f}, //{0,    .5,   -1},
                      // Leg30Joint Leg31Joint
                      {0,     0.125f}, //{0,    -1,   1},
                      {0,     0.375f}  //{0,    -.5f, -1}
              }
    );
}
