#ifndef YAML_BODY_PARSE_H
#define YAML_BODY_PARSE_H

#include "brain/cppneat/genetic_encoding.h"
#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <yaml-cpp/yaml.h>


namespace tol {

	class Body {
		struct BodyPart {
			int arity;
			std::string name;
			std::string type;
			int rotation; 		//0 is standard, 1 is rotated to the right by 90 degrees, ...
			BodyPart *neighbours; 	//the neighbours of this bodypart. neighbour in socket i is in neighbours[i]
			int coordinates[2]; 	//the coordinates of this bodypart
			CPPNEAT::NeuronGenePtr differential_oscillator[3];
		};
	public:
		Body(std::string &yaml_path);
		~Body();
		CPPNEAT::GeneticEncodingPtr get_coupled_cpg_network();
		std::pair<std::map<int, unsigned int>, std::map<int, unsigned int>> get_input_output_map(const std::vector<revolve::gazebo::MotorPtr> &actuators,
													 const std::vector<revolve::gazebo::SensorPtr> &sensors);
		CPPNEAT::GeneticEncodingPtr get_hyper_neat_network();
		int getInnovNumber() { return innov_number + 1; };
	private:
		//Body parsing
		std::map<std::string, int> part_arity;
		std::vector<BodyPart *> to_parse;
		BodyPart *core;
		std::vector<BodyPart *> delete_later;
		std::map<BodyPart *, YAML::Node> body_to_node;
		
		static void set_coordinates(int x, int y, BodyPart *part);
		static int calc_rotation(int arity, int slot, int parent_rotation) ;
		void make_empty(BodyPart *part);
		void from_yaml(BodyPart *part, YAML::Node &node);
		
		//Network
		int innov_number;
		std::vector<CPPNEAT::ConnectionGenePtr> connections;
		std::vector<CPPNEAT::NeuronGenePtr> neurons;
		std::vector<CPPNEAT::NeuronGenePtr> input_neurons;
		std::vector<CPPNEAT::NeuronGenePtr> output_neurons;
		std::map<CPPNEAT::NeuronGenePtr, std::tuple<int,int,int>> neuron_coordinates;
		


	};
}

#endif // YAML_BODY_PARSE_H
