#ifndef YAML_BODY_PARSE_H
#define YAML_BODY_PARSE_H

#include <vector>
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>


namespace tol {

	class Body {
		struct BodyPart {
			int arity;
			std::string name;
			std::string type;
			int rotation; //0 is standard, 1 is rotated to the right by 90 degrees, ...
			BodyPart *neighbours; //the neighbours of this bodypart. neighbour in socket i is in neighbours[i]
			int coordinates[3]; //the coordinates of this bodypart
		};
	public:
		Body(std::string yaml_path);
		~Body();
	private:
		std::map<std::string, int> part_arity;
		
		BodyPart core;
		std::map<YAML::Node, BodyPart *> node_to_body;
		static void set_coordinates(int x, int y, int z, BodyPart *part);
		static void make_empty(BodyPart *part);
		void from_yaml(BodyPart *part);

	};
}

#endif // YAML_BODY_PARSE_H
