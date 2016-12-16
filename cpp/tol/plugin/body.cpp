#include "body.h"
#include <iostream>
#include <utility>

namespace tol {
	
	
Body::Body(std::string yaml_path) 
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
	

	std::vector<YAML::Node> to_parse(1, policy_file["body"]);
	node_to_body[to_parse[0]] = &core;
	core.rotation = 0;
	set_coordinates(0,0,0, &core);
	YAML::Node cur_node;
	while(!to_parse.empty()) {
		return;
	}
}	
Body::~Body()
{
	for(std::pair<YAML::Node, Body::BodyPart *> todel : node_to_body) {
		delete [] todel.second->neighbours;
	}
}


void Body::set_coordinates(int x, int y, int z, Body::BodyPart *part) {
	part->coordinates[0] = x;
	part->coordinates[1] = y;
	part->coordinates[2] = z;
}

void Body::make_empty(Body::BodyPart *part)
{
	part->name = "empty";
	part->neighbours = new BodyPart[4];
}

void Body::from_yaml(Body::BodyPart *part, YAML::Node node) //only works for slot: 0 currently
{
	part->name = node["id"].as<std::string>();
	part->type = node["type"].as<std::string>();
	part->arity = part_arity[part->type];
	if(part_arity == 4) 
	{
		for(int i = 0; i < 4; i++) {
			
		}
	}
	else 
	{
		
	}
}

}