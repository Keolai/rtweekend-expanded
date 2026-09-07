#ifndef PHYSICS_LAYER_H
#define PHYSICS_LAYER_H

#include "physics/sim.h"

#include <iostream>
#include <map>           // Required for std::map
#include <unordered_map> // Required for std::unordered_map

class physics_layer
{
public:
    physics_layer() {}

    void step(double delta)
    {
        simulation.step(delta); // delta should be in ms
    }

    int add_sphere_to_world(const vec3 &pos, double radius)
    {
          auto new_sphere = std::make_shared<phy_sphere>(pos, radius);
        new_sphere->rigid = false;

        simulation.world.add(new_sphere);
        return new_sphere->id;
    }

    void add_force(const vec3 &direction, double strength)
    {
        auto new_force = std::make_shared<force>();

        new_force->direction = direction;
        new_force->strength = strength;

        simulation.forces.push_back(new_force);
    }

    void connect_objects(int renderId, int physicsId){
        idMap.insert({renderId,physicsId});
    }

private:
    sim simulation = sim();

    std::map<int,int> idMap; //should be renderId, then physicsId
};

#endif