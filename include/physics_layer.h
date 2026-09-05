#ifndef PHYSICS_LAYER_H
#define PHYSICS_LAYER_H

#include "physics/sim.h"

class physics_layer
{
public:
    physics_layer() {}

    void step(double delta){
        simulation.step(delta); //delta should be in ms
    }

    void add_sphere_to_world(const vec3 &pos, double radius){
        phy_sphere new_sphere = phy_sphere(pos,radius);

        simulation.world.add(std::make_shared<phy_sphere>(new_sphere));
    }

private:
    sim simulation = sim();
};

#endif