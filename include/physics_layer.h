#ifndef PHYSICS_LAYER_H
#define PHYSICS_LAYER_H

#include "physics/sim.h"

class physics_layer
{
public:
    physics_layer() {}

    void step(double delta)
    {
        simulation.step(delta); // delta should be in ms
    }

    void add_sphere_to_world(const vec3 &pos, double radius)
    {
        phy_sphere new_sphere = phy_sphere(pos, radius);
        new_sphere.rigid = false;

        simulation.world.add(std::make_shared<phy_sphere>(new_sphere));
    }

    void add_force(const vec3 &direction, double strength)
    {
        auto new_force = std::make_shared<force>();

        new_force->direction = direction;
        new_force->strength = strength;

        simulation.forces.push_back(new_force);
    }

private:
    sim simulation = sim();
};

#endif