#ifndef SIM_H
#define SIM_H

#include "hittable_list.h"
#include "objects/force.h"
#include "objects/sphere.h"
#include "objects/tri.h"

class sim
{
public:
    int cur_step = 0;
    hittable_list world;
    std::vector<shared_ptr<force>> forces;

    int step(double dt) // dt should be ms;
    {

        for (int i = 0; i < world.size(); i++)
        {
            // iterate through list
            hittable cur_object = world.objects[i];

            if (!cur_object.rigid) // object can move
            {
                cur_object.update_state(); // copy new state to old state
                state new_state = state();
                copy(cur_object.current_state,new_state);

                vec3 net_force = vec3(0.);
                for (int j = 0; j < forces.size(); j++)
                {
                    // apply forces/move
                    std::shared_ptr<force> cur_force = forces[j];
                    net_force += cur_force.get_force(new_state.position, cur_object.mass);
                }

                // Newton's second law
                new_state.acceleration = net_force / cur_object.mass;
                // Integrate velocity
                new_state.velocity += new_state.acceleration * (dt/1000);
                // Integrate position
                new_state.position += new_state.velocity * (dt/1000);

                // check for collision
                // react to collision
            }
        }

        return 0;
    }

    void set_world(hittable_list &new_world)
    {
        world = new_world;
    }

    void set_forces(std::make_shared<forces> &force_list)
    {
        forces = force_list;
    }
};
#endif