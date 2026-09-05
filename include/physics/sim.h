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

    double time_step_ms = 30;

    int step()
    {

        for (int i = 0; i < world.size(); i++)
        {
            // iterate through list
            hittable cur_object = world.objects[i];

            if (!cur_object.rigid)
            {
                for (int j = 0; j < forces.size(); j++){
                    // apply forces/move
                }
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

    void set_forces(std::make_shared<forces> &force_list){
        forces = force_list;
    }
};
#endif