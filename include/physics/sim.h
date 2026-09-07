#ifndef SIM_H
#define SIM_H

#include "hittable_list.h"
#include "objects/force.h"
#include "objects/sphere.h"
#include "objects/tri.h"

#include <map>           // Required for std::map
#include <unordered_map> // Required for std::unordered_map

class sim
{
public:
    sim(){}
    int cur_step = 0;
    phy_hittable_list world;

    std::vector<shared_ptr<force>> forces;

    int step(double dt) // dt should be ms;
    {
        //printf("cur step: %d\n",cur_step);
        for (int i = 0; i < world.size(); i++)
        {
            // iterate through list
            auto cur_object = world.objects[i];

            if (cur_object && !cur_object->rigid) // object can move
            {
                cur_object->update_state(); // copy new state to old state
                state new_state = state();
                copy(cur_object->current_state,new_state);

                vec3 net_force = vec3(0.);
                for (int j = 0; j < forces.size(); j++)
                {
                    // apply forces/move
                    std::shared_ptr<force> cur_force = forces[j];
                    if (cur_force){
                         net_force += cur_force->get_force(new_state.position, cur_object->mass);
                    }
                }

                // Newton's second law
                new_state.acceleration = net_force / cur_object->mass;
                // Integrate velocity
                new_state.velocity += new_state.acceleration * (dt/1000);
                // Integrate position
                new_state.position += new_state.velocity * (dt/1000);
                //printf("NEW POSITION: %f, %f, %f\n",new_state.position.x(),new_state.position.y(),new_state.position.z());
                copy(new_state,cur_object->next_state);
                // check for collision
                // react to collision
            }
        }
        cur_step++;
        return 0;
    }

    void set_world(phy_hittable_list &new_world)
    {
        world = new_world;
    }

    void set_forces(std::vector<std::shared_ptr<force>> &force_list)
    {
        forces = force_list;
    }

    void add_objects_to_map(){
        for (int i = 0; i < world.objects.size(); i++){
            std::shared_ptr<phy_hittable> cur_object = world.objects[i];
            idMap[cur_object->id] = cur_object;
        }
    }

    vec3 object_position(int id){
        auto cur_object = idMap[id];
        if (!cur_object){
            printf("ID NOT FOUND: %d\n",id);
        }
        state new_state = cur_object->current_state;
        //printf("Physics POSITION: %f, %f, %f\n",new_state.position.x(),new_state.position.y(),new_state.position.z());
        return cur_object->current_state.position;
    }

    private:
        std::map<int,std::shared_ptr<phy_hittable>> idMap;
};
#endif