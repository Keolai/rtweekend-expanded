#ifndef PHYSICS_LAYER_H
#define PHYSICS_LAYER_H

#include "physics/sim.h"
#include "hittable_list.h"

#include <iostream>
#include <map>           // Required for std::map
#include <unordered_map> // Required for std::unordered_map

#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

class physics_layer
{
public:
    physics_layer() {}

    void step(double delta)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        simulation.step(delta);
    }

    int add_sphere_to_world(const vec3 &pos, double radius, bool is_rigid)
    {
        auto new_sphere = std::make_shared<phy_sphere>(pos, radius);
        new_sphere->rigid = is_rigid;

        simulation.world.add(new_sphere);
        return new_sphere->id;
    }

    void add_force(const vec3 &direction, double strength)
    {
        auto new_force = std::make_shared<force>();

        new_force->direction = direction;
        new_force->strength = strength;

        simulation.forces.push_back(new_force);
        return;
    }

    void add_wind_resistance(double coefficient, double area){
        simulation.forces.push_back(std::make_shared<wind_resistance>(0.47, pi * 1.0 * 1.0));
    }

    void connect_objects(int renderId, int physicsId){
        idMap.insert({renderId,physicsId});
        simulation.add_objects_to_map();
    }

    void set_render_objects(hittable_list &world){
        render_world = world;
    }

    void update_render(){
        std::lock_guard<std::mutex> lock(state_mutex_);
        for(int i = 0; i < render_world.objects.size(); i++){
            auto cur_render_object = render_world.objects[i];
            int cur_render_id = cur_render_object->id;
            int cur_physics_id = idMap[cur_render_id];
             //printf("render id %d -> physics id %d\n", cur_render_id, cur_physics_id);
            if (simulation.object_is_mapped(cur_physics_id)){
                vec3 new_position = simulation.object_position(cur_physics_id);
                cur_render_object->position(new_position);
            }
             //printf("NEW POSITION: %f, %f, %f\n",new_position.x(),new_position.y(),new_position.z());
        }
    }

private:
    sim simulation = sim();
    hittable_list render_world = hittable_list();

    std::map<int,int> idMap; //should be renderId, then physicsId
    std::mutex state_mutex_;   // now owned here, not a free global
};

#endif