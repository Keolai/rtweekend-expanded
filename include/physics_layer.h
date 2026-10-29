#ifndef PHYSICS_LAYER_H
#define PHYSICS_LAYER_H

#include "physics/sim.h"
#include "hittable_list.h"
#include "soft_mesh.h"

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
    double steps = 0;
    double total_ns = 0;

    void step(double delta)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        steps++;
        total_ns += simulation.step(delta);
    }

    double get_steps(){
        return steps;
    }

    int add_sphere_to_world(const vec3 &pos, double radius, bool is_static)
    {
        auto new_sphere = std::make_shared<phy_sphere>(pos, radius);
        new_sphere->is_static = is_static;

        simulation.world.add(new_sphere);
        return new_sphere->id;
    }

    int add_sphere_to_world(const vec3 &pos,const vec3 &v, double radius)
    {
        auto new_sphere = std::make_shared<phy_sphere>(pos,v, radius);
        new_sphere->is_static = false;

        simulation.world.add(new_sphere);
        return new_sphere->id;
    }

    int add_mesh_to_world(const std::string &file_path, bool is_static){
        auto new_mesh = std::make_shared<model>(file_path);
        new_mesh->is_static = is_static;

        simulation.world.add(new_mesh);
        new_mesh->update_tri_id(new_mesh->id); //IMPORTANT
        return new_mesh->id;
    }

    // returned so we can mess with params
    std::shared_ptr<soft_model> add_soft_model_to_world(const std::string &file_path, double node_mass = 0.5, bool use_shear = false)
    {
        auto new_body = std::make_shared<soft_model>(file_path, node_mass, use_shear);
        simulation.add_soft_body(new_body);
        return new_body;
    }

    std::shared_ptr<soft_model> add_cloth_to_world(double width, double depth, int res_x, int res_z, double node_mass = 0.5)
    {
        auto new_body = std::make_shared<soft_model>(width, depth, res_x, res_z, node_mass);
        simulation.add_soft_body(new_body);
        return new_body;
    }

    //this is weird, maybe change to fit the other connector functions but who cares tbh
    std::shared_ptr<soft_mesh> connect_soft_model(std::shared_ptr<soft_model> body, std::shared_ptr<material> mat, hittable_list &world)
    {
        auto render_mesh = std::make_shared<soft_mesh>(body->faces, body->face_uvs, body->nodes, mat);
        world.add(render_mesh);
        soft_links.push_back(std::make_pair(render_mesh, body->id));
        return render_mesh;
    }

    void set_soft_integrator(soft_integrator integrator)
    {
        simulation.integrator = integrator;
    }

    void add_force(const vec3 &direction, double strength)
    {
        auto new_force = std::make_shared<force>();

        new_force->direction = direction;
        new_force->strength = strength;

        simulation.forces.push_back(new_force);
        return;
    }

    //vec3 position, vec3 axis, double field_strength
    void add_vortex_force(const vec3 &pos, const vec3 &axis, double strength){
        auto new_force = std::make_shared<vortex_force>(pos,axis,strength);
        simulation.forces.push_back(new_force);
        return;
    }

    // range_force(vec3 source, double target_distance, double strength)
    void add_range_force(const vec3 &pos, double distance, double strength){
        auto new_force = std::make_shared<range_force>(pos,distance,strength);
        simulation.forces.push_back(new_force);
        return;
    }

    void add_point_force(const vec3 &pos, double strength){
        auto new_force = std::make_shared<point_force>();

        new_force->strength = strength;
        new_force->position = pos;

        simulation.forces.push_back(new_force);
        return;
    }

    void add_wind_resistance(double coefficient, double area){
        simulation.forces.push_back(std::make_shared<wind_resistance>(0.47, pi * 1.0 * 1.0));
    }

    particle_emitter& add_particle_emitter(const vec3 &position, const vec3 &direction, double spread, double speed, double flow){
        particle_emitter new_emitter = particle_emitter(position, direction, spread, speed, 20, flow);
        simulation.emitters.emplace_back(new_emitter);
        return simulation.emitters.back();
    }

     particle_emitter& add_particle_emitter(const vec3 &position, const vec3 &direction, double spread, double speed, double flow, bool is_flock){
        particle_emitter new_emitter = particle_emitter(position, direction, spread, speed, 20, flow);
        new_emitter.is_flock = is_flock;
        new_emitter.flock.rest_density = estimate_rest_density(new_emitter.flock.neighbor_radius, 0.27, new_emitter.flock.particle_mass); 
        simulation.emitters.emplace_back(new_emitter);
        return simulation.emitters.back();
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
        for (auto &link : soft_links)
        {
            const std::vector<soft_node> *nodes = simulation.soft_nodes(link.second);
            if (nodes)
            {
                link.first->set_positions(*nodes);
            }
        }
        for(int i = 0; i < render_world.objects.size(); i++){
            auto cur_render_object = render_world.objects[i];
            cur_render_object->refresh();
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

    int get_step(){
        return simulation.cur_step;
    }

private:
    sim simulation = sim();
    hittable_list render_world = hittable_list();

    std::map<int,int> idMap; //should be renderId, then physicsId
    std::vector<std::pair<std::shared_ptr<soft_mesh>, int>> soft_links; // render mesh, physics id
    std::mutex state_mutex_;   // now owned here, not a free global
};

#endif