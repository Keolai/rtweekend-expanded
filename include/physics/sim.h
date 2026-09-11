#ifndef SIM_H
#define SIM_H

#include "hittable_list.h"
#include "objects/force.h"
#include "objects/sphere.h"
#include "objects/tri.h"

#include <map>           // Required for std::map
#include <unordered_map> // Required for std::unordered_map

#define RESTING_THRESHOLD 0.4

class sim
{
public:
    sim() {}
    int cur_step = 0;
    phy_hittable_list world;

    std::vector<shared_ptr<force>> forces;

    int step(double dt) // dt should be ms;
    {
        // printf("cur step: %d\n",cur_step);
        for (int i = 0; i < world.size(); i++)
        {
            // iterate through list
            auto cur_object = world.objects[i];

            if (cur_object && !cur_object->is_static) // object can move
            {
                cur_object->update_state(); // copy new state to old state
                state new_state = state();
                copy(cur_object->current_state, new_state);

                vec3 net_force = vec3(0.);
                for (int j = 0; j < forces.size(); j++)
                {
                    // apply forces/move
                    auto cur_force = forces[j];
                    if (cur_force)
                    {
                        net_force += cur_force->get_force(new_state.position, new_state.velocity, cur_object->mass);
                    }
                }
                // Newton's second law
                new_state.acceleration = net_force / cur_object->mass;
                // Integrate velocity
                new_state.velocity += new_state.acceleration * (dt / 1000);
                // Integrate position
                new_state.position += new_state.velocity * (dt / 1000);
                // printf("NEW POSITION: %f, %f, %f\n",new_state.position.x(),new_state.position.y(),new_state.position.z());
                copy(new_state, cur_object->next_state);
            } else if (cur_object && cur_object->is_static){
                copy(cur_object->current_state,cur_object->next_state);
            }
        }
        // loop over again, as potential new states have been populated
        for (int i = 0; i < world.size(); i++)
        {
            // check for collision
            // react to collision
            auto cur_object = world.objects[i];
            if (cur_object && !cur_object->is_static) // object can move
            {
                vec3 direction = cur_object->next_state.position - cur_object->current_state.position;
                double length = direction.length();
                direction = unit_vector(direction);
                ray r = ray(cur_object->current_state.position, direction);
                phy_hit_record rec;
                phy_hit_record temp_rec;
                bool hit_anything = false;
                bool embedded = false;

                auto closest_so_far = length + cur_object->hit_adjuster();
                for (int j = 0; j < world.size(); j++)
                {
                    auto test_object = world.objects[j];
                    if (test_object && test_object->id != cur_object->id)
                    {
                        if (test_object->is_inside(r))
                        {
                            embedded = true;
                            // really? maybe switch to modifying rec
                            double restitution = cur_object->restitution;
                            cur_object->next_state.position = cur_object->closest_point_on_surface(r.origin());
                            vec3 out_dir = unit_vector(r.origin() - cur_object->next_state.position);
                            cur_object->next_state.position += out_dir * (cur_object->hit_adjuster() + 0.001);
                            cur_object->next_state.velocity = cur_object->next_state.velocity - (1.0 + restitution) * dot(cur_object->next_state.velocity, out_dir) * out_dir;

                            break;
                        }
                        else if (test_object->hit(r, interval(0.001, closest_so_far), temp_rec))
                        {
                            hit_anything = true;
                            closest_so_far = temp_rec.t;
                            rec = temp_rec;
                            vec3 true_outward = unit_vector(rec.p - test_object->next_state.position);
                            //printf("rec.normal: (%f,%f,%f)  true_outward: (%f,%f,%f)\n", rec.normal.x(), rec.normal.y(), rec.normal.z(), true_outward.x(), true_outward.y(), true_outward.z());
                        }
                    }
                }
                if (hit_anything && !embedded)
                {
                    double restitution = cur_object->restitution;
                    double v_normal = dot(cur_object->next_state.velocity, rec.normal);
                    //printf("v_normal: %f  branch: %s\n", v_normal, (std::abs(v_normal) < RESTING_THRESHOLD) ? "resting" : "bounce");
                    if (std::abs(v_normal) < RESTING_THRESHOLD)
                    {
                        //sliding
                        cur_object->next_state.velocity = cur_object->next_state.velocity - (v_normal * rec.normal);
                    }
                    else
                    {
                        cur_object->next_state.velocity = cur_object->next_state.velocity - (1.0 + restitution) * v_normal * rec.normal;
                    }

                    cur_object->next_state.position = rec.p + (rec.normal * (0.001 + cur_object->hit_adjuster()));

                    double t_fraction = (length > 1e-9) ? (closest_so_far / length) : 0.0;
                    double remaining_dt = dt * (1.0 - t_fraction);
                    if (remaining_dt > 0.0)
                    {
                        cur_object->next_state.position += cur_object->next_state.velocity * (remaining_dt / 1000.0);
                    }
                }
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

    void add_objects_to_map()
    {
        for (int i = 0; i < world.objects.size(); i++)
        {
            std::shared_ptr<phy_hittable> cur_object = world.objects[i];
            idMap[cur_object->id] = cur_object;
        }
    }

    vec3 object_position(int id)
    {
        auto cur_object = idMap[id];
        if (!cur_object)
        {
            printf("ID NOT FOUND: %d\n", id);
            return vec3(0.); // fix this
        }
        state new_state = cur_object->current_state;
        // printf("Physics POSITION: %f, %f, %f\n",new_state.position.x(),new_state.position.y(),new_state.position.z());
        return cur_object->current_state.position;
    }

    bool object_is_mapped(int id)
    {
        auto cur_object = idMap[id];
        return (bool)cur_object;
    }

private:
    std::map<int, std::shared_ptr<phy_hittable>> idMap;
};
#endif