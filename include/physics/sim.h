#ifndef SIM_H
#define SIM_H

#include "hittable_list.h"
#include "objects/force.h"
#include "objects/sphere.h"
#include "objects/model.h"
#include "objects/tri.h"
#include "utilities/phy_bvh.h"

#include <map>           // Required for std::map
#include <unordered_map> // Required for std::unordered_map

#define RESTING_THRESHOLD 0.4
#define MS_PER_SEC 1000

class sim
{
public:
    sim() {}
    int cur_step = 0;
    phy_hittable_list world;
    std::shared_ptr<phy_bvh_node> world_bvh;

    std::vector<shared_ptr<force>> forces;

    int step(double dt) // dt should be ms;
    {
        world_bvh = make_shared<phy_bvh_node>(
            world.objects,
            0,
            world.objects.size());
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

                vec3 net_force = get_net_force(new_state.position, new_state.velocity, cur_object->mass);
                // Newton's second law
                new_state.acceleration = net_force / cur_object->mass;
                // Integrate velocity
                new_state.velocity += new_state.acceleration * (dt / MS_PER_SEC);
                // Integrate position
                new_state.position += new_state.velocity * (dt / MS_PER_SEC);
                // printf("NEW POSITION: %f, %f, %f\n",new_state.position.x(),new_state.position.y(),new_state.position.z());
                copy(new_state, cur_object->next_state);
            }
            else if (cur_object && cur_object->is_static)
            {
                copy(cur_object->current_state, cur_object->next_state);
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
                std::vector<vec3> object_vertices = cur_object->get_vertices();
                vec3 tmp_pos = cur_object->next_state.position;
                vec3 tmp_vel = cur_object->next_state.velocity;
                double highest_penetration = (cur_object->next_state.position - cur_object->current_state.position).length();

                bool test_object_is_static = false;
                std::shared_ptr<phy_hittable> saved_test_object;

                for (int k = 0; k < object_vertices.size(); k++) // test each vertex;
                {
                    vec3 direction = (cur_object->next_state.position) - (cur_object->current_state.position);
                    double length = direction.length();
                    direction = unit_vector(direction);
                    ray r = ray(cur_object->current_state.position + object_vertices[k] + (cur_object->hit_adjuster() * direction), direction); // current state is updated to new potential velocity
                    phy_hit_record rec;
                    phy_hit_record temp_rec;
                    bool hit_anything = false;
                    bool embedded = false;

                    auto closest_so_far = length + cur_object->hit_adjuster();
                    for (int j = 0; j < *world_bvh.size(); j++)
                    {
                        if(*world_bvh->hit(r,interval(0.001, closest_so_far), temp_rec)){
                        auto test_object = std::make_shared<hittable>(temp_rec.hit_object); //just assign whatever was hit to test

                        // test_object_is_static = test_object->is_static;
                        if (test_object && test_object->id != cur_object->id)
                        {
                            auto test_sphere = std::static_pointer_cast<phy_sphere>(test_object);
                            auto cur_sphere = std::static_pointer_cast<phy_sphere>(cur_object);

                            double distance = (test_sphere->next_state.position - cur_sphere->next_state.position).length();

                            if (typeid(*test_object) == typeid(*cur_object) && typeid(*cur_object) == typeid(phy_sphere))
                            {
                                if (distance < (test_sphere->get_radius() + cur_sphere->get_radius()))
                                {
                                    // handle sphere-sphere collision here
                                    double t_hit;
                                    vec3 p1_at_hit = test_sphere->current_state.position + t_hit * (test_sphere->next_state.position - test_sphere->current_state.position);
                                    vec3 p2_at_hit = cur_sphere->current_state.position + t_hit * (cur_sphere->next_state.position - cur_sphere->current_state.position);

                                    vec3 collision_normal = unit_vector(p2_at_hit - p1_at_hit);                    // points from sphere1 toward sphere2
                                    vec3 contact_point = p1_at_hit + collision_normal * test_sphere->get_radius(); // point on sphere1's surface where contact occurred
                                    test_object_is_static = test_object->is_static;                                // NEED
                                    if (!test_object->is_static && sphere_sphere_toi(test_sphere->next_state.position, test_sphere->next_state.velocity, test_sphere->get_radius(),
                                                                                     cur_sphere->next_state.velocity, cur_sphere->next_state.position, cur_sphere->get_radius(), t_hit))
                                    {
                                        sphere_to_sphere_collision(*cur_sphere, *test_sphere, t_hit, dt);
                                    }
                                    hit_anything = false;
                                }
                            }
                            else
                            {
                                if (test_object->is_inside(r))
                                {
                                    embedded = true;
                                    if (test_object->is_static)
                                    {
                                        // printf("embedded!\n");
                                        // really? maybe switch to modifying rec
                                        double restitution = cur_object->restitution;
                                        cur_object->next_state.position = cur_object->closest_point_on_surface(r.origin());
                                        vec3 out_dir = unit_vector(r.origin() - cur_object->next_state.position);
                                        cur_object->next_state.velocity = cur_object->next_state.velocity - (1.0 + restitution) * dot(cur_object->next_state.velocity, out_dir) * out_dir;
                                        cur_object->next_state.position += out_dir * (cur_object->hit_adjuster() + 0.001);
                                    }
                                    if (!test_object->is_static)
                                    {
                                        // printf("running\n");
                                        saved_test_object = test_object;
                                    }
                                }
                                else if (test_object->hit(r, interval(0.001, closest_so_far), temp_rec))
                                {
                                    // printf("candidate hit: t=%f  highest_penetration+0.01=%f  is_static=%d\n",
                                    //    temp_rec.t, highest_penetration + 0.01, test_object->is_static);
                                    if (temp_rec.t < (highest_penetration + 0.01))
                                    {
                                        // printf("hit\n");
                                        hit_anything = true;
                                        closest_so_far = temp_rec.t;
                                        rec = temp_rec;
                                        test_object_is_static = test_object->is_static; // NEED
                                        saved_test_object = test_object;
                                    }
                                    embedded = false;
                                    // printf("rec.normal: (%f,%f,%f)  true_outward: (%f,%f,%f)\n", rec.normal.x(), rec.normal.y(), rec.normal.z(), true_outward.x(), true_outward.y(), true_outward.z());
                                }
                            }
                        }
                    }
                    }
                    double t_fraction = (length > 1e-9) ? (closest_so_far / length) : 0.0;
                    double remaining_dt = dt * (1.0 - t_fraction);
                    if (hit_anything)
                    {
                        if (test_object_is_static) // bounc normally
                        {
                            static_collision_reaction(cur_object, rec, tmp_vel, tmp_pos, remaining_dt);
                        }
                        else
                        {
                            dynamic_collision_reaction(cur_object, saved_test_object, rec, tmp_vel, tmp_pos, remaining_dt);
                        }
                    }
                    if (!test_object_is_static && embedded) // both are non-static
                    {
                        // printf("not-static\n");
                        // printf("%f%f%f\n", cur_object->next_state.velocity.x(), cur_object->next_state.velocity.y(), cur_object->next_state.velocity.z());
                        dynamic_collision_reaction(cur_object, saved_test_object, rec, tmp_vel, tmp_pos, remaining_dt);
                    }
                } // end of vertex loop
            }
        }
        cur_step++;
        return 0;
    }

    void
    set_world(phy_hittable_list &new_world)
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

    vec3 get_net_force(vec3 &position, vec3 &velocity, double mass)
    {
        vec3 net_force = vec3(0.);
        for (int j = 0; j < forces.size(); j++)
        {
            // apply forces/move
            auto cur_force = forces[j];
            if (cur_force)
            {
                net_force += cur_force->get_force(position, velocity, mass);
            }
        }
        return net_force;
    }

    void static_collision_reaction(std::shared_ptr<phy_hittable> &cur_object, phy_hit_record &rec,
                                   vec3 &tmp_vel, vec3 &tmp_pos, double remaining_dt)
    {
        double restitution = cur_object->restitution;
        double v_normal = dot(tmp_vel, rec.normal); // movement along velocity
        vec3 v_normal_vec = v_normal * rec.normal;
        vec3 v_tangent = tmp_vel - v_normal_vec; // movement along tangent

        // cur_object->next_state.velocity = cur_object->next_state.velocity * restitution * v_normal * rec.normal;
        vec3 new_normal_vec = -restitution * v_normal_vec;
        vec3 new_tangent_vec = v_tangent * (1.0 - cur_object->friction);

        cur_object->next_state.velocity = new_normal_vec + new_tangent_vec;
        cur_object->next_state.position = tmp_pos + (rec.normal * 0.01); // this line is causing issues

        if (remaining_dt > 0.0)
        {
            // integrate
            cur_object->next_state.position += cur_object->next_state.velocity * (remaining_dt / MS_PER_SEC);
        }
    }

    void dynamic_collision_reaction(std::shared_ptr<phy_hittable> &cur_object, std::shared_ptr<phy_hittable> &saved_test_object, phy_hit_record &rec,
                                    vec3 &tmp_vel, vec3 &tmp_pos, double remaining_dt)
    {
        cur_object->next_state.velocity = saved_test_object->next_state.velocity;
        // cur_object->next_state.position = tmp_pos + (rec.normal * 0.1); // this line is causing issues
        saved_test_object->next_state.velocity = tmp_vel;
        // saved_test_object->next_state.position = tmp_pos + (-rec.normal * 0.01); //issue
        if (remaining_dt > 0.0)
        {
            // integrate
            cur_object->next_state.position += cur_object->next_state.velocity * (remaining_dt / MS_PER_SEC);
            saved_test_object->next_state.position += saved_test_object->next_state.velocity * (remaining_dt / MS_PER_SEC);
        }
    }

    void sphere_to_sphere_collision(phy_sphere &cur_sphere, phy_sphere &test_sphere, double t_hit, double dt)
    {
        double dt_frac = t_hit * dt / MS_PER_SEC; // scaled time-into-frame, in seconds

        vec3 p1_at_hit = cur_sphere.current_state.position + dt_frac * cur_sphere.current_state.velocity;
        vec3 p2_at_hit = test_sphere.current_state.position + dt_frac * test_sphere.current_state.velocity;

        vec3 collision_normal = unit_vector(p2_at_hit - p1_at_hit); // points from cur_sphere toward test_sphere
        vec3 contact_point = p1_at_hit + collision_normal * cur_sphere.get_radius();

        // "1" = cur_sphere, "2" = test_sphere, consistently:
        double v1n_scalar = dot(cur_sphere.next_state.velocity, collision_normal);
        double v2n_scalar = dot(test_sphere.next_state.velocity, collision_normal);
        vec3 v1t = cur_sphere.next_state.velocity - v1n_scalar * collision_normal;
        vec3 v2t = test_sphere.next_state.velocity - v2n_scalar * collision_normal;

        double m1 = cur_sphere.mass;
        double m2 = test_sphere.mass;
        double restitution = std::min(cur_sphere.restitution, test_sphere.restitution);

        double new_v1n = v1n_scalar - (1.0 + restitution) * (m2 / (m1 + m2)) * (v1n_scalar - v2n_scalar);
        double new_v2n = v2n_scalar + (1.0 + restitution) * (m1 / (m1 + m2)) * (v1n_scalar - v2n_scalar);

        cur_sphere.next_state.velocity = v1t + new_v1n * collision_normal;
        test_sphere.next_state.velocity = v2t + new_v2n * collision_normal;

        double overlap = (cur_sphere.get_radius() + test_sphere.get_radius()) - (p2_at_hit - p1_at_hit).length();
        if (overlap > 0.0)
        {
            double total_mass = m1 + m2;
            double push1 = overlap * (m2 / total_mass);
            double push2 = overlap * (m1 / total_mass);

            cur_sphere.next_state.position = p1_at_hit - collision_normal * push1;
            test_sphere.next_state.position = p2_at_hit + collision_normal * push2;
        }
        else
        {
            cur_sphere.next_state.position = p1_at_hit;
            test_sphere.next_state.position = p2_at_hit;
        }

        double remaining_dt = dt * (1.0 - t_hit);
        if (remaining_dt > 0.0)
        {
            cur_sphere.next_state.position += cur_sphere.next_state.velocity * (remaining_dt / MS_PER_SEC);
            test_sphere.next_state.position += test_sphere.next_state.velocity * (remaining_dt / MS_PER_SEC);
        }
    }

    bool sphere_sphere_toi(
        const vec3 &p1, const vec3 &v1, double r1,
        const vec3 &p2, const vec3 &v2, double r2,
        double &t_hit) // output: fraction of the frame [0,1] at first contact
    {
        vec3 p_rel = p1 - p2;
        vec3 v_rel = v1 - v2;
        double r_sum = r1 + r2;

        double a = dot(v_rel, v_rel);
        double b = 2.0 * dot(p_rel, v_rel);
        double c = dot(p_rel, p_rel) - r_sum * r_sum;

        // already overlapping at the start of the frame
        if (c < 0.0)
        {
            t_hit = 0.0;
            return true;
        }

        // not moving relative to each other — can't newly collide this frame
        if (a < 1e-12)
            return false;

        double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0)
            return false; // never gets close enough this frame

        double sqrt_disc = std::sqrt(discriminant);
        double t0 = (-b - sqrt_disc) / (2.0 * a);
        double t1 = (-b + sqrt_disc) / (2.0 * a);

        // want the earliest root that falls within this frame's time range
        if (t0 >= 0.0 && t0 <= 1.0)
        {
            t_hit = t0;
            return true;
        }
        if (t1 >= 0.0 && t1 <= 1.0)
        {
            t_hit = t1;
            return true;
        }

        return false; // closest approach happens outside this frame's time window
    }
};
#endif