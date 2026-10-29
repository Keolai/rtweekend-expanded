#ifndef SIM_H
#define SIM_H

#include "hittable_list.h"
#include "objects/force.h"
#include "objects/sphere.h"
#include "objects/model.h"
#include "objects/tri.h"
#include "objects/particle.h"
#include "objects/soft_model.h"

#include "utilities/phy_bvh.h"
#include "utilities/particle_bvh.h"

#include "flock_behaviors/boids.h"
#include "flock_behaviors/sph.h"
#include "flock_behaviors/gross.h"

#include <map>           // Required for std::map
#include <unordered_map> // Required for std::unordered_map
#include <chrono>

#define RESTING_THRESHOLD 0.4
#define PARTICLE_MASS 0.1
#define MS_PER_SEC 1000

#define MAX_EMITTERS 5
#define NEIGHBOR_RADIUS 0.5
#define SOFT_LOG_INTERVAL 10 // steps between soft body energy prints, 0 = off
#define SOFT_SUBSTEPS 8

enum class soft_integrator
{
    forward_euler,       // position uses the old velocity, explodes the fastest
    semi_implicit_euler, // velocity first, same order as the rigid body loop
    runge_kutta_4,       // needs the whole body
    heuns_method,        // RK2 / trapezoidal, needs the whole body
    velocity_verlet      // leapfrog (kick-drift-kick), needs the whole body
};

class sim
{
public:
    sim() { emitters.reserve(MAX_EMITTERS); }
    int cur_step = 0;
    phy_hittable_list world;
    std::shared_ptr<phy_bvh_node> world_bvh;
    std::vector<particle_emitter> emitters;

    std::vector<shared_ptr<force>> forces;
    std::vector<std::shared_ptr<soft_model>> soft_bodies; // kept out of world so nodes don't hit their own triangles ehh
    soft_integrator integrator = soft_integrator::forward_euler;

    double step(double dt) // dt should be ms;
    {
        auto start = std::chrono::high_resolution_clock::now();
        world_bvh = make_shared<phy_bvh_node>( // is this slow?
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
                    // for (int j = 0; j < *world_bvh.size(); j++)
                    // {
                    if (world_bvh->hit(r, interval(0.001, closest_so_far), temp_rec))
                    {
                        auto test_object = temp_rec.hit_object; // just assign whatever was hit to test
                        // phy_hittable* test_object = rec.hit_object;
                        //  test_object_is_static = test_object->is_static;
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
                        //}
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
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
            stop - start);
        rigid_soft_collision(dt);
        particle_step(dt);
        soft_step(dt);
        // std::cout << duration.count()  << " ns\n";
        cur_step++;
        return duration.count();
    }

    // rigid->soft
    void rigid_soft_collision(double dt)
    {
        for (int i = 0; i < world.size(); i++)
        {
            auto cur_object = world.objects[i];
            if (!cur_object || cur_object->is_static)
            {
                continue;
            }

            std::vector<vec3> object_vertices = cur_object->get_vertices();
            for (int s = 0; s < soft_bodies.size(); s++)
            {
                soft_model &soft_body = *soft_bodies[s];
                for (int k = 0; k < object_vertices.size(); k++)
                {
                    vec3 vertex_pos = cur_object->current_state.position + object_vertices[k];
                    vec3 vertex_next_pos = cur_object->next_state.position + object_vertices[k];
                    vec3 direction = vertex_next_pos - vertex_pos;
                    double length = direction.length();
                    if (length < 1e-12)
                    {
                        continue;
                    }

                    ray r = ray(vertex_pos, direction / length);
                    phy_hit_record rec;
                    if (soft_body.hit(r, interval(0.001, length + 0.01), rec))
                    {
                        rigid_soft_response(*cur_object, soft_body, rec);
                    }
                }
            }
        }
    }

    void rigid_soft_response(phy_hittable &rigid, soft_model &soft, phy_hit_record &rec)
    {
        auto hit_tri = std::static_pointer_cast<phy_soft_tri>(rec.hit_object);
        soft_node *corners[3];
        double weights[3] = {rec.bay_coord.x(), rec.bay_coord.y(), rec.bay_coord.z()};

        vec3 contact_vel = vec3(0);
        double tri_inv_mass = 0.0;
        for (int i = 0; i < 3; i++)
        {
            corners[i] = &soft.nodes[hit_tri->index(i)];
            contact_vel += weights[i] * corners[i]->vel;
            if (!corners[i]->pinned)
            {
                tri_inv_mass += weights[i] * weights[i] / corners[i]->mass;
            }
        }

        double rigid_inv_mass = 1.0 / rigid.mass;
        double inv_mass_sum = rigid_inv_mass + tri_inv_mass;
        if (inv_mass_sum <= 0.0)
        {
            return;
        }

        vec3 normal = rec.normal; // points from the soft triangle toward the incoming rigid vertex
        vec3 rel_vel = rigid.next_state.velocity - contact_vel;
        double v_normal = dot(rel_vel, normal);
        if (v_normal < 0.0) // approaching
        {
            double restitution = std::min(rigid.restitution, soft.restitution);
            double friction = std::max(rigid.friction, soft.friction);
            vec3 v_tangent = rel_vel - v_normal * normal;
            vec3 impulse = (-(1.0 + restitution) * v_normal * normal - friction * v_tangent) / inv_mass_sum;

            rigid.next_state.velocity += impulse * rigid_inv_mass;
            for (int i = 0; i < 3; i++)
            {
                if (!corners[i]->pinned)
                {
                    corners[i]->vel += -impulse * (weights[i] / corners[i]->mass);
                }
            }
        }
        rigid.next_state.position += normal * 0.01;
    }

    void particle_step(double dt)
    { // particles are a lot less expensive than real objects
        for (int i = 0; i < emitters.size(); i++)
        {
            particle_emitter &cur_emitter = emitters[i];
            cur_emitter.step(dt);
            if (!cur_emitter.is_flock) // behave normally
            {
                normal_particle_behavior(cur_emitter, dt);
            }
            else
            {
                // fun behavior!
                flock_particle_behavior(cur_emitter, dt);
            }
        }
    }

    void soft_step(double dt)
    {
        double sub_dt = dt / SOFT_SUBSTEPS;
        for (int s = 0; s < SOFT_SUBSTEPS; s++)
        {
            soft_substep(sub_dt);
        }
        if (SOFT_LOG_INTERVAL > 0 && cur_step % SOFT_LOG_INTERVAL == 0)
        {
            for (auto &body : soft_bodies)
            {
                print_soft_energy(*body);
            }
        }
    }

    void soft_substep(double dt)
    {
        double h = dt / MS_PER_SEC;
        for (int b = 0; b < soft_bodies.size(); b++)
        {
            soft_model &body = *soft_bodies[b];
            std::vector<soft_node> &nodes = body.nodes;

            std::vector<vec3> new_positions;
            integrate_soft_body(body, h, new_positions);

            for (int i = 0; i < nodes.size(); i++)
            {
                if (nodes[i].pinned)
                {
                    nodes[i].vel = vec3(0);
                    continue;
                }
                soft_soft_collision(b, nodes[i], new_positions[i]);
                soft_node_collision(body, nodes[i], new_positions[i]);
            }

            body.update_centroid();
            body.refit();
        }
        for (int b = 0; b < soft_bodies.size(); b++)
        {
            for (int j = 0; j < soft_bodies.size(); j++)
            {
                // edge edge collision pass;
                if (b != j)
                {
                    edge_edge_collision(*soft_bodies[b], *soft_bodies[j]);
                }
            }
        }
    }

    void add_soft_body(std::shared_ptr<soft_model> body)
    {
        body->id = world.cur_id;
        world.cur_id++;
        soft_bodies.push_back(body);
        idMap[body->id] = body;
    }

    const std::vector<soft_node> *soft_nodes(int id)
    {
        for (auto &body : soft_bodies)
        {
            if (body->id == id)
            {
                return &body->nodes;
            }
        }
        return nullptr;
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
        for (auto &body : soft_bodies)
        {
            idMap[body->id] = body;
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

    // the node and the 3 nodes of the hit triangle, weighted by the barycentric coords of the contact.
    void soft_soft_collision(int body_index, soft_node &n, vec3 &new_pos)
    {
        vec3 direction = new_pos - n.pos;
        double length = direction.length();
        if (length < 1e-12) // too small
        {
            return;
        }
        ray r = ray(n.pos, direction / length);

        phy_hit_record rec;
        phy_hit_record temp_rec;
        int hit_body = -1; // index of body hit
        double closest_so_far = length + 0.01;
        for (int i = 0; i < soft_bodies.size(); i++)
        {
            if (i != body_index && soft_bodies[i]->hit(r, interval(0.001, closest_so_far), temp_rec))
            {
                hit_body = i;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        if (hit_body < 0)
        {
            return;
        }

        soft_model &self = *soft_bodies[body_index];
        soft_model &other = *soft_bodies[hit_body];
        auto hit_tri = std::static_pointer_cast<phy_soft_tri>(rec.hit_object);
        soft_node *corners[3];
        double weights[3] = {rec.bay_coord.x(), rec.bay_coord.y(), rec.bay_coord.z()};

        vec3 contact_vel = vec3(0);
        double tri_inv_mass = 0.0; // inverse mass of the contact point on the triangle
        for (int i = 0; i < 3; i++)
        {
            corners[i] = &other.nodes[hit_tri->index(i)];
            contact_vel += weights[i] * corners[i]->vel;
            if (!corners[i]->pinned)
            {
                tri_inv_mass += weights[i] * weights[i] / corners[i]->mass;
            }
        }
        double node_inv_mass = n.pinned ? 0.0 : 1.0 / n.mass;
        double inv_mass_sum = node_inv_mass + tri_inv_mass;

        vec3 normal = rec.normal;
        vec3 rel_vel = n.vel - contact_vel;
        double v_normal = dot(rel_vel, normal);
        if (v_normal < 0 && inv_mass_sum > 0) // moving toward each other
        {
            double restitution = std::min(self.restitution, other.restitution);
            double friction = std::max(self.friction, other.friction);
            vec3 v_tangent = rel_vel - v_normal * normal;
            vec3 impulse = (-(1.0 + restitution) * v_normal * normal - friction * v_tangent) / inv_mass_sum;

            n.vel += impulse * node_inv_mass;
            for (int i = 0; i < 3; i++)
            {
                if (!corners[i]->pinned)
                {
                    corners[i]->vel += -impulse * (weights[i] / corners[i]->mass);
                }
            }
        }
        new_pos = rec.p + normal * 0.01;
    }

    // fills node.force from the nodes' current pos and vel
    void compute_soft_forces(soft_model &body)
    {
        std::vector<soft_node> &nodes = body.nodes;
        for (soft_node &n : nodes)
        {
            n.force = get_net_force(n.pos, n.vel, n.mass);
        }
        for (spring &s : body.springs)
        {
            vec3 f = spring_force(s, nodes);
            nodes[s.a].force += f;
            nodes[s.b].force += -f;
        }
        if (body.pressure != 0.0)
        {
            body.add_pressure_forces();
        }
    }

    // updates every node's vel and fills where each node wants to move, collision then decides where it ends up
    void integrate_soft_body(soft_model &body, double h, std::vector<vec3> &new_positions)
    {
        if (integrator == soft_integrator::runge_kutta_4) // instead of a case in the normal integration function, but there is a catcher there
        {
            runge_kutta_four(body, h, new_positions);
            return;
        }
        if (integrator == soft_integrator::heuns_method)
        {
            heuns_method(body, h, new_positions);
            return;
        }
        if (integrator == soft_integrator::velocity_verlet)
        {
            velocity_verlet(body, h, new_positions);
            return;
        }

        compute_soft_forces(body);
        new_positions.resize(body.nodes.size());
        for (int i = 0; i < body.nodes.size(); i++)
        {
            soft_node &n = body.nodes[i];
            new_positions[i] = n.pinned ? n.pos : integrate_soft_node(n, h);
        }
    }

    // per node schemes, they only need the node's own force from the start of the step
    vec3 integrate_soft_node(soft_node &n, double h)
    {
        switch (integrator)
        {
        case soft_integrator::semi_implicit_euler:
            return semi_implicit_euler(n, h);
        case soft_integrator::runge_kutta_4:
            // to catch errors
            assert(false && "runge_kutta_4 is integrated per body, use integrate_soft_body");
            return semi_implicit_euler(n, h);
        case soft_integrator::heuns_method:
            assert(false && "heuns_method is integrated per body, use integrate_soft_body");
            return semi_implicit_euler(n, h);
        case soft_integrator::velocity_verlet:
            assert(false && "velocity_verlet is integrated per body, use integrate_soft_body");
            return semi_implicit_euler(n, h);
        case soft_integrator::forward_euler:
        default:
            return forward_euler(n, h);
        }
    }

    vec3 forward_euler(soft_node &n, double h)
    {
        vec3 acc = n.force / n.mass;
        vec3 new_pos = n.pos + n.vel * h;
        n.vel += acc * h;
        return new_pos;
    }

    vec3 semi_implicit_euler(soft_node &n, double h)
    {
        vec3 acc = n.force / n.mass;
        n.vel += acc * h;
        return n.pos + n.vel * h; // new velocity
    }

    void runge_kutta_four(soft_model &body, double h, std::vector<vec3> &new_positions)
    {
        std::vector<soft_node> &nodes = body.nodes;
        int count = nodes.size();

        std::vector<vec3> start_pos(count), start_vel(count);
        for (int i = 0; i < count; i++)
        {
            start_pos[i] = nodes[i].pos;
            start_vel[i] = nodes[i].vel;
        }

        std::vector<vec3> k_vel[4], k_acc[4]; //velocity and acceleration
        const double stage_step[4] = {0.0, 0.5 * h, 0.5 * h, h};
        for (int stage = 0; stage < 4; stage++)
        {
            for (int i = 0; i < count; i++)
            {
                if (stage > 0 && !nodes[i].pinned)
                {
                    nodes[i].pos = start_pos[i] + k_vel[stage - 1][i] * stage_step[stage];
                    nodes[i].vel = start_vel[i] + k_acc[stage - 1][i] * stage_step[stage];
                }
            }
            compute_soft_forces(body);

            k_vel[stage].resize(count);
            k_acc[stage].resize(count);
            for (int i = 0; i < count; i++)
            {
                k_vel[stage][i] = nodes[i].pinned ? vec3(0) : nodes[i].vel;
                k_acc[stage][i] = nodes[i].pinned ? vec3(0) : nodes[i].force / nodes[i].mass;
            }
        }

        // put the body back at the start so collision raycasts from the old position to the new one
        new_positions.resize(count);
        for (int i = 0; i < count; i++)
        {
            nodes[i].pos = start_pos[i];
            nodes[i].vel = start_vel[i] + (k_acc[0][i] + 2.0 * k_acc[1][i] + 2.0 * k_acc[2][i] + k_acc[3][i]) * (h / 6.0);
            new_positions[i] = start_pos[i] + (k_vel[0][i] + 2.0 * k_vel[1][i] + 2.0 * k_vel[2][i] + k_vel[3][i]) * (h / 6.0);
        }
    }

    // RK2 / trapezoidal rule
    void heuns_method(soft_model &body, double h, std::vector<vec3> &new_positions)
    {
        std::vector<soft_node> &nodes = body.nodes;
        int count = nodes.size();

        std::vector<vec3> start_pos(count), start_vel(count);
        for (int i = 0; i < count; i++)
        {
            start_pos[i] = nodes[i].pos;
            start_vel[i] = nodes[i].vel;
        }

        compute_soft_forces(body);
        std::vector<vec3> k0_vel(count), k0_acc(count);
        for (int i = 0; i < count; i++)
        {
            k0_vel[i] = nodes[i].pinned ? vec3(0) : nodes[i].vel;
            k0_acc[i] = nodes[i].pinned ? vec3(0) : nodes[i].force / nodes[i].mass;
        }

        for (int i = 0; i < count; i++) 
        {
            if (!nodes[i].pinned)
            {
                nodes[i].pos = start_pos[i] + k0_vel[i] * h;
                nodes[i].vel = start_vel[i] + k0_acc[i] * h;
            }
        }
        compute_soft_forces(body);

        new_positions.resize(count);
        for (int i = 0; i < count; i++)
        {
            if (nodes[i].pinned)
            {
                new_positions[i] = start_pos[i];
                nodes[i].vel = start_vel[i];
            }
            else
            {
                vec3 k1_vel = nodes[i].vel;
                vec3 k1_acc = nodes[i].force / nodes[i].mass;
                new_positions[i] = start_pos[i] + (k0_vel[i] + k1_vel) * (h / 2.0);
                nodes[i].vel = start_vel[i] + (k0_acc[i] + k1_acc) * (h / 2.0);
            }
            nodes[i].pos = start_pos[i]; // put the body back
        }
    }

    // leapfrog
    void velocity_verlet(soft_model &body, double h, std::vector<vec3> &new_positions)
    {
        std::vector<soft_node> &nodes = body.nodes;
        int count = nodes.size();

        compute_soft_forces(body);
        std::vector<vec3> start_pos(count), start_vel(count), start_acc(count);
        for (int i = 0; i < count; i++)
        {
            start_pos[i] = nodes[i].pos;
            start_vel[i] = nodes[i].vel;
            start_acc[i] = nodes[i].force / nodes[i].mass;
        }

        new_positions.resize(count);
        for (int i = 0; i < count; i++)
        {
            if (nodes[i].pinned)
            {
                new_positions[i] = start_pos[i];
            }
            else
            {
                new_positions[i] = start_pos[i] + start_vel[i] * h + start_acc[i] * (0.5 * h * h);
                nodes[i].pos = new_positions[i];
            }
        }

        compute_soft_forces(body);

        for (int i = 0; i < count; i++)
        {
            if (!nodes[i].pinned)
            {
                vec3 end_acc = nodes[i].force / nodes[i].mass;
                nodes[i].vel = start_vel[i] + (start_acc[i] + end_acc) * (0.5 * h);
            }
            nodes[i].pos = start_pos[i]; // put the body back
        }
    }

    void soft_node_collision(soft_model &body, soft_node &n, const vec3 &new_pos)
    {
        vec3 direction = new_pos - n.pos;
        double length = direction.length();
        if (length < 1e-12)
        {
            n.pos = new_pos;
            return;
        }
        ray r = ray(n.pos, direction / length);
        phy_hit_record rec;

        if (world_bvh->hit(r, interval(0.001, length + 0.01), rec))
        {
            vec3 normal = rec.normal;
            double v_dot_n = dot(n.vel, normal);
            if (v_dot_n < 0) // moving into the surface
            {
                vec3 v_normal = v_dot_n * normal;
                vec3 v_tangent = n.vel - v_normal;
                n.vel = v_tangent * (1.0 - body.friction) - body.restitution * v_normal;
            }
            n.pos = rec.p + normal * 0.01;
        }
        else
        {
            n.pos = new_pos;
        }
    }

    void edge_edge_collision(soft_model &a, soft_model &b)
    {
        // iterate a.springs directly instead of a.get_edges() for real node indices
        for (spring &s : a.springs)
        {
            if (s.type != spring_type::structural)
            {
                continue;
            }

            edge e;
            e.u = &a.nodes[s.a].pos;
            e.v = &a.nodes[s.b].pos;

            phy_hit_record rec;
            if (b.edge_hit(e, rec))
            {
                resolve_edge_contact(a, s.a, s.b, b, rec);
            }
        }
    }

    void resolve_edge_contact(soft_model &a, int node_u, int node_v, soft_model &b, phy_hit_record &rec)
    {
        // blending
        soft_node &a0 = a.nodes[node_u];
        soft_node &a1 = a.nodes[node_v];
        double weight_a1 = rec.bay_coord.x();
        double weight_a0 = 1.0 - weight_a1;

        auto hit_tri = std::static_pointer_cast<phy_soft_tri>(rec.hit_object);
        int side = (int)rec.bay_coord.z();
        soft_node &b0 = b.nodes[hit_tri->index(side)];
        soft_node &b1 = b.nodes[hit_tri->index((side + 1) % 3)];
        double weight_b1 = rec.bay_coord.y();
        double weight_b0 = 1.0 - weight_b1;

        double inv_mass_a = 0.0;
        if (!a0.pinned)
        {
            inv_mass_a += weight_a0 * weight_a0 / a0.mass;
        }
        if (!a1.pinned)
        {
            inv_mass_a += weight_a1 * weight_a1 / a1.mass;
        }

        double inv_mass_b = 0.0;
        if (!b0.pinned)
        {
            inv_mass_b += weight_b0 * weight_b0 / b0.mass;
        }
        if (!b1.pinned)
        {
            inv_mass_b += weight_b1 * weight_b1 / b1.mass;
        }

        double inv_mass_sum = inv_mass_a + inv_mass_b;
        if (inv_mass_sum <= 0.0)
        {
            return; // both sides pinned
        }

        vec3 normal = rec.normal; // points from b toward a

        double target_gap = 0.01;
        if (rec.t < target_gap)
        {
            double push = (target_gap - rec.t) / inv_mass_sum;
            if (!a0.pinned)
            {
                a0.pos += normal * (push * weight_a0 / a0.mass);
            }
            if (!a1.pinned)
            {
                a1.pos += normal * (push * weight_a1 / a1.mass);
            }
            if (!b0.pinned)
            {
                b0.pos += -normal * (push * weight_b0 / b0.mass);
            }
            if (!b1.pinned)
            {
                b1.pos += -normal * (push * weight_b1 / b1.mass);
            }
        }

        vec3 contact_vel_a = weight_a0 * a0.vel + weight_a1 * a1.vel;
        vec3 contact_vel_b = weight_b0 * b0.vel + weight_b1 * b1.vel;

        vec3 rel_vel = contact_vel_a - contact_vel_b;
        double v_normal = dot(rel_vel, normal);
        if (v_normal >= 0.0) // already separating
        {
            return;
        }

        double restitution = std::min(a.restitution, b.restitution);
        double friction = std::max(a.friction, b.friction);
        vec3 v_tangent = rel_vel - v_normal * normal;
        vec3 impulse = (-(1.0 + restitution) * v_normal * normal - friction * v_tangent) / inv_mass_sum;

        if (!a0.pinned)
        {
            a0.vel += impulse * (weight_a0 / a0.mass);
        }
        if (!a1.pinned)
        {
            a1.vel += impulse * (weight_a1 / a1.mass);
        }
        if (!b0.pinned)
        {
            b0.vel += -impulse * (weight_b0 / b0.mass);
        }
        if (!b1.pinned)
        {
            b1.vel += -impulse * (weight_b1 / b1.mass);
        }
    }

    void print_soft_energy(const soft_model &body)
    {
        double kinetic = 0.0;
        double potential = 0.0;
        double max_stretch = 0.0;
        for (auto &n : body.nodes)
        {
            kinetic += 0.5 * n.mass * n.vel.length_squared();
        }
        for (auto &s : body.springs)
        {
            potential += spring_energy(s, body.nodes);
            double ratio = std::fabs((body.nodes[s.a].pos - body.nodes[s.b].pos).length() - s.rest_length) / s.rest_length;
            max_stretch = std::max(max_stretch, ratio);
        }
        printf("soft %d step %d: kinetic %f spring %f total %f max stretch %.1f%%\n",
               body.id, cur_step, kinetic, potential, kinetic + potential, max_stretch * 100.0);
    }

    void sphere_to_sphere_collision(phy_sphere &cur_sphere, phy_sphere &test_sphere, double t_hit, double dt)
    {
        double dt_frac = t_hit * dt / MS_PER_SEC; // scaled time-into-frame

        vec3 p1_at_hit = cur_sphere.current_state.position + dt_frac * cur_sphere.current_state.velocity;
        vec3 p2_at_hit = test_sphere.current_state.position + dt_frac * test_sphere.current_state.velocity;

        vec3 collision_normal = unit_vector(p2_at_hit - p1_at_hit); // points from cur_sphere toward test_sphere
        vec3 contact_point = p1_at_hit + collision_normal * cur_sphere.get_radius();

        // "1" = cur_sphere, "2" = test_sphere
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

        // not moving relative to each other
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

    void normal_particle_behavior(particle_emitter &cur_emitter, double dt)
    {
        for (int j = 0; j < cur_emitter.particles.size(); j++)
        {
            particle &cur_particle = cur_emitter.particles[j];
            if (cur_particle.alive) // skip dead particle
            {
                vec3 new_acceleration = get_net_force(cur_particle.position, cur_particle.velocity, PARTICLE_MASS) / PARTICLE_MASS;
                vec3 new_velocity = cur_particle.velocity + new_acceleration * (dt / MS_PER_SEC);
                vec3 new_position = cur_particle.position + new_velocity * (dt / MS_PER_SEC);

                vec3 direction = new_position - cur_particle.position;
                double length = direction.length() + 0.01; // need to add extra to prevent points from going thru walls
                ray r = ray(cur_particle.position, unit_vector(direction));
                double closest_so_far = length;
                phy_hit_record rec;

                if (world_bvh->hit(r, interval(0.001, closest_so_far), rec))
                {
                    // resolve collision
                    double restitution = rec.hit_object->restitution; // needed?
                    vec3 n = rec.normal;
                    double v_dot_n = dot(new_velocity, n);

                    // reflect
                    vec3 reflected_velocity = new_velocity - n * ((1.0 + restitution) * v_dot_n);

                    cur_particle.position = rec.p + (n * 0.01); // maybe change
                    cur_particle.velocity = reflected_velocity;
                }
                else
                {
                    cur_particle.position = new_position;
                    cur_particle.velocity = new_velocity;
                }
            }
        }
    }

    void flock_particle_behavior(particle_emitter &cur_emitter, double dt)
    {
        particle_bvh neighbor_bvh;
        std::vector<int> neighbors; // maybe declare with max particle count idk
        std::vector<vec3> steering;

        neighbor_bvh.build(cur_emitter.particles);
        switch (cur_emitter.behavior)
        {
        case emitter_behavior::flocking:
            compute_flock_steering(cur_emitter.particles, neighbor_bvh, cur_emitter.flock, steering, world_bvh.get());
            break;
        case emitter_behavior::fluid:
            compute_sph_accel(cur_emitter.particles, neighbor_bvh, cur_emitter.flock, fluid_scratch, steering);
            break;
        case emitter_behavior::gross:
            compute_gross_steering(cur_emitter.particles, neighbor_bvh, cur_emitter.flock, steering, world_bvh.get());
            break;
        default:
            break; // ehh
        }
        // compute_flock_steering(cur_emitter.particles, neighbor_bvh, cur_emitter.flock, steering, world_bvh.get());
        for (size_t i = 0; i < cur_emitter.particles.size(); i++)
        {
            particle &p = cur_emitter.particles[i];
            if (!p.alive)
            {
                continue;
            }
            p.velocity = limit_length(p.velocity + steering[i] * (dt / MS_PER_SEC), cur_emitter.flock.max_speed);
        }

        normal_particle_behavior(cur_emitter, dt); // now just do normal behavior I guess
    }

private:
    sph_scratch fluid_scratch;
};

#endif