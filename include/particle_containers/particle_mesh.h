#ifndef PARTICLE_MESH_CONTAINER_H
#define PARTICLE_MESH_CONTAINER_H

#include "../hittable.h"
#include "../physics/objects/particle.h"

#define DEBUG true // this is to debug bvh

class particle_mesh_container : public hittable
{
public:
    std::vector<particle> *particles;
    hittable_list local_world;

    particle_mesh_container(std::vector<particle> &particles,
                            const std::string &file_name,
                            shared_ptr<material> override_mat,
                            double scale = 1.0)
        : particles(&particles), scale(scale), override_mat(override_mat)
    {
        mesh_container = make_shared<mesh>(file_name, override_mat, local_world);

        if (local_world.objects.empty())
            throw std::runtime_error("particle_mesh_container: no triangles loaded from " + file_name);

        create_local_box();
        create_bvh();
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        bool hit_anything = false;
        double closest_so_far = ray_t.max;
        const double inv_s = 1.0 / scale;

        for (const particle &p : *particles)
        {
            if (!p.alive)
            {
                continue;
            }
            
            mesh_container->face_direction(p.velocity); //YUCK!!!


            // world -> local, direction is scaled but NOT normalized
            ray local_r((r.origin() - p.position) * inv_s,
                        r.direction() * inv_s);

            hit_record temp_rec;
            if (DEBUG) //this needs to be on to allow for rotation NEED TO FIX
            {
                for (int i = 0; i < local_world.objects.size(); i++) // bvh is messing it up, this fixes it.
                {
                    if (local_world.objects[i]->hit(local_r, interval(ray_t.min, closest_so_far), temp_rec)) // this was mesh_bvh instead of local_world.objects[i]
                    {
                        // local -> world
                        temp_rec.t = temp_rec.t;       // unchanged by construction
                        temp_rec.p = r.at(temp_rec.t); // recompute in world space
                        if (override_mat){
                            temp_rec.mat = override_mat;
                        }

                        hit_anything = true;
                        closest_so_far = temp_rec.t;
                        rec = temp_rec;
                    }
                }
            }
            else
            {
                if (mesh_bvh->hit(local_r, interval(ray_t.min, closest_so_far), temp_rec))
                {
                    // local -> world
                    temp_rec.t = temp_rec.t;       // unchanged by construction
                    temp_rec.p = r.at(temp_rec.t); // recompute in world space
                    if (override_mat) {
                        temp_rec.mat = override_mat;
                    }

                    hit_anything = true;
                    closest_so_far = temp_rec.t;
                    rec = temp_rec;
                }
            }
        }
        return hit_anything;
    }

    void create_bvh()
    {
        mesh_bvh = make_shared<bvh_node>(local_world.objects, 0, local_world.objects.size());
    }

    aabb bounding_box() const override
    {
        aabb box;
        bool first = true;
        for (const particle &p : *particles)
        {
            if (!p.alive)
            {
                continue;
            }
            vec3 lo(local_box.x.min, local_box.y.min, local_box.z.min);
            vec3 hi(local_box.x.max, local_box.y.max, local_box.z.max);
            aabb b(p.position + lo * scale, p.position + hi * scale);

            box = first ? b : surrounding_box(box, b);
            first = false;
        }
        return box;
    }

    void position(vec3 &pos) override {}

    void create_local_box()
    {
        aabb box = local_world.objects[0]->bounding_box();

        for (size_t i = 1; i < local_world.objects.size(); i++) // can be int too
        {
            box = surrounding_box(
                box,
                local_world.objects[i]->bounding_box());
        }
        local_box = box;
    }

private:
    shared_ptr<mesh> mesh_container;
    shared_ptr<bvh_node> mesh_bvh;
    double scale;
    shared_ptr<material> override_mat;
    aabb local_box;
};

#endif