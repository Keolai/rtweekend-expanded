#ifndef PARTICLE_MESH_CONTAINER_H
#define PARTICLE_MESH_CONTAINER_H

#include "particle_hittable.h"

#define DEBUG true // this is to debug bvh

class particle_mesh_container : public particle_hittable
{
public:
    hittable_list local_world;

    particle_mesh_container(std::vector<particle> &particles,
                            const std::string &file_name,
                            shared_ptr<material> override_mat,
                            double scale = 1.0)
        : particle_hittable(particles), scale(scale), override_mat(override_mat)
    {
        mesh_container = make_shared<mesh>(file_name, override_mat, local_world);

        if (local_world.objects.empty())
            throw std::runtime_error("particle_mesh_container: no triangles loaded from " + file_name);

        compute_local_radius();
        create_bvh();
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        bool hit_anything = false;
        interval search = ray_t;
        const double inv_s = 1.0 / scale;

        auto visit = [&](int index, const vec3 &position)
        {
            mesh_container->face_direction(snapshot[index].velocity); //YUCK!!!

            // world -> local, direction is scaled but NOT normalized
            ray local_r((r.origin() - position) * inv_s,
                        r.direction() * inv_s);

            hit_record temp_rec;
            bool local_hit = false;
            if (DEBUG) //this needs to be on to allow for rotation NEED TO FIX
            {
                for (int i = 0; i < local_world.objects.size(); i++) // bvh is messing it up, this fixes it.
                {
                    hit_record tri_rec;
                    if (local_world.objects[i]->hit(local_r, search, tri_rec))
                    {
                        local_hit = true;
                        search.max = tri_rec.t;
                        temp_rec = tri_rec;
                    }
                }
            }
            else
            {
                local_hit = mesh_bvh->hit(local_r, search, temp_rec);
            }

            if (local_hit)
            {
                // local -> world, t is unchanged by construction
                temp_rec.p = r.at(temp_rec.t);
                if (override_mat)
                {
                    temp_rec.mat = override_mat;
                }

                hit_anything = true;
                search.max = temp_rec.t;
                rec = temp_rec;
            }
        };
        bvh.query_ray(r, search, bound_radius(), visit);
        return hit_anything;
    }

    void create_bvh()
    {
        mesh_bvh = make_shared<bvh_node>(local_world.objects, 0, local_world.objects.size());
    }

protected:
    double bound_radius() const override { return local_radius * scale; }

private:
    shared_ptr<mesh> mesh_container;
    shared_ptr<bvh_node> mesh_bvh;
    double scale;
    shared_ptr<material> override_mat;
    double local_radius = 0.0;

    // face_direction rotates the mesh about its pivot, so bound it with a sphere around the
    // local origin that contains every rotation: |pivot| + farthest point from the pivot
    void compute_local_radius()
    {
        point3 pivot = mesh_container->get_pivot();
        double max_dist = 0.0;
        for (size_t i = 0; i < local_world.objects.size(); i++)
        {
            aabb box = local_world.objects[i]->bounding_box();
            for (int c = 0; c < 8; c++)
            {
                vec3 corner((c & 1) ? box.x.max : box.x.min,
                            (c & 2) ? box.y.max : box.y.min,
                            (c & 4) ? box.z.max : box.z.min);
                max_dist = std::max(max_dist, (corner - pivot).length());
            }
        }
        local_radius = pivot.length() + max_dist;
    }
};

#endif