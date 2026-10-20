#ifndef PARTICLE_CONTAINER_H
#define PARTICLE_CONTAINER_H

#include "particle_hittable.h"
#include "../sphere.h"

class particle_container : public particle_hittable
{
public:
    particle_container(std::vector<particle> &particles, shared_ptr<material> mat, double radius)
        : particle_hittable(particles), mat(mat), second_mat(mat), particle_radius(radius) {}
    particle_container(std::vector<particle> &particles, shared_ptr<material> mat, shared_ptr<material> second_mat, double radius)
        : particle_hittable(particles), mat(mat), second_mat(second_mat), particle_radius(radius) {}

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        bool hit_anything = false;
        interval search = ray_t;

        auto visit = [&](int index, const vec3 &position)
        {
            hit_record temp_rec;
            if (sphere_hit(r, position, particle_radius, search, temp_rec))
            {
                hit_anything = true;
                search.max = temp_rec.t;
                rec = temp_rec;
                if (snapshot[index].is_gross)
                {
                    rec.mat = second_mat;
                }
            }
        };
        bvh.query_ray(r, search, particle_radius, visit);
        return hit_anything;
    }

protected:
    double bound_radius() const override { return particle_radius; }

private:
    //std::vector<sphere> spheres;
    shared_ptr<material> mat;
     shared_ptr<material> second_mat;
    double particle_radius;

    bool sphere_hit(const ray &r, const vec3 &position, double radius, interval ray_t, hit_record &rec) const
    {
        vec3 oc = position - r.origin();
        auto a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius * radius;

        auto discriminant = h * h - a * c;
        if (discriminant < 0)
            return false;

        auto sqrtd = std::sqrt(discriminant);

        // Find the nearest root that lies in the acceptable range.
        auto root = (h - sqrtd) / a;
        if (!ray_t.surrounds(root))
        {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        vec3 outward_normal = (rec.p - position) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.set_geometry_normal(r, outward_normal);
        rec.mat = mat;
        vec3 p = unit_vector(rec.p - position);

        double u = 0.5 + atan2(p.z(), p.x()) / (2.0 * pi);
        double v = 0.5 - asin(p.y()) / pi;
        rec.texture_sample_point = vec3(u, v, 0.);

        vec3 T(-p.z(), 0, p.x());
        vec3 T_raw(-p.z(), 0, p.x());
        if (T_raw.length_squared() < 1e-12)
        {
            T = vec3(1, 0, 0);
        }
        else
        {
            T = unit_vector(T_raw);
        }

        vec3 B = unit_vector(cross(outward_normal, T));
        rec.tangent = T;
        rec.bitangent = B;

        return true;
    }
};

#endif