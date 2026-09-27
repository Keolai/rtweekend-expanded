#ifndef PARTICLE_CONTAINER_H
#define PARTICLE_CONTAINER_H

#include "hittable.h"
#include "sphere.h"
#include "physics/objects/particle.h"

class particle_container : public hittable
{
public:
    std::vector<particle> *particles;
    particle_container(std::vector<particle> &particles, shared_ptr<material> mat, double radius)
        : particles(&particles), mat(mat), particle_radius(radius) {}

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        bool hit_anything = false;
        double closest_so_far = ray_t.max;

        for (int i = 0; i < particles->size(); i++)
        {
            const particle &p = (*particles)[i];
            if (!p.alive)
                continue;

            // inline sphere-ray intersection using p.position, radius, current closest_so_far
            hit_record temp_rec;
            if (sphere_hit(r, p.position, particle_radius, interval(ray_t.min, closest_so_far), temp_rec))
            {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_anything;
    }
    aabb bounding_box() const override
    {
        aabb box; // default-constructed, should represent an empty/invalid box per your aabb impl

        bool first = true;

        for (int i = 0; i < particles->size(); i++)
        {
            const particle &p = (*particles)[i];
            if (!p.alive)
                continue;

            vec3 rvec(particle_radius, particle_radius, particle_radius);
            aabb particle_box(p.position - rvec, p.position + rvec);

            if (first)
            {
                box = particle_box;
                first = false;
            }
            else
            {
                box = surrounding_box(box, particle_box); // assuming your aabb has a "surrounding box" ctor
            }
        }

        return box;
    }

    void position(vec3 &pos) override
    {
        // do nothing
    }

private:
    //std::vector<sphere> spheres;
    shared_ptr<material> mat;
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