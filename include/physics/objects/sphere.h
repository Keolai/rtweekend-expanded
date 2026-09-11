#ifndef PHYSICS_phy_sphere_H
#define PHYSICS_phy_sphere_H

#include "../hittable.h"

class phy_sphere : public phy_hittable
{
public:
    phy_sphere(const vec3 &center, double radius)
        : center(center), radius(std::fmax(0, radius))
    {
        current_state.position = center;
        next_state.position = center;
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) const override
    {
        vec3 next_center = next_state.position;
        vec3 oc = next_center - r.origin();
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
        vec3 outward_normal = (rec.p - next_center) / radius;
        rec.set_normal(r, outward_normal);
        vec3 p = unit_vector(rec.p - next_center);

        double u = 0.5 + atan2(p.z(), p.x()) / (2.0 * pi);
        double v = 0.5 - asin(p.y()) / pi;
        rec.bay_coord = vec3(u, v, 0.);

        vec3 T(-p.z(), 0, p.x());

        T = unit_vector(T);

        if (T.length_squared() < 1e-12)
        {
            // north/south pole
            T = vec3(1, 0, 0);
        }
        // printf("discriminant: %f\n", discriminant);
        // printf("rec.t: %f\n", rec.t);

        return true;
    }

    phy_aabb bounding_box() const override
    {
        vec3 rvec(radius, radius, radius);

        return phy_aabb(
            center - rvec,
            center + rvec);
    }

    void position(vec3 &newPos)
    {
        center = newPos;
    }

    void update_state() override
    {
        copy(next_state, current_state); // copy the next predicted state to the new state;
    }

    double hit_adjuster() const override
    {
        return radius;
    }

    bool is_inside(const ray &r) const override
    {
        double length_from_center = (r.origin() - next_state.position).length_squared();
        return (length_from_center) < (radius * radius);
    }

    vec3 closest_point_on_surface(const vec3 &pos) const override
    {
        vec3 dir = unit_vector(next_state.position - pos); //next_state.position is the predicted center next step incase it is moving
        vec3 point_on_surface = (next_state.position) + (dir * radius);
        return point_on_surface;
    }

private:
    point3 center;
    double radius;
};

#endif