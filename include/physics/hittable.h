#ifndef PHY_HITTABLE_H
#define PHY_HITTABLE_H

#include "utilities/aabb.h"
#include "utilities/state.h"

class phy_hit_record
{
public:
    point3 p;
    vec3 normal;

    vec3 bay_coord;

    double t;
    bool front_face;

    void set_normal(const ray &r, const vec3 &outward_normal)
    {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.

        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class phy_hittable
{
public:
    virtual ~phy_hittable() = default;
    virtual phy_aabb bounding_box() const = 0; // need?

    virtual bool hit(const ray &r, interval ray_t, phy_hit_record &rec) const = 0;

    state current_state = state();
    state next_state = state();

    double mass = 0;

    virtual void update_state() = 0;
    bool rigid = true; // dont move!
};

#endif