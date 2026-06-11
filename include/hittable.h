#ifndef HITTABLE_H
#define HITTABLE_H

class material;

#include "aabb.h"

class hit_record {
  public:
    point3 p;
    vec3 normal;
    vec3 geometry_normal;
    shared_ptr<material> mat;
    vec3 texture_sample_point;
    double t;
    bool front_face;

    void set_face_normal(const ray& r, const vec3& outward_normal) {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.

        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }

    void set_geometry_normal(const ray& r, const vec3& outward_normal) {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.

        front_face = dot(r.direction(), outward_normal) < 0;
        geometry_normal = front_face ? outward_normal : -outward_normal;
    }
   
};

class hittable {
  public:
    virtual ~hittable() = default;
    virtual aabb bounding_box() const = 0;

    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
};

#endif