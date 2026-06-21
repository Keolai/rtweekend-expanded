#ifndef HITTABLE_H
#define HITTABLE_H

class material;

#include "aabb.h"
#include "texture.h"

class hit_record
{
public:
  point3 p;
  vec3 normal;
  vec3 geometry_normal;

  vec3 tangent;
  vec3 bitangent;

  shared_ptr<material> mat;
  vec3 texture_sample_point;
  double t;
  bool front_face;

  void set_face_normal(const ray &r, const vec3 &outward_normal)
  {
    // Sets the hit record normal vector.
    // NOTE: the parameter `outward_normal` is assumed to have unit length.

    front_face = dot(r.direction(), outward_normal) < 0;
    normal = front_face ? outward_normal : -outward_normal;
  }

  void set_geometry_normal(const ray &r, const vec3 &outward_normal)
  {
    // Sets the hit record normal vector.
    // NOTE: the parameter `outward_normal` is assumed to have unit length.

    front_face = dot(r.direction(), outward_normal) < 0;
    geometry_normal = front_face ? outward_normal : -outward_normal;
  }
};

class hittable
{
public:
  virtual ~hittable() = default;
  virtual aabb bounding_box() const = 0;

  virtual bool hit(const ray &r, interval ray_t, hit_record &rec) const = 0;

   void find_world_tangent(const ray &r, hit_record &rec, texture &tex) const
  {
    color texel = tex.get_color_at_coordinate(rec.texture_sample_point.x(), rec.texture_sample_point.y());
    vec3 tangent_normal(
        2.0 * texel.x() - 1.0,
        2.0 * texel.y() - 1.0,
        2.0 * texel.z() - 1.0);

    tangent_normal =
        unit_vector(tangent_normal);

    vec3 world_normal =
        unit_vector(
            tangent_normal.x() * rec.tangent +
            tangent_normal.y() * rec.bitangent +
            tangent_normal.z() * rec.normal);

    rec.set_face_normal(r, world_normal);
  }

};

#endif