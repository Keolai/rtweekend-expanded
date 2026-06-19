#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"

class sphere : public hittable
{
public:
  sphere(const point3 &center, double radius, shared_ptr<material> mat)
      : center(center), radius(std::fmax(0, radius)), mat(mat) {}

  bool hit(const ray &r, interval ray_t, hit_record &rec) const override
  {
    vec3 oc = center - r.origin();
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
    vec3 outward_normal = (rec.p - center) / radius;
    rec.set_face_normal(r, outward_normal);
    rec.set_geometry_normal(r, outward_normal);
    rec.mat = mat;
    vec3 p = unit_vector(rec.p - center);

    double u = 0.5 + atan2(p.z(), p.x()) / (2.0 * pi);
    double v = 0.5 - asin(p.y()) / pi;
    rec.texture_sample_point = vec3(u, v, 0.);

    vec3 T(-p.z(), 0, p.x());

    T = unit_vector(T);

    if (T.length_squared() < 1e-12)
    {
      // north/south pole
      T = vec3(1, 0, 0);
    }

    vec3 B = unit_vector(cross(outward_normal, T));
    rec.tangent = T;
    rec.bitangent = B;

    find_world_tangent(r, rec);

    return true;
  }

  aabb bounding_box() const override
  {
    vec3 rvec(radius, radius, radius);

    return aabb(
        center - rvec,
        center + rvec);
  }

private:
  point3 center;
  double radius;
  shared_ptr<material> mat;

  void find_world_tangent(const ray &r, hit_record &rec) const
  {
    color texel = mat->normal_texture.get_color_at_coordinate(rec.texture_sample_point.x(), rec.texture_sample_point.y());
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