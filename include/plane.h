#ifndef PLANE_H
#define PLANE_H

#include "hittable.h"
#include "tri.h"

class plane : public hittable
{
public:
    plane(const point3 &center, double length, double width, shared_ptr<material> mat)
        : center(center), length(length), width(width), mat(mat) { create_plane(); }
    plane(const point3 &center, double length, shared_ptr<material> mat)
        : center(center), length(length), width(length), mat(mat) { create_plane(); }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        hit_record temp_rec;
        bool hit_anything = false;
        double closest_so_far = ray_t.max;
        // printf("testing plane\n");

        for (const auto &triangle : triangles)
        {
            if (triangle->hit(
                    r,
                    interval(ray_t.min, closest_so_far),
                    temp_rec))
            {
                hit_anything = true;
                // std::cout << "tri normal: " << temp_rec.normal << '\n';
                closest_so_far = temp_rec.t;
                rec = temp_rec;
                // rec.mat = mat;
            }
        }

        return hit_anything;
    }

    aabb bounding_box() const override
    {
        constexpr double epsilon = 0.0001;

        point3 min_point(
            center.x() - length / 2,
            center.y() - epsilon,
            center.z() - width / 2);

        point3 max_point(
            center.x() + length / 2,
            center.y() + epsilon,
            center.z() + width / 2);

        return aabb(min_point, max_point);
    }

private:
    point3 center;
    double length;
    double width;
    std::array<std::shared_ptr<tri>, 2> triangles;
    shared_ptr<material> mat;

    void create_plane()
    {
        point3 p0 = center + vec3(-length / 2, 0, -width / 2);
        point3 p1 = center + vec3(length / 2, 0, -width / 2);
        point3 p2 = center + vec3(length / 2, 0, width / 2);
        point3 p3 = center + vec3(-length / 2, 0, width / 2);
        vec3 normals = vec3(0,1,0);

        triangles[0] = std::make_shared<tri>(
            std::array<point3, 3>{p0, p3, p2}, std::array<point3,3>{normals,normals,normals},
            std::array<vec3, 3>{
                vec3(0, 0,0),
                vec3(0, 1,0),
                vec3(1, 1,0)},
            mat);

        triangles[1] = std::make_shared<tri>(
            std::array<point3, 3>{p0, p2, p1}, std::array<point3,3>{normals,normals,normals},
            std::array<vec3, 3>{
                vec3(0, 0,0),
                vec3(1, 1,0),
                vec3(1, 0,0)},
            mat);
    }
};

#endif