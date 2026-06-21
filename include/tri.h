#ifndef tri_H
#define tri_H

#include "hittable.h"

class tri : public hittable
{
public:
    tri(const std::array<point3, 3> &vertices, shared_ptr<material> mat)
        : vertices(vertices), mat(mat)
    {
        normals_stored = false;
        textcoords_stored = false;
    }

    tri(const std::array<point3, 3> &vertices, const std::array<point3, 3> &normals, shared_ptr<material> mat)
        : vertices(vertices), mat(mat), normals(normals)
    {
        normals_stored = true;
        textcoords_stored = false;
        tangent_space();
    }

    tri(const std::array<point3, 3> &vertices, const std::array<point3, 3> &normals,
        const std::array<point3, 3> &UV, shared_ptr<material> mat)
        : vertices(vertices), mat(mat), normals(normals), UV(UV)
    {
        normals_stored = true;
        textcoords_stored = true;
        tangent_space();
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        auto epsilon = 1e-8;

        vec3 edge1 = vertices.at(1) - vertices.at(0);
        vec3 edge2 = vertices.at(2) - vertices.at(0);
        const vec3 normal = unit_vector(cross(edge1, edge2));

        vec3 ray_cross_e2 = cross(r.direction(), edge2);
        float det = dot(edge1, ray_cross_e2);

        if (std::fabs(det) < epsilon)
        { // ray is parallel
            return false;
        }

        float inv_det = 1.0 / det;
        vec3 s = r.origin() - vertices.at(0);
        float u = inv_det * dot(s, ray_cross_e2);
        if (u < 0.0 || u > 1.0)
        {
            return false; // ray passes outside of edge2 bounds
        }

        vec3 s_cross_e1 = cross(s, edge1);
        float v = inv_det * dot(r.direction(), s_cross_e1);

        if (v < 0.0 || u + v > 1.0)
        {
            return false;
        } // Ray passes outside edge1's bounds

        float t = inv_det * dot(edge2, s_cross_e1);

        if (ray_t.contains(t)) // Ray intersection !!
        {
            rec.p = r.at(t); // ray at point
            if (!normals_stored)
            { // just do flat shading
                rec.set_face_normal(r, normal);
                rec.set_geometry_normal(r, normal);
            }
            else
            {
                rec.set_face_normal(r, unit_vector(interpolated_texture(u, v, normals)));
                rec.set_geometry_normal(r, normal);
            }
            tangent_on_hit(rec.normal, rec);
            rec.t = t; // point where it hit
            rec.mat = mat;
            rec.texture_sample_point = interpolated_texture(u, v, UV);

            find_world_tangent(r, rec, mat->normal_texture);

            return true;
        }
        else
        {
            return false;
        } // This means that there is a line intersection but not a ray intersection.
    }

    aabb bounding_box() const override
    {
        double min_x = std::min({vertices[0].x(), vertices[1].x(), vertices[2].x()});
        double min_y = std::min({vertices[0].y(), vertices[1].y(), vertices[2].y()});
        double min_z = std::min({vertices[0].z(), vertices[1].z(), vertices[2].z()});

        double max_x = std::max({vertices[0].x(), vertices[1].x(), vertices[2].x()});
        double max_y = std::max({vertices[0].y(), vertices[1].y(), vertices[2].y()});
        double max_z = std::max({vertices[0].z(), vertices[1].z(), vertices[2].z()});

        point3 min_point(min_x, min_y, min_z);
        point3 max_point(max_x, max_y, max_z);

        return aabb(min_point, max_point);
    }

    vec3 interpolated_texture(double u, double v, const std::array<point3, 3> &arr) const
    {
        double w = 1.0 - u - v;
        return w * arr[0] +
               u * arr[1] +
               v * arr[2];
    }

private:
    bool normals_stored;
    bool textcoords_stored;
    std::array<point3, 3> vertices;
    std::array<point3, 3> normals;
    std::array<point3, 3> UV;
    vec3 const_tangent;
    vec3 const_bitangent;
    shared_ptr<material> mat;

    void tangent_space()
    {
        vec3 edge1 = vertices.at(1) - vertices.at(0);
        vec3 edge2 = vertices.at(2) - vertices.at(0);

        vec3 deltaUV1 = UV.at(1) - UV.at(0);
        vec3 deltaUV2 = UV.at(2) - UV.at(0);

        double f =
            1.0 /
            (deltaUV1.x() * deltaUV2.y() - deltaUV2.x() * deltaUV1.y());

        const_tangent =
            unit_vector(f *
                        (deltaUV2.y() * edge1 -
                         deltaUV1.y() * edge2));

        const_bitangent =
            unit_vector(f *
                        (-deltaUV2.x() * edge1 +
                         deltaUV1.x() * edge2));
    }

    void tangent_on_hit(vec3 N, hit_record &rec) const
    {
        vec3 tangent =
            unit_vector(
                const_tangent -
                dot(const_tangent, N) * N);

        vec3 bitangent =
            cross(N, tangent);
        // Gram-Schmidt

        rec.tangent = tangent;
        rec.bitangent = bitangent;
    }
};

#endif