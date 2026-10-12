#ifndef tri_H
#define tri_H

#include "hittable.h"
#include "quaternion.h"
#include <array>

#define tolerance 1e-4

class tri : public hittable
{
public:
    tri(const std::array<point3, 3> &vertices, shared_ptr<material> mat)
        : vertices(vertices), mat(mat)
    {
        copy_vertices(vertices, original_vertices);
        normals_stored = false;
        textcoords_stored = false;
        pivot = original_centroid();
    }

    tri(const std::array<point3, 3> &vertices, const std::array<point3, 3> &normals, shared_ptr<material> mat)
        : vertices(vertices), mat(mat), normals(normals)
    {
        copy_vertices(vertices, original_vertices);
        normals_stored = true;
        textcoords_stored = false;
        tangent_space();
        original_normals = normals;
        original_tangent = const_tangent;
        original_bitangent = const_bitangent;
        pivot = original_centroid();
    }

    tri(const std::array<point3, 3> &vertices, const std::array<point3, 3> &normals,
        const std::array<point3, 3> &UV, shared_ptr<material> mat)
        : vertices(vertices), mat(mat), normals(normals), UV(UV)
    {
        copy_vertices(vertices, original_vertices);
        normals_stored = true;
        textcoords_stored = true;
        tangent_space();
        original_normals = normals;
        original_tangent = const_tangent;
        original_bitangent = const_bitangent;
        pivot = original_centroid();
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        auto epsilon = 1e-8;
        vec3 edge1 = vertices.at(1) - vertices.at(0);
        vec3 edge2 = vertices.at(2) - vertices.at(0);
        vec3 cross_e1e2 = cross(edge1, edge2);

        if (cross_e1e2.length_squared() < 1e-16) // degenerate/zero-area triangle
            return false;

        const vec3 normal = cross_e1e2 / sqrt(cross_e1e2.length_squared());

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
            // std::cout << "tri normal: " << rec.normal << '\n';
            rec.t = t; // point where it hit
            rec.mat = mat;
            rec.texture_sample_point = interpolated_texture(u, v, UV);
            if (std::isnan(rec.normal.x()))
            {
                printf("NaN normal! t=%f p=(%f,%f,%f)\n", rec.t, rec.p.x(), rec.p.y(), rec.p.z());
            }

            find_world_tangent(r, rec, mat->normal_texture);
            if (std::isnan(rec.normal.x()))
            {
                printf("NaN normal after world_tangent!! t=%f p=(%f,%f,%f)\n", rec.t, rec.p.x(), rec.p.y(), rec.p.z());
            }
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
        if (max_x - min_x < tolerance)
        {
            min_x -= tolerance * 0.5;
            max_x += tolerance * 0.5;
        }
        if (max_y - min_y < tolerance)
        {
            min_y -= tolerance * 0.5;
            max_y += tolerance * 0.5;
        }
        if (max_z - min_z < tolerance)
        {
            min_z -= tolerance * 0.5;
            max_z += tolerance * 0.5;
        }

        return aabb(min_point, max_point);
    }

    vec3 interpolated_texture(double u, double v, const std::array<point3, 3> &arr) const
    {
        double w = 1.0 - u - v;
        return w * arr[0] +
               u * arr[1] +
               v * arr[2];
    }

    void copy_vertices(const std::array<point3, 3> &orig, std::array<point3, 3> &dest)
    {
        for (int i = 0; i < 3; i++)
        {
            copy(orig[i], dest[i]);
        }
    }

    void position(vec3 &pos) override
    {
        current_position = pos;
        update();
    }

    void rotate(const vec3 &degrees)
    {
        orientation = (quat::from_euler(degrees) * orientation).normalized();
        update();
    }

    void rotate_axis(const vec3 &axis, double degrees)
    {
        orientation = (quat::from_axis_angle(axis, degrees) * orientation).normalized();
        update();
    }

    void set_pivot(const point3 &p)
    {
        pivot = p;
        update();
    }

    void set_rotation(const vec3 &degrees)
    {
        orientation = quat::from_euler(degrees).normalized();
        update();
    }

    void reset_rotation()
    {
        // identity
        orientation = quat();
        update();
    }

    point3 original_centroid() const
    {
        return (original_vertices[0] + original_vertices[1] + original_vertices[2]) / 3.0;
    }

private:
    bool normals_stored;
    bool textcoords_stored;
    vec3 current_position = vec3(0);
    quat orientation;
    std::array<point3, 3> vertices;
    std::array<point3, 3> original_vertices; // original positions of vertices in model
    std::array<point3, 3> normals;
    std::array<point3, 3> UV;
    vec3 const_tangent;
    vec3 const_bitangent;
    shared_ptr<material> mat;

    point3 pivot;

    std::array<point3, 3> original_normals;
    vec3 original_tangent, original_bitangent;

    static vec3 rotate_point(vec3 p, double rx, double ry, double rz)
    {
        double c;
        double s;
        // X axis
        c = std::cos(rx);
        s = std::sin(rx);
        p = vec3(p.x(),
                 c * p.y() - s * p.z(),
                 s * p.y() + c * p.z());

        // Y axis
        c = std::cos(ry);
        s = std::sin(ry);
        p = vec3(c * p.x() + s * p.z(),
                 p.y(),
                 -s * p.x() + c * p.z());

        // Z axis
        c = std::cos(rz);
        s = std::sin(rz);
        p = vec3(c * p.x() - s * p.y(),
                 s * p.x() + c * p.y(),
                 p.z());

        return p;
    }

    void tangent_space()
    {
        vec3 edge1 = vertices.at(1) - vertices.at(0);
        vec3 edge2 = vertices.at(2) - vertices.at(0);

        vec3 deltaUV1 = UV.at(1) - UV.at(0);
        vec3 deltaUV2 = UV.at(2) - UV.at(0);

        double denom = deltaUV1.x() * deltaUV2.y() - deltaUV2.x() * deltaUV1.y();

        if (std::fabs(denom) < 1e-8)
        {
            // Degenerate/duplicate UVs: fall back to an arbitrary tangent
            // perpendicular to the face normal.
            printf("degenerate UV determinant! denom=%e, UVs: (%f,%f) (%f,%f) (%f,%f)\n",
                   denom, UV[0].x(), UV[0].y(), UV[1].x(), UV[1].y(), UV[2].x(), UV[2].y());
            vec3 n = unit_vector(cross(edge1, edge2));
            vec3 ref = (std::fabs(n.x()) < 0.999) ? vec3(1, 0, 0) : vec3(0, 1, 0);
            const_tangent = unit_vector(ref - dot(ref, n) * n);
            const_bitangent = cross(n, const_tangent);
            return;
        }

        double f = 1.0 / denom;

        const_tangent = unit_vector(f * (deltaUV2.y() * edge1 - deltaUV1.y() * edge2));
        const_bitangent = unit_vector(f * (-deltaUV2.x() * edge1 + deltaUV1.x() * edge2));
    }

    void tangent_on_hit(vec3 N, hit_record &rec) const
    {
        vec3 ref = const_tangent;
        if (std::fabs(dot(ref, N)) > 0.999)
        {
            ref = (std::fabs(N.x()) < 0.999) ? vec3(1, 0, 0) : vec3(0, 1, 0);
        }

        vec3 tangent = unit_vector(ref - dot(ref, N) * N);
        vec3 bitangent = cross(N, tangent);

        rec.tangent = tangent;
        rec.bitangent = bitangent;
    }

    void update()
    {

        for (int i = 0; i < 3; i++)
        {
            vertices[i] = pivot + orientation.rotate(original_vertices[i] - pivot) + current_position;
            if (normals_stored)
                normals[i] = orientation.rotate(original_normals[i]);
        }
        if (normals_stored)
        {
            const_tangent = orientation.rotate(original_tangent);
            const_bitangent = orientation.rotate(original_bitangent);
        }
    }
};

#endif