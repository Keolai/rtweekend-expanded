#ifndef PHYSICS_phy_tri_H
#define PHYSICS_phy_tri_H

#include "../hittable.h"
#include <array>

class phy_tri : public phy_hittable
{
public:
    int model_id = -1;

    phy_tri(const std::array<point3, 3> &vertices)
        : vertices(vertices)
    {
        copy_vertices(vertices, original_vertices);
    }

    phy_tri(point3 &v1, point3 &v2, point3 &v3)
    {
        vertices[0] = v1;
        vertices[1] = v2;
        vertices[2] = v3;
        original_vertices[0] = v1;
        original_vertices[1] = v2;
        original_vertices[2] = v3;
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) const override
    {
        auto epsilon = 1e-8;

        vec3 edge1 = vertices.at(1) - vertices.at(0);
        vec3 edge2 = vertices.at(2) - vertices.at(0);
        const vec3 normal = unit_vector(cross(edge1, edge2));

        vec3 ray_cross_e2 = cross(r.direction(), edge2);
        float det = dot(edge1, ray_cross_e2);

        if (std::fabs(det) < epsilon)
        { // phy_ray  is parallel
            return false;
        }

        float inv_det = 1.0 / det;
        vec3 s = r.origin() - vertices.at(0);
        float u = inv_det * dot(s, ray_cross_e2);
        if (u < 0.0 || u > 1.0)
        {
            return false; // phy_ray  passes outside of edge2 bounds
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
            rec.p = r.at(t); // phy_ray  at point
            rec.set_normal(r, normal);
            // std::cout << "phy_tri normal: " << rec.normal << '\n';
            rec.t = t; // point where it hit
            rec.bay_coord = bay_coordinate(u, v);
            return true;
        }
        else
        {
            return false;
        } // This means that there is a line intersection but not a phy_ray  intersection.
    }

    phy_aabb bounding_box() const override
    {
        double min_x = std::min({vertices[0].x(), vertices[1].x(), vertices[2].x()});
        double min_y = std::min({vertices[0].y(), vertices[1].y(), vertices[2].y()});
        double min_z = std::min({vertices[0].z(), vertices[1].z(), vertices[2].z()});

        double max_x = std::max({vertices[0].x(), vertices[1].x(), vertices[2].x()});
        double max_y = std::max({vertices[0].y(), vertices[1].y(), vertices[2].y()});
        double max_z = std::max({vertices[0].z(), vertices[1].z(), vertices[2].z()});

        point3 min_point(min_x, min_y, min_z);
        point3 max_point(max_x, max_y, max_z);

        return phy_aabb(min_point, max_point);
    }

    vec3 bay_coordinate(double u, double v) const
    {
        double w = 1.0 - u - v;
        return w +
               u +
               v;
    }

    void copy_vertices(const std::array<point3, 3> &orig, std::array<point3, 3> &dest)
    {
        for (int i = 0; i < 3; i++)
        {
            copy(orig[i], dest[i]);
        }
    }

    void position(vec3 &pos)
    {
        for (int i = 0; i < 3; i++)
        {
            copy(original_vertices[i] + pos, vertices[i]);
        }
    }

    void update_state() override
    {
        copy(next_state,current_state); //copy the next predicted state to the new state;
    }

private:
    std::array<point3, 3> vertices;
    std::array<point3, 3> original_vertices; // original positions of vertices in model
};

#endif