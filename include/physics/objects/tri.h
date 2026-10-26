#ifndef PHYSICS_phy_tri_H
#define PHYSICS_phy_tri_H

#include "../hittable.h"
#include <array>

#define tolerance 1e-4 // can change

// shared by phy_tri and phy_soft_tri, caller sets rec.hit_object
inline bool ray_tri_hit(const ray &r, interval ray_t, const point3 &v0, const point3 &v1, const point3 &v2, phy_hit_record &rec)
{
    auto epsilon = 1e-8;

    vec3 edge1 = v1 - v0;
    vec3 edge2 = v2 - v0;
    const vec3 normal = unit_vector(cross(edge1, edge2));

    vec3 ray_cross_e2 = cross(r.direction(), edge2);
    float det = dot(edge1, ray_cross_e2);

    if (std::fabs(det) < epsilon)
    { // phy_ray  is parallel
        return false;
    }

    float inv_det = 1.0 / det;
    vec3 s = r.origin() - v0;
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
        rec.t = t; // point where it hit
        rec.bay_coord = vec3(1.0 - u - v, u, v);
        return true;
    }
    else
    {
        return false;
    } // This means that there is a line intersection but not a phy_ray  intersection.
}

inline phy_aabb tri_bounding_box(const point3 &v0, const point3 &v1, const point3 &v2)
{
    double min_x = std::min({v0.x(), v1.x(), v2.x()});
    double min_y = std::min({v0.y(), v1.y(), v2.y()});
    double min_z = std::min({v0.z(), v1.z(), v2.z()});

    double max_x = std::max({v0.x(), v1.x(), v2.x()});
    double max_y = std::max({v0.y(), v1.y(), v2.y()});
    double max_z = std::max({v0.z(), v1.z(), v2.z()});

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

    return phy_aabb(point3(min_x, min_y, min_z), point3(max_x, max_y, max_z));
}

class phy_tri : public phy_hittable
{
public:
    phy_tri(const std::array<point3, 3> &vertices)
        : vertices(vertices)
    {
        copy_vertices(vertices, original_vertices);
        pivot = original_centroid();
    }

    phy_tri(point3 &v1, point3 &v2, point3 &v3)
    {
        vertices[0] = v1;
        vertices[1] = v2;
        vertices[2] = v3;
        original_vertices[0] = v1;
        original_vertices[1] = v2;
        original_vertices[2] = v3;
        pivot = original_centroid();
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) override
    {
        if (ray_tri_hit(r, ray_t, vertices[0], vertices[1], vertices[2], rec))
        {
            rec.hit_object = shared_from_this();
            return true;
        }
        return false;
    }

    phy_aabb bounding_box() const override
    {
        return tri_bounding_box(vertices[0], vertices[1], vertices[2]);
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

    point3 original_centroid() const
    {
        return (original_vertices[0] + original_vertices[1] + original_vertices[2]) / 3.0;
    }

    void update_state() override
    {
        copy(next_state, current_state); // copy the next predicted state to the new state;
    }

    bool is_inside(const ray &r) const override
    {
        return false;
    }

    double hit_adjuster() const override
    {
        return 0.001;
    }

    vec3 closest_point_on_surface(const vec3 &pos) const override
    {

        return vec3(0);
    }

    std::vector<vec3> get_vertices() const override
    {
        std::vector<vec3> tmp;
        return tmp;
    }

     std::vector<edge> get_edges() override {
        std::vector<edge> tmp;
        return tmp;
    }

private:
    std::array<point3, 3> vertices;
    std::array<point3, 3> original_vertices; // original positions of vertices in model
    vec3 current_position = vec3(0);
    quat orientation;
    point3 pivot;

    void update()
    {
        for (int i = 0; i < 3; i++)
        {
            vertices[i] = pivot + orientation.rotate(original_vertices[i] - pivot) + current_position;
        }
    }
};

#endif