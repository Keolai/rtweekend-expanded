#ifndef PHY_HITTABLE_H
#define PHY_HITTABLE_H

#include "utilities/aabb.h"
#include "utilities/state.h"
#include <vector>

#define EDGE_TOLERANCE 1e-12
#define EDGE_THICKNESS 1e-1

struct edge
{ // this is for edge-edge
    vec3 *u = nullptr;
    vec3 *v = nullptr;

    const vec3 &start() const { return *u; }
    const vec3 &end() const { return *v; }
};

class phy_hittable;

class phy_hit_record
{
public:
    point3 p;
    vec3 normal;
    std::shared_ptr<phy_hittable> hit_object = nullptr;

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

class phy_hittable : public std::enable_shared_from_this<phy_hittable>
{
public:
    virtual ~phy_hittable() = default;
    virtual phy_aabb bounding_box() const = 0; // need?

    virtual bool hit(const ray &r, interval ray_t, phy_hit_record &rec) = 0;
    virtual double hit_adjuster() const = 0;
    virtual bool is_inside(const ray &r) const = 0;
    virtual vec3 closest_point_on_surface(const vec3 &pos) const = 0;
    virtual std::vector<vec3> get_vertices() const = 0;
    virtual void position(vec3 &pos) = 0;
    virtual std::vector<edge> get_edges() = 0;
    virtual bool edge_hit(edge &e, phy_hit_record &rec) = 0;

    state current_state = state();
    state next_state = state();
    int id = -1;

    double mass = 1;
    double restitution = 0.4;
    double friction = 0.1;

    virtual void update_state() = 0;
    bool is_static = true; // dont move!
protected:
    // closest contact between e and the 3 sides of a triangle, within EDGE_THICKNESS.
    // rec.t is the distance, rec.bay_coord is (along e, along the side, which side), the side runs from corner z to corner (z + 1) % 3
    bool edge_tri_hit(edge &e, const vec3 &v0, const vec3 &v1, const vec3 &v2, phy_hit_record &rec)
    {
        const vec3 *corners[3] = {&v0, &v1, &v2};
        bool hit = false;
        double best = EDGE_THICKNESS * EDGE_THICKNESS;

        for (int side = 0; side < 3; side++)
        {
            const vec3 &my_start = *corners[side];
            const vec3 &my_end = *corners[(side + 1) % 3];

            double s, t;
            vec3 c_e, c_my;
            double d2 = closest_pt_seg_seg(e.start(), e.end(), my_start, my_end, s, t, c_e, c_my);
            if (d2 >= best)
            {
                continue;
            }

            vec3 n = c_e - c_my; // points from this object toward the incoming edge
            double len = std::sqrt(d2);
            if (len > 1e-9)
            {
                n = n / len;
            }
            else
            {
                n = cross(e.end() - e.start(), my_end - my_start); // touching: use the edges' cross product
                if (n.length_squared() < EDGE_TOLERANCE)
                {
                    continue; // parallel and touching, no usable direction
                }
                n = unit_vector(n);
            }

            best = d2;
            hit = true;

            rec.p = c_my;
            rec.normal = n;
            rec.t = len;
            rec.bay_coord = vec3(s, t, side);
            rec.hit_object = shared_from_this();
        }
        return hit;
    }

    inline double closest_pt_seg_seg(const vec3 &p1, const vec3 &q1,
                                     const vec3 &p2, const vec3 &q2,
                                     double &s, double &t, vec3 &c1, vec3 &c2)
    {
        vec3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        double a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);

        if (a <= EDGE_TOLERANCE && e <= EDGE_TOLERANCE)
        {
            s = t = 0.0;
        }
        else if (a <= EDGE_TOLERANCE)
        {
            s = 0.0;
            t = std::min(1.0, std::max(0.0, f / e));
        }
        else
        {
            double c = dot(d1, r);
            if (e <= EDGE_TOLERANCE)
            {
                t = 0.0;
                s = std::min(1.0, std::max(0.0, -c / a));
            }
            else
            {
                double b = dot(d1, d2);
                double denom = a * e - b * b; // 0 when parallel
                if (denom > EDGE_TOLERANCE)
                {
                    s = std::min(1.0, std::max(0.0, (b * f - c * e) / denom));
                }
                else
                {
                    s = 0.0;
                }
                t = (b * s + f) / e;
                if (t < 0.0)
                {
                    t = 0.0;
                    s = std::min(1.0, std::max(0.0, -c / a));
                }
                else if (t > 1.0)
                {
                    t = 1.0;
                    s = std::min(1.0, std::max(0.0, (b - c) / a));
                }
            }
        }
        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
        vec3 diff = c1 - c2;
        return dot(diff, diff);
    }
};

#endif