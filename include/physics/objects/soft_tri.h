#ifndef PHYSICS_SOFT_TRI_H
#define PHYSICS_SOFT_TRI_H

#include "tri.h"
#include "spring.h"

// A triangle that references shared soft body nodes instead of owning its vertices for easy deforms
class phy_soft_tri : public phy_hittable
{
public:
    phy_soft_tri(const std::vector<soft_node> *nodes, int i0, int i1, int i2) : nodes(nodes)
    {
        indices[0] = i0;
        indices[1] = i1;
        indices[2] = i2;
        is_static = false;
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) override
    {
        if (ray_tri_hit(r, ray_t, vertex(0), vertex(1), vertex(2), rec))
        {
            rec.hit_object = shared_from_this();
            return true;
        }
        return false;
    }

    phy_aabb bounding_box() const override
    {
        return tri_bounding_box(vertex(0), vertex(1), vertex(2));
    }

    const point3 &vertex(int i) const
    {
        return (*nodes)[indices[i]].pos;
    }

    int index(int i) const
    {
        return indices[i];
    }

    void position(vec3 &pos) override {} // nodes are moved by the soft body

    void update_state() override
    {
        copy(next_state, current_state);
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
        return {vertex(0), vertex(1), vertex(2)};
    }

    std::vector<edge> get_edges() override {
        std::vector<edge> tmp;
        return tmp;
    }

    bool edge_hit(edge &e, phy_hit_record &rec) override {
        return false;
    }

private:
    const std::vector<soft_node> *nodes; //this is defined in spring.h, I know weird
    int indices[3];
};

#endif
