#ifndef PHY_BVH_H
#define PHY_BVH_H

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>

#include "aabb.h"
#include "../hittable.h"

    class phy_bvh_node : public phy_hittable
{
public:
    phy_bvh_node() {}
 
    phy_bvh_node(std::vector<std::shared_ptr<phy_hittable>> &objects,
                 size_t start,
                 size_t end)
    {
        size_t object_span = end - start;
 
        if (object_span == 0)
            return; // empty tree: left stays null, hit() returns false
 
        if (object_span == 1)
        {
            // Single object: no right child, so it is only tested once.
            left = objects[start];
        }
        else
        {
            // Split along the longest axis of the combined bounding box.
            phy_aabb span_box = objects[start]->bounding_box();
            for (size_t i = start + 1; i < end; ++i)
                span_box = surrounding_box(span_box, objects[i]->bounding_box());
 
            int axis = longest_axis(span_box);
 
            auto comparator =
                (axis == 0)   ? box_x_compare
                : (axis == 1) ? box_y_compare
                              : box_z_compare;
 
            if (object_span == 2)
            {
                if (comparator(objects[start], objects[start + 1]))
                {
                    left = objects[start];
                    right = objects[start + 1];
                }
                else
                {
                    left = objects[start + 1];
                    right = objects[start];
                }
            }
            else
            {
                std::sort(objects.begin() + start,
                          objects.begin() + end,
                          comparator);
 
                size_t mid = start + object_span / 2;
 
                auto l = std::make_shared<phy_bvh_node>(objects, start, mid);
                auto r = std::make_shared<phy_bvh_node>(objects, mid, end);
 
                left = l;
                right = r;
                left_node = l.get();   // remembered so refit() needs no dynamic_cast
                right_node = r.get();
            }
        }
 
        recompute_bbox();
    }
 
    // Recomputes every node's bounding box bottom-up without changing the tree shape.
    void refit()
    {
        if (!left)
            return;
 
        if (left_node)
            left_node->refit();
        if (right_node)
            right_node->refit();
 
        recompute_bbox();
    }
 
    bool hit(const ray &r,
             interval ray_t,
             phy_hit_record &rec) const override
    {
        if (!left || !bbox.hit(r, ray_t))
            return false;
 
        bool hit_left = left->hit(r, ray_t, rec);
 
        // Shrink the far limit to the closest hit so far so the right side can be culled.
        bool hit_right =
            right &&
            right->hit(r,
                       interval(ray_t.min, hit_left ? rec.t : ray_t.max),
                       rec);
 
        return hit_left || hit_right;
    }
 
    phy_aabb bounding_box() const override
    {
        return bbox;
    }
 
    // A BVH node has no state of its own; update the leaf objects, then call refit().
    void update_state() override {}
 
    // The functions below only make sense for an actual body. rec.hit_object always
    // points at a leaf object, never at a BVH node, so these should never be called.
    // They assert in debug builds so a mistaken call is loud instead of silently wrong.
    double hit_adjuster() const override
    {
        assert(false && "phy_bvh_node::hit_adjuster called; use rec.hit_object instead");
        return 0.0;
    }
 
    bool is_inside(const ray &r) const override
    {
        assert(false && "phy_bvh_node::is_inside called; use rec.hit_object instead");
        return false;
    }
 
    vec3 closest_point_on_surface(const vec3 &pos) const override
    {
        assert(false && "phy_bvh_node::closest_point_on_surface called; use rec.hit_object instead");
        return vec3(0);
    }
 
    std::vector<vec3> get_vertices() const override
    {
        assert(false && "phy_bvh_node::get_vertices called; use rec.hit_object instead");
        return {};
    }
 
private:
    std::shared_ptr<phy_hittable> left;
    std::shared_ptr<phy_hittable> right;   // null when the node holds a single object
 
    // Non-owning typed views of left/right when they are BVH nodes (null for leaf objects).
    phy_bvh_node *left_node = nullptr;
    phy_bvh_node *right_node = nullptr;
 
    phy_aabb bbox;
 
    void recompute_bbox()
    {
        bbox = right
                   ? surrounding_box(left->bounding_box(), right->bounding_box())
                   : left->bounding_box();
    }
 
    static int longest_axis(const phy_aabb &box)
    {
        double sx = box.axis(0).max - box.axis(0).min;
        double sy = box.axis(1).max - box.axis(1).min;
        double sz = box.axis(2).max - box.axis(2).min;
 
        if (sx >= sy && sx >= sz)
            return 0;
        return (sy >= sz) ? 1 : 2;
    }
 
    static bool box_compare(
        const std::shared_ptr<phy_hittable> &a,
        const std::shared_ptr<phy_hittable> &b,
        int axis_index)
    {
        return a->bounding_box().axis(axis_index).min <
               b->bounding_box().axis(axis_index).min;
    }
 
    static bool box_x_compare(
        const std::shared_ptr<phy_hittable> &a,
        const std::shared_ptr<phy_hittable> &b)
    {
        return box_compare(a, b, 0);
    }
 
    static bool box_y_compare(
        const std::shared_ptr<phy_hittable> &a,
        const std::shared_ptr<phy_hittable> &b)
    {
        return box_compare(a, b, 1);
    }
 
    static bool box_z_compare(
        const std::shared_ptr<phy_hittable> &a,
        const std::shared_ptr<phy_hittable> &b)
    {
        return box_compare(a, b, 2);
    }
};

#endif