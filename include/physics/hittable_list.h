#ifndef PHY_HITTABLE_LIST_H
#define PHY_HITTABLE_LIST_H

#include "hittable.h"

#include <vector>

using std::make_shared;
using std::shared_ptr;

class phy_hittable_list : public phy_hittable
{
public:
    std::vector<shared_ptr<phy_hittable>> objects;

    phy_hittable_list() {}
    phy_hittable_list(shared_ptr<phy_hittable> object) { add(object); }

    void clear() { objects.clear(); }

    void add(shared_ptr<phy_hittable> object)
    {
        objects.push_back(object);
    }

    bool hit(const ray &r, interval ray_t, phy_hit_record &rec) const override
    {
        phy_hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;

        for (const auto &object : objects)
        { // iterate thru all objects, bvh should be placed here
            if (object->hit(r, interval(ray_t.min, closest_so_far), temp_rec))
            {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }

    phy_aabb bounding_box() const override // dont really need yet?
    {
        if (objects.empty())
            return phy_aabb();

        phy_aabb output_box;
        bool first_box = true;

        for (const auto &object : objects)
        {
            phy_aabb obj_box = object->bounding_box();

            if (first_box)
            {
                output_box = obj_box;
                first_box = false;
            }
            else
            {
                output_box = surrounding_box(output_box, obj_box);
            }
        }

        return output_box;
    }

    int size() { return objects.size(); }

    void update_state() override
    {
        // ...
    }
};

#endif