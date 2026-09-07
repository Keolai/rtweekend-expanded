#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H

#include "hittable.h"

#include <vector>

using std::make_shared;
using std::shared_ptr;

class hittable_list : public hittable
{
public:
    std::vector<shared_ptr<hittable>> objects;
    int cur_id = 0;

    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }

    void add(shared_ptr<hittable> object)
    {
        object->id = cur_id;
        objects.push_back(object);
        cur_id++;
    }

    void add(shared_ptr<hittable> object, int id) //incase it is a sub-object of a main object
    {
        objects.push_back(object);
        object->id = id;
        cur_id++; //just incase
    }

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        hit_record temp_rec;
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

    aabb bounding_box() const override
    {
        if (objects.empty())
            return aabb();

        aabb output_box;
        bool first_box = true;

        for (const auto &object : objects)
        {
            aabb obj_box = object->bounding_box();

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

    int size(){
        return objects.size();
    }

    void position(vec3 &pos){
        return;
    }
};

#endif