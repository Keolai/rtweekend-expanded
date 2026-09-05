#ifndef MODEL_H
#define MODEL_H

#include "tri.h"

class model : public hittable
{
public:
    std::vector<std::shared_ptr<tri>> triangles;

    int model_id = -1;

    model(int id) : model_id(id) {}
    model(std::vector<std::shared_ptr<tri>> &tris, int id) : model_id(id) { triangles = tris; }

    void add_tri(vec3 v1, vec3 v2, vec3 v3){
        std::shared_ptr<tri> new_tri = std::make_shared<tri>(v1,v2,v3);
        triangles.push_back(new_tri);
    } 

    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
       return false;
    }

    aabb bounding_box() const override //need to add
    {
       return aabb();
    }

    void position(vec3 &newPos)
    {
        pos = newPos;
    }

    void update_state() const override{
        copy(next_state,current_state); //copy the next predicted state to the new state;
    }

private:
   point3 pos = vec3(0);
};

#endif