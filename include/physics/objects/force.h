#ifndef FORCE_H
#define FORCE_H

#include "../utilities/vec3.h"

class world_force
{
public:
    int strength = 0;
    vec3 direction = vec3(0);

    virtual double get_force(vec3 object_position) const {
        return (double)strength;
    }
};
class local_force : public world_force
{
    public:
    vec3 position = vec3(0);
    double get_force(vec3 object_position) const override{
        return std::sqrt((object_position - position).length()) * (double)strength;
    }
};
#endif