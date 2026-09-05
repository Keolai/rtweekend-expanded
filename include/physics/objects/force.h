#ifndef FORCE_H
#define FORCE_H

class force
{
public:
    int strength = 0;
    vec3 direction = vec3(0);

    virtual vec3 get_force(vec3 object_position, double mass) const {
        return ((double)strength * mass) * direction ;
    }
};
class local_force : public force
{
    public:
    vec3 position = vec3(0);
    vec3 get_force(vec3 object_position, double mass) const override{
        return (std::sqrt((object_position - position).length()) * (double)strength * mass) * direction;
    }
};
#endif