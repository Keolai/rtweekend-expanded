#ifndef FORCE_H
#define FORCE_H

class force
{
public:
    force(){}
    double strength = 0;
    vec3 direction = vec3(0);

    virtual vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const {
        return ((double)strength * mass) * direction ;
    }
};
class local_force : public force
{
    public:
    vec3 position = vec3(0);
    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override{
        return (std::sqrt((object_position - position).length()) * (double)strength * mass) * direction;
    }
};

class wind_resistance : public force {
public:
    wind_resistance(double coefficient, double area, double fluid_density = 1.225)
        : drag_coefficient(coefficient), cross_section_area(area), fluid_density(fluid_density) {}

    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override
    {
        double speed = object_velocity.length();
        if (speed < 1e-6) return vec3(0.);

        vec3 drag_dir = -unit_vector(object_velocity);
        double drag_magnitude = 0.5 * fluid_density * drag_coefficient * cross_section_area * speed * speed;
        return drag_dir * drag_magnitude;
    }

private:
    double drag_coefficient;
    double cross_section_area;
    double fluid_density;
};
#endif