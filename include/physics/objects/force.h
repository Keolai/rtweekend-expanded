#ifndef FORCE_H
#define FORCE_H

class force
{
public:
    force() {}
    double strength = 0;
    vec3 direction = vec3(0);

    virtual vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const
    {
        return ((double)strength * mass) * direction;
    }
};
class local_force : public force
{
public:
    vec3 position = vec3(0);
    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override
    {
        return (std::sqrt((object_position - position).length()) * (double)strength * mass) * (-1) * direction;
    }
};

class point_force : public local_force
{
public:
    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override
    {
        return (std::sqrt((object_position - position).length()) * (double)strength * mass) * (object_position - position);
    }
};

class wind_resistance : public force
{
public:
    // drag coefficient is weird
    wind_resistance(double coefficient, double area, double fluid_density = 1.225) // normal air
        : drag_coefficient(coefficient), cross_section_area(area), fluid_density(fluid_density)
    {
    }

    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override
    {
        double speed = object_velocity.length();
        if (speed < 1e-6)
            return vec3(0.);

        vec3 drag_dir = -unit_vector(object_velocity);
        double drag_force = 0.5 * fluid_density * drag_coefficient * cross_section_area * (speed * speed);
        return drag_dir * drag_force;
    }

private:
    double drag_coefficient;
    double cross_section_area;
    double fluid_density;
};

class vortex_force : public force
{
public:
    vortex_force(vec3 position, vec3 axis, double field_strength, double falloff_power = 1.0)
        : position(position), axis(unit_vector(axis)), field_strength(field_strength), falloff_power(falloff_power) {}

    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override
    {
        vec3 to_object = object_position - position;
        vec3 radial = to_object - dot(to_object, axis) * axis;
        double dist = radial.length();

        if (dist < 1e-6)
            return vec3(0.);

        double clamped_dist = std::max(dist, 0.5);
        vec3 tangent_dir = unit_vector(cross(axis, radial));
        double magnitude = field_strength * mass / std::pow(clamped_dist, falloff_power);
        return tangent_dir * magnitude;
    }

private:
    vec3 position;
    vec3 axis;
    double field_strength; // controls spin direction
    double falloff_power;  // 0 = uniform swirl strength regardless of distance, 1+ = weaker further out
};

class range_force : public force
{
public:
    range_force(vec3 position, double target_distance, double strength)
        : position(position), target_distance(target_distance), strength(strength) {}

    vec3 get_force(vec3 &object_position, vec3 &object_velocity, double mass) const override
    {
        vec3 to_object = object_position - position;
        double dist = to_object.length();
        if (dist < 1e-6) return vec3(0.);

        vec3 dir = to_object / dist;

        double error = dist - target_distance;
        double magnitude = -strength * mass * error; 

        return dir * magnitude;
    }

private:
    vec3 position;
    double target_distance;
    double strength;
};

#endif