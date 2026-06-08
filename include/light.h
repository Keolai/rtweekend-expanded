#ifndef LIGHT_H
#define LIGHT_H

#include "vec3.h"
#include "color.h"

class light {
  public:
    light() {}
    double power = 75;

    light(const point3& origin, const color light_color) : orig(origin), light_color(light_color) { radius = 1.;}
    light(const point3& origin, const color light_color, const double power) : orig(origin), light_color(light_color), power(power) { radius = 1.;}

    const point3& origin() const  { return orig; }
    const vec3& get_color() const { return light_color; }
    const float get_radius() const {return radius;}

  private:
    point3 orig;
    color light_color;
    float radius; 
};

#endif