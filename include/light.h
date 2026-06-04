#ifndef LIGHT_H
#define LIGHT_H

#include "vec3.h"
#include "color.h"

class light {
  public:
    light() {}

    light(const point3& origin, const color light_color) : orig(origin), light_color(light_color) {}

    const point3& origin() const  { return orig; }
    const vec3& get_color() const { return light_color; }

  private:
    point3 orig;
    color light_color;
};

#endif