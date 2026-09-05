#ifndef STATE_H
#define STATE_H

#include "vec3.h"

class state {
  public:
    state() {}
    vec3 position = vec3(0.);
    vec3 acceleration = vec3(0.);
    vec3 velocity = vec3(0.);
};

#endif