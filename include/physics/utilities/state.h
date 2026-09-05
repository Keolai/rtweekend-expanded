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

inline void copy(state orig, state dest){
  dest.position = orig.position;
  dest.acceleration = orig.acceleration;
  dest.velocity = orig.velocity;
}

#endif