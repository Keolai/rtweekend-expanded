#ifndef STATE_H
#define STATE_H

#include "../../quaternion.h"

class state {
  public:
    state() {}
    vec3 position = vec3(0.);
    vec3 acceleration = vec3(0.);
    vec3 velocity = vec3(0.);

    quat orientation = quat();
    vec3 angular_velocity = vec3(0.); // world space, rad/s
};

inline void copy(state &orig, state &dest){
  dest.position = orig.position;
  dest.acceleration = orig.acceleration;
  dest.velocity = orig.velocity;
  dest.orientation = orig.orientation;
  dest.angular_velocity = orig.angular_velocity;
}

#endif
