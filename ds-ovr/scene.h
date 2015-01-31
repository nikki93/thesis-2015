#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "maths.h"

class Game;

class Scene
{
public:
    void update(void);
    void draw(const std::vector<vec3> &points);

private:
    float m_yaw = 0;
    float m_pitch = 0;
    float m_dist = 1;
    vec2 m_prev_mouse;
};

#endif
