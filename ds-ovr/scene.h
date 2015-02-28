#ifndef SCENE_H
#define SCENE_H

#include "maths.h"
#include "cloud.h"

class Game;

class Scene
{
public:
    void update(void);
    void draw(const Cloud &cloud);

private:
    float m_yaw = 0;
    float m_pitch = 0;
    float m_dist = 2;
    vec2 m_prev_mouse;
};

#endif
