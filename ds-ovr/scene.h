#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "maths.h"

class Game;

class Scene
{
public:
    void update(void);
    void draw(const std::vector<Vec3> &points);

private:
    Scalar m_yaw = 0;
    Scalar m_pitch = 0;
    Scalar m_dist = 1;
    Vec3 m_prev_mouse;
};

#endif
