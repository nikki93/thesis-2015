#ifndef SCENE_H
#define SCENE_H

#include <vector>

#include "maths.h"

class Game;

class Scene
{
public:
    void draw(const std::vector<Vec3> &points);

private:
    Scalar m_yaw = 0;
};

#endif
