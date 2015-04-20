#ifndef SCENE_H
#define SCENE_H

#include <list>
#include <memory>

#include "maths.h"
#include "cloud.h"

class Game;

class Scene
{
public:
    void add_cloud(std::shared_ptr<Cloud> cloud);

    void update(void);
    void draw(std::shared_ptr<Cloud> cloud);

    void reset_view(void)
    {
        m_yaw = 0;
        m_pitch = 0;
        m_dist = 2;
    }

private:
    float m_yaw = 0;
    float m_pitch = 0;
    float m_dist = 2;
    vec2 m_prev_mouse;

    Cloud main_cloud;
};

#endif
