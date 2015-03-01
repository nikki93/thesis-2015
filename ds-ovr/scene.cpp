#include "scene.h"

#include <GL/glew.h>
#include <SDL.h>

void Scene::add_cloud(std::shared_ptr<Cloud> cloud)
{
    clouds.push_back(cloud);
}

void Scene::update(void)
{
    // pitch/yaw using mouse
    int x, y;
    auto state = SDL_GetMouseState(&x, &y);
    if (state & SDL_BUTTON_LMASK)
    {
        m_yaw += 0.3f * (x - m_prev_mouse.x);
        m_pitch += 0.3f * (y - m_prev_mouse.y);
    }
    if (state & SDL_BUTTON_MMASK)
        m_dist += 0.05f * (y - m_prev_mouse.y);
    m_prev_mouse = vec2(x, y);
}

void Scene::draw(std::shared_ptr<Cloud> cloud)
{
    glMatrixMode(GL_MODELVIEW);

    // rotate around the scene
    glTranslatef(0, 0, -m_dist);
    glRotatef(m_pitch, 1, 0, 0);
    glRotatef(m_yaw, 0, 1, 0);
    glTranslatef(0, 0, 2);

    // clouds
    cloud->draw();
    for (auto &c : clouds)
        c->draw();
}

