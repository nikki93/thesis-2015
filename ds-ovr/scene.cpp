#include "scene.h"

#include <GL/glew.h>
#include <SDL.h>

void Scene::add_cloud(std::shared_ptr<Cloud> cloud)
{
    main_cloud.merge(cloud);
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
    main_cloud.draw();
    if (cloud)
    {
        cloud->draw();
        if (cloud->has_finger)
        {
            glLineWidth(2.5);
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            glVertex3fv(value_ptr(cloud->finger));
            auto end = cloud->finger + vec3(0, 0.2, 0);
            glVertex3fv(value_ptr(end));
            glEnd();
        }
    }
}

