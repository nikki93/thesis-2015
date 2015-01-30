#include "scene.h"

#include <GL/glew.h>
#include <SDL.h>

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
    m_prev_mouse = Vec3{ x, y };
}

void Scene::draw(const std::vector<Vec3>& points)
{
    glMatrixMode(GL_MODELVIEW);

    // rotate around the scene
    glTranslatef(0, 0, -m_dist);
    glRotatef(m_pitch, 1, 0, 0);
    glRotatef(m_yaw, 0, 1, 0);
    glTranslatef(0, 0, 2);

    // lights
    float lpos[][4] = { { -8, 2, 10, 1 }, { 0, 15, 0, 1 } };
    float lcol[][4] = { { 0.8f, 0.8f, 0.8f, 1 }, { 0.4f, 0.3f, 0.3f, 1 } };
    for (unsigned int i = 0; i < 2; i++)
    {
        glLightfv(GL_LIGHT0 + i, GL_POSITION, lpos[i]);
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lcol[i]);
    }

    // points
    glBegin(GL_POINTS);
    for (std::vector<Vec3>::const_iterator point = points.begin();
         point != points.end(); ++point)
         glVertex3fv(point->gl());
    glEnd();
}
