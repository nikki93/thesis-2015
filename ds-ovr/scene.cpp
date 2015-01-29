#include "scene.h"

#include <GL/glew.h>

void Scene::draw(const std::vector<Vec3>& points)
{
    glMatrixMode(GL_MODELVIEW);

    // rotate around the scene
    glTranslatef(0, 0, -2);
    m_yaw += 0.3f;
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
    glPointSize(4);
    glBegin(GL_POINTS);
    for (std::vector<Vec3>::const_iterator point = points.begin();
         point != points.end(); ++point)
         glVertex3fv(point->gl());
    glEnd();
}
