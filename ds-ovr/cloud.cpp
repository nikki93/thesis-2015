#include "cloud.h"

#include <glm/gtc/matrix_transform.inl>

vec3 Cloud::offset = vec3(0, 0, -0.49);
vec3 Cloud::scale = vec3(1, 1, 1);

Cloud::Cloud()
    : dlist(0), dlist_dirty(true), points(new pcl::PointCloud<Point>())
{
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    update_dlist();
}

Cloud::~Cloud()
{
    if (dlist != 0)
        glDeleteLists(dlist, 1);
    glDeleteTextures(1, &texture);
}

void Cloud::add(const Point &point)
{
    points->push_back(point);
    dlist_dirty = true;
}

void Cloud::set_texture_data(unsigned int width, unsigned int height, const GLvoid *pixels)
{
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, pixels);
}

void Cloud::transform(const mat4 &trans)
{
    auto etrans = Eigen::Matrix4f(value_ptr(trans));
    transformPointCloud(*points, *points, etrans);
    dlist_dirty = true;
}

void Cloud::draw()
{
    update_dlist();

    glEnable(GL_TEXTURE_2D);
    glCallList(dlist);
}

void Cloud::update_dlist(void)
{
    if (!dlist_dirty)
        return;

    if (dlist != 0)
        glDeleteLists(dlist, 1);

    dlist = glGenLists(1);
    glNewList(dlist, GL_COMPILE);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBegin(GL_POINTS);
    for (auto point : *points)
    {
        glTexCoord2f(point.u, point.v);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
    glEndList();

    dlist_dirty = false;
}

