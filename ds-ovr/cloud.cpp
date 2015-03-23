#include "cloud.h"

#include <glm/gtc/matrix_transform.inl>

vec3 Cloud::offset = vec3(0, 0, -0.49);
vec3 Cloud::scale = vec3(1, 1, 1);

Cloud::Cloud(const mat4 &trans)
    : dlist(0), dlist_dirty(true), transform(trans)
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

void Cloud::reserve(unsigned int n)
{
    points.reserve(n);
}

void Cloud::add(const Point &point)
{
    points.push_back(point);
    dlist_dirty = true;
}

void Cloud::set_texture_data(unsigned int width, unsigned int height, const GLvoid *pixels)
{
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, pixels);
}

void Cloud::draw()
{
    update_dlist();

    glEnable(GL_TEXTURE_2D);
    glPushMatrix();
    auto scaled = scale_slow(transform, scale);
    auto trans = translate(scaled, offset);
    glMultMatrixf(value_ptr(trans));
    glCallList(dlist);
    glPopMatrix();
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
    for (auto point : points)
    {
        glTexCoord2fv(value_ptr(point.texcoord));
        glVertex3fv(value_ptr(point.position));
    }
    glEnd();
    glEndList();

    dlist_dirty = false;
}

