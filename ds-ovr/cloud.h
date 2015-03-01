#ifndef CLOUD_H
#define CLOUD_H

#include <vector>
#include <GL/glew.h>

#include "point.h"

class Cloud
{
    std::vector<Point> points;
    mat4 transform;
    GLuint texture;

public:
    explicit Cloud(const mat4 &trans = mat4())
        : transform(trans)
    {
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    }

    ~Cloud()
    {
        glDeleteTextures(1, &texture);
    }

    void reserve(unsigned int n)
    {
        points.reserve(n);
    }

    void add(const Point &point)
    {
        points.push_back(point);
    }

    void set_texture_data(unsigned int width, unsigned int height, const GLvoid *pixels)
    {
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height,
                     0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, pixels);
    }

    void draw() const
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture);
        glPushMatrix();
        glMultMatrixf(value_ptr(transform));
        glBegin(GL_POINTS);
        for (auto point : points)
        {
            glTexCoord2fv(value_ptr(point.texcoord));
            glVertex3fv(value_ptr(point.position));
        }
        glEnd();
        glPopMatrix();
    }
};

#endif