#ifndef CLOUD_H
#define CLOUD_H

#include <vector>
#include <GL/glew.h>

#include "point.h"

class Cloud
{
    std::vector<Point> points;
    mat4 transform;

public:
    explicit Cloud(const mat4 &trans)
        : transform(trans)
    {
    }

    void reserve(unsigned int n)
    {
        points.reserve(n);
    }

    void add(const Point &point)
    {
        points.push_back(point);
    }

    void draw() const
    {
        glPushMatrix();
        glMultMatrixf(value_ptr(transform));
        glBegin(GL_POINTS);
        for (auto point : points)
            glVertex3fv(value_ptr(point.position));
        glEnd();
        glPopMatrix();
    }
};

#endif