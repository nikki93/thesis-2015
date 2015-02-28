#ifndef POINT_H
#define POINT_H

#include "maths.h"

struct Point
{
    vec3 position;
    vec2 texcoord;

    explicit Point(const vec3 &p, const vec2 &t = vec2(0, 0))
        : position(p), texcoord(t)
    {
    }
};

#endif