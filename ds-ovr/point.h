#ifndef POINT_H
#define POINT_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "maths.h"

struct Point
{
    PCL_ADD_POINT4D;
    float u, v;
    vec2 texcoord;

    explicit Point(const vec3 &p, const vec2 &t = vec2(0, 0))
        : x(p.x), y(p.y), z(p.z), u(t.x), v(t.y)
    {
    }
};

POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, u, u)
    (float, v, v)
    )

#endif