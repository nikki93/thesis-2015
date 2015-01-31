#ifndef DS_H
#define DS_H

#include <DSAPI.h>
#include <vector>

#include "maths.h"

class DS
{
public:
    DS();
    ~DS();

    const std::vector<vec3> &points();

private:
    DSAPI *m_api;
    DSThird *m_third;
    std::vector<vec3> m_points;
};

#endif
