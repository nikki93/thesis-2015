#ifndef MATHS_H
#define MATHS_H

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

using glm::vec2;
using glm::vec3;
using glm::mat4;
using glm::quat;

inline unsigned int next_pow2(unsigned int x)
{
    x -= 1;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x + 1;
}

#endif
