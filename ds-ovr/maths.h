#ifndef MATHS_H
#define MATHS_H

#include <GL/glew.h>


typedef float Scalar;


struct Vec3
{
    Scalar x, y, z;

    const GLfloat *gl(void) const { return &x; }
};


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

inline void quat_to_matrix(const Scalar *quat, Scalar *mat)
{
    mat[0] = 1.0f - 2.0f * quat[1] * quat[1] - 2.0f * quat[2] * quat[2];
    mat[4] = 2.0f * quat[0] * quat[1] + 2.0f * quat[3] * quat[2];
    mat[8] = 2.0f * quat[2] * quat[0] - 2.0f * quat[3] * quat[1];
    mat[12] = 0.0f;

    mat[1] = 2.0f * quat[0] * quat[1] - 2.0f * quat[3] * quat[2];
    mat[5] = 1.0f - 2.0f * quat[0] * quat[0] - 2.0f * quat[2] * quat[2];
    mat[9] = 2.0f * quat[1] * quat[2] + 2.0f * quat[3] * quat[0];
    mat[13] = 0.0f;

    mat[2] = 2.0f * quat[2] * quat[0] + 2.0f * quat[3] * quat[1];
    mat[6] = 2.0f * quat[1] * quat[2] - 2.0f * quat[3] * quat[0];
    mat[10] = 1.0f - 2.0f * quat[0] * quat[0] - 2.0f * quat[1] * quat[1];
    mat[14] = 0.0f;

    mat[3] = mat[7] = mat[11] = 0.0f;
    mat[15] = 1.0f;
}

#endif
