#ifndef CLOUD_H
#define CLOUD_H

#include <vector>
#include <GL/glew.h>

#include "point.h"

class Cloud
{
    std::vector<Point> points;
    GLuint texture;
    GLuint dlist;
    bool dlist_dirty;

    void update_dlist(void);

public:
    explicit Cloud(const mat4 &trans = mat4());
    ~Cloud();

    void reserve(unsigned int n);
    void add(const Point &point);

    void set_texture_data(unsigned int width, unsigned int height, const GLvoid *pixels);
    void draw();

    const std::vector<Point> &get_points(void) { return points; }

    static vec3 offset;
    static vec3 scale;
    mat4 transform;
};

#endif