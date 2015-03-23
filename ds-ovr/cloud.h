#ifndef CLOUD_H
#define CLOUD_H

#include <GL/glew.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "point.h"

class Cloud
{
    pcl::PointCloud<Point>::Ptr points;
    GLuint texture;
    GLuint dlist;
    bool dlist_dirty;

    void update_dlist(void);

public:
    Cloud();
    ~Cloud();

    void add(const Point &point);

    void set_texture_data(unsigned int width, unsigned int height, const GLvoid *pixels);
    void transform(const mat4 &trans);
    void draw();

    static vec3 offset;
    static vec3 scale;
};

#endif