#ifndef CLOUD_H
#define CLOUD_H

#include <memory>
#include <GL/glew.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include "point.h"

class Cloud
{
    pcl::PointCloud<Point>::Ptr points;
    GLuint dlist;
    bool dirty;

public:
    Cloud();
    ~Cloud();

    void add(const Point &point);
    void merge(std::shared_ptr<Cloud> other);

    void transform(const mat4 &trans);
    void draw();

    static vec3 offset;
    static vec3 scale;

private:
    void update_dlist(void);
};

#endif