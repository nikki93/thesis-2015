#include "cloud.h"

#include <glm/gtc/matrix_transform.inl>

vec3 Cloud::offset = vec3(0, 0, -0.49);
vec3 Cloud::scale = vec3(1, 1, 1);

Cloud::Cloud()
    : dlist(0), dirty(true), points(new pcl::PointCloud<Point>())
{
    update_dlist();
}

Cloud::~Cloud()
{
    if (dlist != 0)
        glDeleteLists(dlist, 1);
}

void Cloud::add(const Point &point)
{
    points->push_back(point);
    dirty = true;
}

void Cloud::merge(std::shared_ptr<Cloud> other)
{
    *points += *other->points;
    dirty = true;
}

void Cloud::transform(const mat4 &trans)
{
    auto etrans = Eigen::Matrix4f(value_ptr(trans));
    transformPointCloud(*points, *points, etrans);
    dirty = true;
}

void Cloud::draw()
{
    update_dlist();
    glCallList(dlist);
}

void Cloud::update_dlist(void)
{
    if (!dirty)
        return;

    if (dlist != 0)
        glDeleteLists(dlist, 1);

    dlist = glGenLists(1);
    glNewList(dlist, GL_COMPILE);
    glBegin(GL_POINTS);
    for (auto point : *points)
    {
        glColor3f(point.r / 255.f, point.g / 255.f, point.b / 255.f);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
    glEndList();

    dirty = false;
}

