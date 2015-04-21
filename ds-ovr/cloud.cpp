#include "cloud.h"

#include <pcl/registration/icp.h>
#include <glm/gtc/matrix_transform.inl>

vec3 Cloud::offset = vec3(0, 0, -0.49);
vec3 Cloud::scale = vec3(1, 1, 1);

Cloud::Cloud(bool make_octree)
    : dlist(0), dirty(true), points(new pcl::PointCloud<Point>()), has_finger(false), octree(nullptr)
{
    if (make_octree)
    {
        octree.reset(new pcl::octree::OctreePointCloudSearch<Point>(0.012));
        octree->setInputCloud(points);
        octree->addPointsFromInputCloud();
    }
    update_dlist();
}

Cloud::~Cloud()
{
    if (dlist != 0)
        glDeleteLists(dlist, 1);
}

void Cloud::resize(unsigned int n)
{
    points->resize(n);
}

void Cloud::add(const Point &point)
{
    if (octree)
        octree->addPointToCloud(point, points);
    else
        points->push_back(point);
    dirty = true;
}

void Cloud::merge(std::shared_ptr<Cloud> other)
{
    if (octree)
        for (auto &point : *(other->points))
            octree->addPointToCloud(point, points);
    else
        *points += *other->points;
    dirty = true;
}

void Cloud::register_merge(std::shared_ptr<Cloud> other)
{
    if (points->empty())
    {
        merge(other);
        return;
    }

    pcl::PointCloud<Point>::Ptr final(new pcl::PointCloud<Point>());

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(other->points);
    icp.setInputTarget(points);
    icp.align(*final);

    *points += *final;
    dirty = true;
}


void Cloud::transform(const mat4 &trans)
{
    auto etrans = Eigen::Matrix4f(value_ptr(trans));
    transformPointCloud(*points, *points, etrans);
    finger = (trans * vec4(finger, 1)).xyz;
    dirty = true;
}

void Cloud::draw()
{
    glBegin(GL_POINTS);
    for (auto &point : *points)
    {
        glColor3ub(point.r, point.g, point.b);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
    glEnableClientState(GL_VERTEX_ARRAY);
    return;

    //glEnableClientState(GL_COLOR_ARRAY);

    glColor3f(1, 1, 1);
    glVertexPointer(3, GL_FLOAT, sizeof(Point), &((*points)[0].x));
    //glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(Point), &((*points)[0].b));
    glDrawArrays(GL_POINT, 0, points->size());

    glDisableClientState(GL_VERTEX_ARRAY);
    //glDisableClientState(GL_COLOR_ARRAY);
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
    for (auto &point : *points)
    {
        glColor3ub(point.r, point.g, point.b);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
    glEndList();

    dirty = false;
}

