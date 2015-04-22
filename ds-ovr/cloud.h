#ifndef CLOUD_H
#define CLOUD_H

#include <memory>
#include <GL/glew.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "point.h"

class Cloud
{
    pcl::PointCloud<Point>::Ptr points;
    pcl::octree::OctreePointCloudSearch<Point>::Ptr octree = nullptr;
    GLuint dlist = 0;
    bool dirty = true;

public:
    Cloud(bool mk_octree = false);
    ~Cloud();

    void make_octree(void);
    void make_mesh(void);

    void resize(unsigned int n);
    void add(const Point &point);
    void merge(std::shared_ptr<Cloud> other);
    void register_merge(std::shared_ptr<Cloud> other);

    void transform(const mat4 &trans);
    void draw();

    // paint along pointing direction a --> b
    void paint(const vec3 &a, const vec3 &b)
    {
        if (!octree)
            return;

        auto dir = normalize(b - a);
        Eigen::Vector3f origin(a.x, a.y, a.z), direction(dir.x, dir.y, dir.z);

        pcl::octree::OctreePointCloudSearch<Point>::AlignedPointTVector hit_points;
        octree->getIntersectedVoxelCenters(origin, direction, hit_points, 1);

        if (hit_points.empty())
            return;

        auto dist = distance(a, vec3(hit_points[0].x, hit_points[0].y, hit_points[0].z));
        auto corrected = a + dist * dir;
        Point corrected_p;
        corrected_p.x = corrected.x;
        corrected_p.y = corrected.y;
        corrected_p.z = corrected.z;

        std::vector<int> hits;
        std::vector<float> dists;
        octree->radiusSearch(corrected_p, 0.03, hits, dists);
        for (auto &hit : hits)
        {
            Point &p = (*points)[hit];
            p.r = 0;
            p.g = 0;
            p.b = 255;
        }
    }

    vec3 finger;
    bool has_finger = false;

    pcl::PolygonMesh::Ptr mesh = nullptr;
    pcl::PointCloud<Point>::Ptr mesh_cloud = nullptr;

    static vec3 offset;
    static vec3 scale;

private:
    void update_dlist(void);
};

#endif