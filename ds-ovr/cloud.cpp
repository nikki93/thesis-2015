#include "cloud.h"

#include <pcl/registration/icp.h>
#include <glm/gtc/matrix_transform.inl>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

vec3 Cloud::offset = vec3(0, 0, -0.49);
vec3 Cloud::scale = vec3(1, 1, 1);

void Cloud::make_octree(void)
{
    octree.reset(new pcl::octree::OctreePointCloudSearch<Point>(0.012));
    octree->setInputCloud(points);
    octree->addPointsFromInputCloud();
}

void Cloud::make_mesh(void)
{
    if (points->empty())
        return;

    // downsample
    printf("downsampling...\n"); fflush(stdout);
    pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
    pcl::VoxelGrid<Point> sor;
    sor.setInputCloud(points);
    sor.setLeafSize(0.07f, 0.07f, 0.07f);
    sor.filter(*cloud);
    mesh_cloud = cloud;

    // copy to no RGB cloud
    printf("copying...\n"); fflush(stdout);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*mesh_cloud, *cloud2);

    // normal estimation
    printf("estimating normals...\n"); fflush(stdout);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud2);
    n.setInputCloud(cloud2);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    pcl::concatenateFields(*cloud2, *normals, *cloud_with_normals);

    //pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //mls.setComputeNormals(true);
    //mls.setInputCloud(cloud);
    //mls.setPolynomialFit(true);
    //mls.setSearchMethod(tree);
    //mls.setSearchRadius(0.03);
    //mls.process(*cloud_with_normals);

    // search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // perform gp3
    printf("triangulating...\n"); fflush(stdout);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius(0.3);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    mesh.reset(new pcl::PolygonMesh());
    gp3.reconstruct(*mesh);
}

Cloud::Cloud(bool mk_octree)
    : points(new pcl::PointCloud<Point>())
{
    if (mk_octree)
        make_octree();
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
    {
        for (auto &point : *(other->points))
            octree->addPointToCloud(point, points);

        //*points += *other->points;
        //pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
        //pcl::VoxelGrid<Point> sor;
        //sor.setInputCloud(points);
        //sor.setLeafSize(0.07f, 0.07f, 0.07f);
        //sor.filter(*cloud);
        //points = cloud;
        //make_octree();
    }
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
    if (mesh)
    {
        glBegin(GL_TRIANGLES);
        for (auto &poly : mesh->polygons)
            if (poly.vertices.size() == 3)
                for (auto &ind : poly.vertices)
                {
                    Point &point = (*mesh_cloud)[ind];
                    glColor3ub(point.r, point.g, point.b);
                    glVertex3f(point.x, point.y, point.z);
                }
        glEnd();
    }

    glBegin(GL_POINTS);
    for (auto &point : *points)
    {
        glColor3ub(point.r, point.g, point.b);
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();
    return;

    glEnableClientState(GL_VERTEX_ARRAY);
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

