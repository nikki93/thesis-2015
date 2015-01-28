#include <stdio.h>
#include <stdbool.h>
#include <GL/glew.h>
#include <SDL.h>

#include "array.h"
#include "maths.h"
#include "main.h"

static Array *points; /* the points to show */

#define DEFAULT_SCENE_PATH \
    "../../urban_scenes_velodyne/urban_scenes_velodyne/scene-1"

void scene_init()
{
    FILE *f;
    char dummy1[256], dummy2[256];
    Vec3 point;

    /* read in points */
    points = array_new(Vec3);
    f = fopen(main_argc > 1 ? main_argv[1] : DEFAULT_SCENE_PATH, "r");
    while (!feof(f))
        if (fscanf(f, "%s %f %f %f %s\n",
            dummy1, &point.x, &point.y, &point.z, dummy2) == 5)
            array_add_val(Vec3, points) = vec3(point.x, point.z, -point.y);
}

void scene_deinit()
{
    array_free(points);
}

void scene_draw()
{
    unsigned int i, npoints;
    float gray[] = { 0.8f, 0.8f, 0.8f, 1 };
    float red[] = { 0.8f, 0.2f, 0.2f, 1 };
    float lpos[][4] = { { -8, 2, 10, 1 }, { 0, 15, 0, 1 } };
    float lcol[][4] = { { 0.8f, 0.8f, 0.8f, 1 }, { 0.4f, 0.3f, 0.3f, 1 } };

    glMatrixMode(GL_MODELVIEW);

    /* lights */
    for (i = 0; i < 2; i++)
    {
        glLightfv(GL_LIGHT0 + i, GL_POSITION, lpos[i]);
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lcol[i]);
    }

    /* points */
    glPointSize(4);
    glBegin(GL_POINTS);
    npoints = array_length(points);
    for (i = 0; i < npoints; ++i)
        glVertex3fv(array_get(points, i));
    glEnd();
}

