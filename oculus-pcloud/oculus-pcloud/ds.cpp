extern "C" {
#include "ds.h"
}

#include <cstdio>
#include <DSAPI.h>
#include <DSAPIUtil.h>
#include <GL/glew.h>
#include <algorithm>

extern "C" {
#include "error.h"
#include "array.h"
#include "scene.h"
#include "maths.h"
}

static DSAPI *api;
static DSThird *third;

void ds_update(void)
{
    auto points = array_new(Vec3);

    /* read from camera */
    error_assert(api->grab());
    auto img = api->getZImage();
    auto width = api->zWidth(), height = api->zHeight();
    DSCalibIntrinsicsRectified z_intrin;
    error_assert(api->getCalibIntrinsicsZ(z_intrin));
    for (int j = 0; j < height; ++j)
        for (int i = 0; i < width; ++i)
            if (auto d = *img++)
            {
                float z_img[] = { float(i), float(j), d }, z_camera[3];
                DSTransformFromZImageToZCamera(z_intrin, z_img, z_camera);
                array_add_val(Vec3, points) = vec3(
                    z_camera[0] / 400,
                    -z_camera[1] / 300 + 1,
                    -z_camera[2] / 300 - 0.2
                    );
            }

    scene_set_points(points);
}

void ds_init(void)
{
    api = DSCreate(DS_DS4_PLATFORM);
    error_assert(api->probeConfiguration());

    /* check calibration data */
    error_assert(api->isCalibrationValid());

    /* configure core Z-from-stereo capabilities */
    error_assert(api->enableZ(true));
    error_assert(api->enableLeft(true));
    error_assert(api->enableRight(false));
    error_assert(api->setLRZResolutionMode(true, 480, 360, 60, DS_LUMINANCE8));
    api->enableLRCrop(true);

    /* configure third camera */
    if (third)
    {
        third->enableThird(true);
        third->setThirdResolutionMode(true, 640, 480, 30, DS_BGRA8);
    }

    /* change exposure, gain */
    if (auto hardware = api->accessHardware())
    {
        error_assert(hardware->setImagerExposure(16.3f, DS_BOTH_IMAGERS));
        error_assert(hardware->setImagerGain(2.0f, DS_BOTH_IMAGERS));
    }

    /* begin capturing */
    error_assert(api->startCapture());
}

void ds_deinit(void)
{
    DSDestroy(api);
}

