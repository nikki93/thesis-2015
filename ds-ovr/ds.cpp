#include "ds.h"

#include <DSAPIUtil.h>

#include "error.h"

DS::DS() :
    m_api(DSCreate(DS_DS4_PLATFORM)),
    m_third(m_api->accessThird())
{
    // valid?
    error_assert(m_api->probeConfiguration());
    error_assert(m_api->isCalibrationValid());

    // configure core Z-from-stereo capabilities
    error_assert(m_api->enableZ(true));
    error_assert(m_api->enableLeft(true));
    error_assert(m_api->enableRight(false));
    error_assert(m_api->setLRZResolutionMode(true, 480, 360, 60, DS_LUMINANCE8));
    m_api->enableLRCrop(true);

    // configure third camera
    if (m_third)
    {
        m_third->enableThird(true);
        m_third->setThirdResolutionMode(true, 640, 480, 30, DS_BGRA8);
    }

    // change exposure, gain
    if (auto hardware = m_api->accessHardware())
    {
        error_assert(hardware->setImagerExposure(16.3f, DS_BOTH_IMAGERS));
        error_assert(hardware->setImagerGain(2.0f, DS_BOTH_IMAGERS));
    }

    // begin capturing
    error_assert(m_api->startCapture());
}

DS::~DS()
{
    DSDestroy(m_api);
}

const std::vector<Vec3> &DS::points()
{
    error_assert(m_api->grab());
    DSCalibIntrinsicsRectified z_intrin;
    error_assert(m_api->getCalibIntrinsicsZ(z_intrin));
    auto img = m_api->getZImage();
    auto width = m_api->zWidth(), height = m_api->zHeight();

    m_points.clear();
    m_points.reserve(width * height);
    for (float j = 0; j < height; ++j)
        for (float i = 0; i < width; ++i)
            if (auto d = *img++)
            {
                float z_img[]{ i, j, d }, z_camera[3];
                DSTransformFromZImageToZCamera(z_intrin, z_img, z_camera);
                m_points.push_back(Vec3{
                    z_camera[0] / 400,
                    -z_camera[1] / 300 + 1,
                    -z_camera[2] / 300 - 0.2f
                });
            }
    return m_points;
}
