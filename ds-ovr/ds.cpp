#include "ds.h"

#include <DSAPIUtil.h>
#include <glm/gtc/matrix_transform.inl>

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
    error_assert(m_api->enableRight(true));
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
        hardware->setAutoExposure(DS_BOTH_IMAGERS, false);
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

static double unit_random(void)
{
    return ((double) rand()) / RAND_MAX;
}

std::shared_ptr<Cloud> DS::cloud(const VR &vr, float factor)
{
    error_assert(m_api->grab());
    DSCalibIntrinsicsRectified z_intrin, t_intrin;
    double t_trans[3];
    error_assert(m_api->getCalibIntrinsicsZ(z_intrin));
    error_assert(m_third->getCalibIntrinsicsRectThird(t_intrin));
    error_assert(m_third->getCalibExtrinsicsZToRectThird(t_trans));
    auto img = m_api->getZImage();
    auto width = m_api->zWidth(), height = m_api->zHeight();

    auto cloud = std::make_shared<Cloud>();
    //cloud->set_texture_data(m_third->thirdwidth(), m_third->thirdheight(),
    //                       m_third->getthirdimage());
    auto texture = (uint8_t *) m_third->getThirdImage();
    auto t_width = m_third->thirdWidth(), t_height = m_third->thirdHeight();
    for (float j = 0; j < height; ++j)
        for (float i = 0; i < width; ++i)
            if (auto d = *img++)
            {
                if (factor < 1 && unit_random() > factor)
                    continue;

                float z_img[]{ i, j, d }, z_camera[3], t_camera[3], t_image[2];
                DSTransformFromZImageToZCamera(z_intrin, z_img, z_camera);
                DSTransformFromZCameraToRectThirdCamera(t_trans, z_camera, t_camera);
                DSTransformFromThirdCameraToRectThirdImage(t_intrin, t_camera, t_image);

                Point p;
                p.x = DSConvertZUnitsToM(z_camera[0], m_api->getZUnits());
                p.y = -DSConvertZUnitsToM(z_camera[1], m_api->getZUnits());
                p.z = -DSConvertZUnitsToM(z_camera[2], m_api->getZUnits());

                float u = t_image[0] / t_intrin.rw, v = t_image[1] / t_intrin.rh;
                unsigned int imgx = t_width * u, imgy = t_height * v;
                if (imgx >= t_width) imgx = t_width;
                if (imgy >= t_height) imgy = t_height;
                uint8_t *pix = &texture[4 * (imgx + t_width * imgy)];
                p.b = pix[0];
                p.g = pix[1];
                p.r = pix[2];
                p.a = 1.0f;

                cloud->add(p);
            }
    auto trans = translate(vr.eye_transforms(true)[0], vec3(0, 0, 0));
    cloud->transform(trans);
    return cloud;
}
