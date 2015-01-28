extern "C" {
#include "ds.h"
}

#include <DSAPI.h>
#include <DSAPIUtil.h>

extern "C" {
#include "error.h"
}

static DSAPI *api;
static DSThird *third;

void ds_update(void)
{
    error_assert(api->grab());
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

