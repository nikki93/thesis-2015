#ifndef DS_H
#define DS_H

#include <memory>
#include <DSAPI.h>

#include "cloud.h"
#include "vr.h"

class DS
{
public:
    DS();
    ~DS();

    std::shared_ptr<Cloud> cloud(const VR &vr);

private:
    DSAPI *m_api;
    DSThird *m_third;
};

#endif
