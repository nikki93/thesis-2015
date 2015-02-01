#ifndef VR_H
#define VR_H

#include <functional>

#include "GL/glew.h" 
#define OVR_OS_WIN32
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>
#include <array>

#include "maths.h"

class Game;

class VR
{
public:
    static void preinit(); // call before creating OpenGL context

    explicit VR(Game &game);
    ~VR();

    void draw(const std::function<void(void)> &drawer) const;
    inline void recenter(void) const { ovrHmd_RecenterPose(m_hmd); }

    // get eye transforms, if 'mid' is true then both are for 'middle eye'
    std::array<mat4, 2> eye_transforms(bool mid = false) const;

private:
    void update_fb(void);
    std::array<mat4, 2> eye_transforms(ovrPosef pose[2], bool mid = false) const;

    ovrHmd m_hmd;
    ovrEyeRenderDesc m_eye_rdesc[2];
    ovrGLTexture m_ovr_tex[2];
    GLuint m_fbo, m_fb_color, m_fb_depth;
    int m_fb_width, m_fb_height, m_fb_tex_width, m_fb_tex_height;
};

#endif
