#ifndef VR_H
#define VR_H

#include <functional>

#include "GL/glew.h" 
#define OVR_OS_WIN32
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

class Game;

class VR
{
public:
    static void preinit(); // call before creating OpenGL context

    explicit VR(Game &game);
    ~VR();

    void draw(const std::function<void(void)> &drawer) const;
    inline void recenter(void) const { ovrHmd_RecenterPose(m_hmd); }

private:
    void update_fb(void);

    ovrHmd m_hmd;
    ovrEyeRenderDesc m_eye_rdesc[2];
    ovrGLTexture m_ovr_tex[2];
    GLuint m_fbo, m_fb_color, m_fb_depth;
    int m_fb_width, m_fb_height, m_fb_tex_width, m_fb_tex_height;
};

#endif
