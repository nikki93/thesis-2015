#include "vr.h"

#include <iostream>
#include <SDL.h>

#include "error.h"
#include "game.h"
#include "maths.h"

extern "C"
{
    void ovrhmd_EnableHSWDisplaySDKRender(ovrHmd hmd, ovrBool enable);
}

void VR::preinit()
{
    ovr_Initialize();
}

VR::VR(Game &game)
{
    // create HMD
    if (!(m_hmd = ovrHmd_Create(0)))
    {
        std::cerr << "couldn't create Oculus HMD, falling back to debug HMD"
            << std::endl;
        if (!(m_hmd = ovrHmd_CreateDebug(ovrHmd_DK2)))
            throw Error("couldn't create debug HMD");
    }

    // set window resolution
    SDL_SetWindowSize(game.window(), m_hmd->Resolution.w, m_hmd->Resolution.h);
    SDL_SetWindowPosition(game.window(),
                          m_hmd->WindowsPos.x, m_hmd->WindowsPos.y);
    SDL_SetWindowFullscreen(game.window(), SDL_WINDOW_FULLSCREEN_DESKTOP);

    // enable position, rotation tracking
    ovrHmd_ConfigureTracking(m_hmd, ovrTrackingCap_Orientation
        | ovrTrackingCap_MagYawCorrection
        | ovrTrackingCap_Position, 0);

    // calculate framebuffer resolution and create framebuffer
    ovrSizei eye_res[2];
    eye_res[0] = ovrHmd_GetFovTextureSize(m_hmd, ovrEye_Left,
        m_hmd->DefaultEyeFov[0], 1.0);
    eye_res[1] = ovrHmd_GetFovTextureSize(m_hmd, ovrEye_Right,
        m_hmd->DefaultEyeFov[1], 1.0);
    m_fb_width = eye_res[0].w + eye_res[1].w;
    m_fb_height = eye_res[0].h > eye_res[1].h ? eye_res[0].h : eye_res[1].h;
    update_fb();

    // fill in ovrGLConfig
    ovrGLConfig glcfg;
    memset(&glcfg, 0, sizeof glcfg);
    glcfg.OGL.Header.API = ovrRenderAPI_OpenGL;
    glcfg.OGL.Header.RTSize = m_hmd->Resolution;
    glcfg.OGL.Header.Multisample = 1;
    glcfg.OGL.Window = GetActiveWindow();
    glcfg.OGL.DC = wglGetCurrentDC();
    if (!(m_hmd->HmdCaps & ovrHmdCap_ExtendDesktop))
        ovrHmd_AttachToWindow(m_hmd, glcfg.OGL.Window, 0, 0);

    // enable HMD, distortion capabilities and enable SDK rendering
    ovrHmd_SetEnabledCaps(m_hmd, ovrHmdCap_LowPersistence
        | ovrHmdCap_DynamicPrediction);
    if (!ovrHmd_ConfigureRendering(m_hmd, &glcfg.Config,
        ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette
        | ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive,
        m_hmd->DefaultEyeFov, m_eye_rdesc))
        throw Error("failed to configure distortion rendering");

    // disable health/safety warning
    ovrhmd_EnableHSWDisplaySDKRender(m_hmd, 0);
}

VR::~VR()
{
    if (m_hmd)
        ovrHmd_Destroy(m_hmd);
    ovr_Shutdown();
}

void VR::draw(const std::function<void()>& drawer) const
{
    ovrPosef pose[2];

    ovrHmd_BeginFrame(m_hmd, 0);

    // draw onto our framebuffer, clearing it first
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // per-eye
    for (auto i = 0; i < 2; ++i)
    {
        auto eye = m_hmd->EyeRenderOrder[i];

        // viewport transform: select left/right based on eye
        glViewport(eye == ovrEye_Left ? 0 : m_fb_width / 2, 0,
            m_fb_width / 2, m_fb_height);

        // projection transform: just use OVR's eye matrix
        auto proj = ovrMatrix4f_Projection(m_hmd->DefaultEyeFov[eye],
                                           0.5, 500.0, 1);
        glMatrixMode(GL_PROJECTION);
        glLoadTransposeMatrixf(&proj.M[0][0]); // GL uses column-major

        // view transform: use OVR's eye position, orientation
        pose[eye] = ovrHmd_GetHmdPosePerEye(m_hmd, eye); // TODO: deprecated
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(m_eye_rdesc[eye].HmdToEyeViewOffset.x,
            m_eye_rdesc[eye].HmdToEyeViewOffset.y,
            m_eye_rdesc[eye].HmdToEyeViewOffset.z);
        float rot_mat[16];
        quat_to_matrix(&pose[eye].Orientation.x, rot_mat);
        glMultMatrixf(rot_mat);
        glTranslatef(-pose[eye].Position.x,
            -pose[eye].Position.y,
            -pose[eye].Position.z);
        glTranslatef(0, -ovrHmd_GetFloat(m_hmd, OVR_KEY_EYE_HEIGHT, 1.65f), 0);

        // draw!
        drawer();
    }

    // draw onto display
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ovrHmd_EndFrame(m_hmd, pose, &m_ovr_tex[0].Texture);

    glUseProgram(0); // OVR doesn't restore shader program when done
}


void VR::update_fb()
{
    // generate fbo, textures if don't exist; bind fbo
    if (!m_fbo)
    {
        glGenFramebuffers(1, &m_fbo);
        glGenTextures(1, &m_fb_color);
        glGenRenderbuffers(1, &m_fb_depth);
        glBindTexture(GL_TEXTURE_2D, m_fb_color);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

    // underlying texture size is next power of two
    m_fb_tex_width = next_pow2(m_fb_width);
    m_fb_tex_height = next_pow2(m_fb_height);

    // color buffer
    glBindTexture(GL_TEXTURE_2D, m_fb_color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_fb_tex_width, m_fb_tex_height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D, m_fb_color, 0);

    // depth buffer
    glBindRenderbuffer(GL_RENDERBUFFER, m_fb_depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
        m_fb_tex_width, m_fb_tex_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, m_fb_depth);

    // check done
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw Error("incomplete framebuffer");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // update OVR framebuffer description for each eye
    for (unsigned int i = 0; i < 2; ++i)
    {
        m_ovr_tex[i].OGL.Header.API = ovrRenderAPI_OpenGL;
        m_ovr_tex[i].OGL.Header.TextureSize.w = m_fb_tex_width;
        m_ovr_tex[i].OGL.Header.TextureSize.h = m_fb_tex_height;
        m_ovr_tex[i].OGL.Header.RenderViewport.Pos.x = i ? m_fb_width / 2 : 0;
        m_ovr_tex[i].OGL.Header.RenderViewport.Pos.y = 0;
        m_ovr_tex[i].OGL.Header.RenderViewport.Size.w = m_fb_width / 2;
        m_ovr_tex[i].OGL.Header.RenderViewport.Size.h = m_fb_height;
        m_ovr_tex[i].OGL.TexId = m_fb_color;
    }
}

