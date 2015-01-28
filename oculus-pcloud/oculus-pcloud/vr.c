#include "vr.h"

#include <stdio.h>
#include <GL/glew.h>
#include <SDL.h>
#define OVR_OS_WIN32
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include "maths.h"
#include "error.h"
#include "main.h"

static unsigned int fbo = 0, fb_color, fb_depth;  /* target framebuffer */
static int fb_width, fb_height;                   /* framebuffer size */
static int fb_tex_width, fb_tex_height;           /* underlying texture size */
static ovrHmd hmd;
static ovrEyeRenderDesc eye_rdesc[2];
static ovrGLTexture fb_ovr_tex[2];
static union ovrGLConfig glcfg;
static unsigned int distort_caps;

void vr_recenter()
{
    ovrHmd_RecenterPose(hmd);
}


/* -------------------------------------------------------------------------- */

void vr_preinit()
{
    ovr_Initialize(); /* must be done before creating OpenGL context */
}

/* resize vr framebuffer, creating if doesn't exist */
static void _vr_update_fb()
{
    unsigned int i;

    /* generate fbo, textures if don't exist; bind fbo */
    if (!fbo)
    {
        glGenFramebuffers(1, &fbo);
        glGenTextures(1, &fb_color);
        glGenRenderbuffers(1, &fb_depth);
        glBindTexture(GL_TEXTURE_2D, fb_color);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    /* underlying texture size is next power of two */
    fb_tex_width = _next_pow2(fb_width);
    fb_tex_height = _next_pow2(fb_height);

    /* color buffer */
    glBindTexture(GL_TEXTURE_2D, fb_color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, fb_tex_width, fb_tex_height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D, fb_color, 0);

    /* depth buffer */
    glBindRenderbuffer(GL_RENDERBUFFER, fb_depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
        fb_tex_width, fb_tex_height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, fb_depth);

    /* check done */
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        error("incomplete framebuffer");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    /* update OVR framebuffer description for each eye */
    for (i = 0; i < 2; ++i)
    {
        fb_ovr_tex[i].OGL.Header.API = ovrRenderAPI_OpenGL;
        fb_ovr_tex[i].OGL.Header.TextureSize.w = fb_tex_width;
        fb_ovr_tex[i].OGL.Header.TextureSize.h = fb_tex_height;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Pos.x = i ? fb_width / 2 : 0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Pos.y = 0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Size.w = fb_width / 2;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Size.h = fb_height;
        fb_ovr_tex[i].OGL.TexId = fb_color;
    }
}

/* init all vr stuff */
void vr_init()
{
    ovrSizei eye_res[2];

    /* create HMD */
    if (!(hmd = ovrHmd_Create(0)))
    {
        fputs("couldn't create Oculus HMD, falling back to debug HMD", stderr);
        if (!(hmd = ovrHmd_CreateDebug(ovrHmd_DK2)))
            error("couldn't create debug HMD");
    }

    /* resize window to HMD resolution */
    SDL_SetWindowSize(main_window, hmd->Resolution.w, hmd->Resolution.h);
    SDL_SetWindowPosition(main_window, hmd->WindowsPos.x, hmd->WindowsPos.y);
    SDL_SetWindowFullscreen(main_window, SDL_WINDOW_FULLSCREEN_DESKTOP);

    /* enable position, rotation tracking */
    ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation
        | ovrTrackingCap_MagYawCorrection
        | ovrTrackingCap_Position, 0);

    /* calculate framebuffer resolution and create framebuffer */
    eye_res[0] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Left,
        hmd->DefaultEyeFov[0], 1.0);
    eye_res[1] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Right,
        hmd->DefaultEyeFov[1], 1.0);
    fb_width = eye_res[0].w + eye_res[1].w;
    fb_height = eye_res[0].h > eye_res[1].h ? eye_res[0].h : eye_res[1].h;
    _vr_update_fb();

    /* fill in ovrGLConfig */
    memset(&glcfg, 0, sizeof glcfg);
    glcfg.OGL.Header.API = ovrRenderAPI_OpenGL;
    glcfg.OGL.Header.RTSize = hmd->Resolution;
    glcfg.OGL.Header.Multisample = 1;
    glcfg.OGL.Window = GetActiveWindow();
    glcfg.OGL.DC = wglGetCurrentDC();
    if (!(hmd->HmdCaps & ovrHmdCap_ExtendDesktop))
        ovrHmd_AttachToWindow(hmd, glcfg.OGL.Window, 0, 0);

    /* enable HMD, distortion capabilities and enable SDK rendering */
    ovrHmd_SetEnabledCaps(hmd, ovrHmdCap_LowPersistence
        | ovrHmdCap_DynamicPrediction);
    if (!ovrHmd_ConfigureRendering(hmd, &glcfg.Config,
        ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette
        | ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive,
        hmd->DefaultEyeFov, eye_rdesc))
        error("failed to configure distortion rendering");

    /* disable health/safety warning */
    void ovrhmd_EnableHSWDisplaySDKRender(ovrHmd hmd, ovrBool enable);
    ovrhmd_EnableHSWDisplaySDKRender(hmd, 0);
}

/* deinit all vr stuff */
void vr_deinit()
{
    if (hmd)
        ovrHmd_Destroy(hmd);
    ovr_Shutdown();
}

/* draw per-eye, calling the given draw callback */
void vr_draw(void(*drawer)(void))
{
    int i;
    ovrMatrix4f proj;
    ovrPosef pose[2];
    float rot_mat[16];

    ovrHmd_BeginFrame(hmd, 0);

    /* draw onto our framebuffer, clear it first */
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* per-eye */
    for (i = 0; i < 2; ++i)
    {
        ovrEyeType eye = hmd->EyeRenderOrder[i];

        /* viewport transform: select left/right based on eye */
        glViewport(eye == ovrEye_Left ? 0 : fb_width / 2, 0,
            fb_width / 2, fb_height);

        /* projection transform: just use OVR's eye matrix */
        proj = ovrMatrix4f_Projection(hmd->DefaultEyeFov[eye], 0.5, 500.0, 1);
        glMatrixMode(GL_PROJECTION);
        glLoadTransposeMatrixf(&proj.M[0][0]); /* GL uses column-major */

        /* view transform: use OVR's eye position, orientation */
        pose[eye] = ovrHmd_GetHmdPosePerEye(hmd, eye); /* TODO: deprecated */
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(eye_rdesc[eye].HmdToEyeViewOffset.x,
            eye_rdesc[eye].HmdToEyeViewOffset.y,
            eye_rdesc[eye].HmdToEyeViewOffset.z);
        _quat_to_matrix(&pose[eye].Orientation.x, rot_mat);
        glMultMatrixf(rot_mat);
        glTranslatef(-pose[eye].Position.x,
            -pose[eye].Position.y,
            -pose[eye].Position.z);
        glTranslatef(0, -ovrHmd_GetFloat(hmd, OVR_KEY_EYE_HEIGHT, 1.65f), 0);

        /* draw! */
        drawer();
    }

    /* draw onto display */
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ovrHmd_EndFrame(hmd, pose, &fb_ovr_tex[0].Texture);

    glUseProgram(0); /* OVR doesn't restore shader program when done */
}


