#include <stdio.h>
#include <stdbool.h>
#include <GL/glew.h>
#include <SDL.h>
#define OVR_OS_WIN32
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include "array.h"

static bool quit = false;

SDL_Window *main_window;
int main_argc;
char **main_argv;

/* print error message and exit */
void error(const char *fmt)
{
    fputs(stderr, fmt);
    exit(1);
}


/* --- types ---------------------------------------------------------------- */

typedef float Scalar;

typedef struct Vec3 Vec3;
struct Vec3 { Scalar x, y, z; };

#define vec3(x, y, z) ((Vec3) { (x), (y), (z) })


/* --- vr ------------------------------------------------------------------- */

static unsigned int fbo = 0, fb_color, fb_depth;  /* target framebuffer */
static int fb_width, fb_height;                   /* framebuffer size */
static int fb_tex_width, fb_tex_height;           /* underlying texture size */
static ovrHmd hmd;
static ovrEyeRenderDesc eye_rdesc[2];
static ovrGLTexture fb_ovr_tex[2];
static union ovrGLConfig glcfg;
static unsigned int distort_caps;

/* return 2^i for smallest i such that 2^i >= x */
static unsigned int _next_pow2(unsigned int x)
{
    x -= 1;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x + 1;
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
        fb_ovr_tex[i].OGL.Header.RenderViewport.Pos.x = i ? fb_width / 2.0 : 0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Pos.y = 0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Size.w = fb_width / 2.0;
        fb_ovr_tex[i].OGL.Header.RenderViewport.Size.h = fb_height;
        fb_ovr_tex[i].OGL.TexId = fb_color;
    }
}

/* init all vr stuff */
static void _vr_init()
{
    ovrSizei eye_res[2];

    /* create HMD */
    if (!(hmd = ovrHmd_Create(0)))
    {
        fputs(stderr, "couldn't create Oculus HMD, falling back to debug HMD");
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
static void _vr_deinit()
{
    if (hmd)
        ovrHmd_Destroy(hmd);
    ovr_Shutdown();
}

/* convert quaternion to rotation matrix */
static void _quat_to_matrix(const float *quat, float *mat)
{
    mat[0] = 1.0 - 2.0 * quat[1] * quat[1] - 2.0 * quat[2] * quat[2];
    mat[4] = 2.0 * quat[0] * quat[1] + 2.0 * quat[3] * quat[2];
    mat[8] = 2.0 * quat[2] * quat[0] - 2.0 * quat[3] * quat[1];
    mat[12] = 0.0f;

    mat[1] = 2.0 * quat[0] * quat[1] - 2.0 * quat[3] * quat[2];
    mat[5] = 1.0 - 2.0 * quat[0] * quat[0] - 2.0 * quat[2] * quat[2];
    mat[9] = 2.0 * quat[1] * quat[2] + 2.0 * quat[3] * quat[0];
    mat[13] = 0.0f;

    mat[2] = 2.0 * quat[2] * quat[0] + 2.0 * quat[3] * quat[1];
    mat[6] = 2.0 * quat[1] * quat[2] - 2.0 * quat[3] * quat[0];
    mat[10] = 1.0 - 2.0 * quat[0] * quat[0] - 2.0 * quat[1] * quat[1];
    mat[14] = 0.0f;

    mat[3] = mat[7] = mat[11] = 0.0f;
    mat[15] = 1.0f;
}

/* draw per-eye, calling the given draw callback */
static void _vr_draw(void(*drawer)(void))
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
        glTranslatef(0, -ovrHmd_GetFloat(hmd, OVR_KEY_EYE_HEIGHT, 1.65), 0);

        /* draw! */
        drawer();
    }

    /* draw onto display */
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ovrHmd_EndFrame(hmd, pose, &fb_ovr_tex[0].Texture);

    glUseProgram(0); /* OVR doesn't restore shader program when done */
}


/* --- scene ---------------------------------------------------------------- */

#define DEFAULT_SCENE_PATH \
    "../../urban_scenes_velodyne/urban_scenes_velodyne/scene-1"

Array *points;

static void _scene_init()
{
    FILE *f;
    char *path, dummy1[256], dummy2[256];
    Vec3 point;

    /* read in points */
    points = array_new(Vec3);
    f = fopen(main_argc > 1 ? main_argv[1] : DEFAULT_SCENE_PATH, "r");
    while (!feof(f))
        if (fscanf(f, "%s %f %f %f %s\n",
            dummy1, &point.x, &point.y, &point.z, dummy2) == 5)
            array_add_val(Vec3, points) = vec3(point.x, point.z, -point.y);
}

/* draw the scene */
static void _scene_draw()
{
    unsigned int i, npoints;
    float gray[] = { 0.8, 0.8, 0.8, 1 };
    float red[] = { 0.8, 0.2, 0.2, 1 };
    float lpos[][4] = { { -8, 2, 10, 1 }, { 0, 15, 0, 1 } };
    float lcol[][4] = { { 0.8, 0.8, 0.8, 1 }, { 0.4, 0.3, 0.3, 1 } };

    glMatrixMode(GL_MODELVIEW);

    /* lights */
    for (i = 0; i < 2; i++) {
        glLightfv(GL_LIGHT0 + i, GL_POSITION, lpos[i]);
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, lcol[i]);
    }

    /* points */
    glPointSize(4);
    glColor3f(0.0f, 1.0f, 1.0f);
    glBegin(GL_POINTS);
    npoints = array_length(points);
    for (i = 0; i < npoints; ++i)
        glVertex3fv(array_get(points, i));
    glEnd();
}


/* --- main ----------------------------------------------------------------- */

static void _main_init()
{
    ovr_Initialize(); /* must be done before creating OpenGL context */

    /* init SDL, create window, GL context */
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    main_window = SDL_CreateWindow("oculus-test",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        1280, 800,
        SDL_WINDOW_OPENGL);
    if (!main_window)
        error("couldn't create window");
    if (!SDL_GL_CreateContext(main_window))
        error("couldn't create OpenGL context");

    /* init GLEW */
    glewInit();

    /* init vr */
    _vr_init();

    /* some GL settings */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);
    glClearColor(0.1, 0.1, 0.1, 1);

    /* init scene */
    _scene_init();
}

static void _main_deinit()
{
    /* deinit vr */
    _vr_deinit();

    /* deinit SDL */
    SDL_Quit();
}

static void _main_events()
{
    SDL_Event ev;

    while (SDL_PollEvent(&ev))
    {
        switch (ev.type)
        {
        case SDL_QUIT:
            quit = true;
            break;

        case SDL_KEYDOWN: case SDL_KEYUP:
            switch (ev.key.keysym.sym)
            {
            case SDLK_ESCAPE:
                quit = true;
                break;

            case SDLK_SPACE: case SDLK_r:
                /* recenter on SPACE or R */
                ovrHmd_RecenterPose(hmd);
                break;
            }
            break;
        }
    }
}

static void _main_update()
{

}

static void _main_draw()
{
    _vr_draw(_scene_draw);
}

int main(int argc, char **argv)
{
    main_argc = argc;
    main_argv = argv;

    _main_init();

    while (!quit)
    {
        _main_events();
        _main_update();
        _main_draw();
    }

    _main_deinit();
    return 0;
}
