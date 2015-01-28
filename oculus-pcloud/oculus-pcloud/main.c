#include "main.h"

#include <stdio.h>
#include <stdbool.h>
#include <GL/glew.h>
#include <SDL.h>

#include "maths.h"
#include "error.h"
#include "vr.h"
#include "scene.h"

SDL_Window *main_window;
int main_argc;
char **main_argv;

static bool quit = false; /* iff true, quit */

static void _main_init(void)
{
    vr_preinit();

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

    /* init vr, scene */
    vr_init();
    scene_init();

    /* some GL settings */
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);
    glClearColor(0.1f, 0.1f, 0.1f, 1);
}

static void _main_deinit(void)
{
    /* deinit scene, vr */
    scene_deinit();
    vr_deinit();

    /* deinit SDL */
    SDL_Quit();
}

static void _main_events(void)
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
                vr_recenter();
                break;
            }
            break;
        }
    }
}

static void _main_update_fps(void)
{
    static int nframes = 0;
    static Uint32 last = 0;
    Uint32 curr;

    ++nframes;
    curr = SDL_GetTicks();
    Scalar elapsed = 0.001f * (curr - last);
    if (elapsed >= 5)
    {
        printf("fps: %f\n", nframes / elapsed);
        nframes = 0;
        last = curr;
    }
}

static void _main_update(void)
{
    _main_update_fps();
}

static void _main_draw(void)
{
    vr_draw(scene_draw);
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


