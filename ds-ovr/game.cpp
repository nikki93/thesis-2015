#include "game.h"

#include <cstdio>
#include <GL/glew.h>
#include <SDL.h>

#include "error.h"
#include "vr.h"
#include "ds.h"
#include "scene.h"

Game::Game(int argc, char **argv) :
    m_argc(argc), m_argv(argv)
{
    // init SDL
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    m_window = SDL_CreateWindow("ds-ovr",
                                SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                1280, 800,
                                SDL_WINDOW_OPENGL);
    if (!m_window)
        throw Error("couldn't create window");
    if (!SDL_GL_CreateContext(m_window))
        throw Error("couldn't create OpenGL context");

    // init GLEW
    glewInit();

    // GL settings
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
    glClearColor(0.1f, 0.1f, 0.1f, 1);
}

Game::~Game()
{
    // deinit SDL
    SDL_Quit();
}

void Game::loop(const std::function<void(void)> &update,
                const std::function<void(void)> &draw,
                const std::function<void(const SDL_Event &)> &ev)
{
    while (!m_quit)
    {
        events(ev);

        update();
        update_fps();

        draw();
    }
}


void Game::events(const std::function<void(const SDL_Event &)> &handler)
{
    SDL_Event ev;

    while (SDL_PollEvent(&ev))
    {
        switch (ev.type)
        {
        case SDL_QUIT:
            quit();
            break;

        case SDL_KEYDOWN: case SDL_KEYUP:
            switch (ev.key.keysym.sym)
            {
            case SDLK_ESCAPE:
                quit();
                break;
            }
            break;
        }

        handler(ev);
    }
}

void Game::update_fps()
{
    static auto nframes = 0;
    static Uint32 last = 0;
    Uint32 curr;

    ++nframes;
    curr = SDL_GetTicks();
    auto elapsed = 0.001f * (curr - last);
    if (elapsed >= 5)
    {
        printf("fps: %f\n", nframes / elapsed);
        nframes = 0;
        last = curr;
    }
}
