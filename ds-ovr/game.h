#ifndef GAME_H
#define GAME_H

#include <functional>
#include <SDL.h>

class DS;
class VR;
class Scene;

class Game
{
public:
    Game(int argc, char **argv);
    ~Game();

    // run game, calling event handlers
    void loop(const std::function<void(float)> &update,
              const std::function<void(void)> &draw,
              const std::function<void(const SDL_Event &)> &ev);

    inline void quit(void) { m_quit = true; }
    inline SDL_Window *window(void) const { return m_window; }
    inline int argc(void) const { return m_argc; }
    inline char **argv(void) const { return m_argv; }

private:
    void events(const std::function<void(const SDL_Event &)> &handler);
    void update_fps(void);

    SDL_Window *m_window = nullptr;
    int m_argc;
    char **m_argv;
    bool m_quit = false;
};

#endif
