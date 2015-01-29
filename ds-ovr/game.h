#ifndef GAME_H
#define GAME_H

#include <functional>
#include <SDL.h>

class VR;
class Scene;

class Game
{
public:
    Game(int argc, char **argv);
    ~Game();

    // run game, calling update and draw callbacks
    void loop(const std::function<void(void)> &update,
              const std::function<void(void)> &draw);

    inline void quit(void) { m_quit = true; }
    inline SDL_Window *window(void) const { return m_window; }
    inline int argc(void) const { return m_argc; }
    inline char **argv(void) const { return m_argv; }

private:
    void events(void);
    void update_fps(void);

    SDL_Window *m_window = nullptr;
    int m_argc;
    char **m_argv;
    bool m_quit = false;
};

#endif
