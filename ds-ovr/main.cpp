#include <exception>
#include <iostream>

#include "game.h"
#include "vr.h"
#include "scene.h"
#include "ds.h"
#include "cloud.h"

static bool update_transform()
{
    auto state = SDL_GetKeyboardState(nullptr);
    float amount = 0.001;

    if (state[SDL_SCANCODE_LCTRL])
        amount *= 5;

    if (state[SDL_SCANCODE_W])
    {
        Cloud::offset += vec3(0, 0, -amount);
        return true;
    }
    if (state[SDL_SCANCODE_S])
    {
        Cloud::offset += vec3(0, 0,  amount);
        return true;
    }
    if (state[SDL_SCANCODE_A])
    {
        Cloud::offset += vec3( amount, 0, 0);
        return true;
    }
    if (state[SDL_SCANCODE_D])
    {
        Cloud::offset += vec3(-amount, 0, 0);
        return true;
    }
    if (state[SDL_SCANCODE_R])
    {
        Cloud::offset += vec3(0,  amount, 0);
        return true;
    }
    if (state[SDL_SCANCODE_F])
    {
        Cloud::offset += vec3(0, -amount, 0);
        return true;
    }
    if (state[SDL_SCANCODE_X])
    {
        Cloud::offset = vec3(0, 0, 0);
        return true;
    }

    if (state[SDL_SCANCODE_Q])
    {
        Cloud::scale *= 1.1;
        return true;
    }
    if (state[SDL_SCANCODE_E])
    {
        Cloud::scale /= 1.1;
        return false;
    }

    return false;
}

int main(int argc, char **argv)
{
    try
    {
        VR::preinit();

        Game game(argc, argv);
        VR vr(game);
        Scene scene;
        DS ds;

        game.loop([&]()
        {
            scene.update();
            if (update_transform()) {
                std::cout << "offset: " << Cloud::offset.x << ", "
                    << Cloud::offset.y << ", " << Cloud::offset.z << std::endl;
                std::cout << "scale: " << Cloud::scale.x << ", "
                    << Cloud::scale.y << ", " << Cloud::scale.z << std::endl;
            }
        },
            [&]()
        {
            auto cloud = ds.cloud(vr);
            vr.draw([&]() { scene.draw(cloud); });
        },
            [&](const SDL_Event &ev)
        {
            switch (ev.type)
            {
            case SDL_KEYDOWN: case SDL_KEYUP:
                switch (ev.key.keysym.sym)
                {
                case SDLK_SPACE:
                    vr.recenter();
                    break;

                case SDLK_c:
                    scene.add_cloud(ds.cloud(vr));
                    break;
                }
            }
        });
    }
    catch (std::exception &e)
    {
        std::cerr << "caught exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

int cpp_main(int argc, char ** const argv)
{
    return main(argc, argv);
}
