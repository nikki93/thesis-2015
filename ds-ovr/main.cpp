#include <exception>
#include <iostream>

#include "game.h"
#include "vr.h"
#include "scene.h"
#include "ds.h"

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
        },
                  [&]()
        {
            auto cloud = ds.cloud(vr);
            vr.draw([&]() { scene.draw(cloud); });
        });
    }
    catch (std::exception &e)
    {
        std::cerr << "caught exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}