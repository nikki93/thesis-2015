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
            auto points = ds.points();
            vr.draw([&]() { scene.draw(points); });
        });
    }
    catch (std::exception &e)
    {
        std::cerr << "caught exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}