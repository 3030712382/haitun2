#include "Logitech_controller.h"

using namespace std;

int main()
{
    char path[] = "/dev/input/js0";
    Logitech gamepad(path);

    gamepad.init();
    gamepad.listen_input();

    return 0;
}
