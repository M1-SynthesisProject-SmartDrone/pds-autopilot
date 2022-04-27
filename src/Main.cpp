
#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <signal.h>

#include <loguru/loguru.hpp>

using namespace std;

int main(int argc, char* argv[])
{
    srand(time(0));
    loguru::init(argc, argv);
    LOG_F(INFO, "Start the test autopilot program");



    LOG_F(INFO, "End of the program test autopilot");
    return EXIT_SUCCESS;
}