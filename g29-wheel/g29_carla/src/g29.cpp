#include "g29.hpp"

int main(int argc, char** argv)
{
    G29FFB g29ffb("/dev/input/event-g29", true);
    while (true) {
        try
        {
            if (g29ffb.cmd_deque_.size() > 0) {
                std::cout << g29ffb.cmd_deque_.back().enter << std::endl;
            }
            for (; g29ffb.cmd_deque_.size() > 1;) 
            {
                g29ffb.cmd_deque_.pop_front();
            }
            sleep(1);
        }
        catch(const std::exception& e)
        {
            break;
        }
    }
    return 0;
}