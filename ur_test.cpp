#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <vector>
#include <atomic>

using namespace ur_rtde;
using namespace std::chrono;

std::ofstream csv_record{};
bool flag_loop = true;
std::atomic<double> extern_force_Pos_offset{0};
constexpr double deg2rad = 0.017453292519943295769236907684886127; // PI/180
RTDEReceiveInterface rtde_receive("192.168.3.101");

void raiseFlag(int param)
{
    flag_loop = false;
}

int main(int argc, char const *argv[])
{
    /* code */
    signal(SIGINT, raiseFlag);
    while (flag_loop)
    {
        double JointTorqe4 = rtde_receive.getActualCurrent()[3];
        std::cout << "JointTorqe 4 =" << JointTorqe4 << std::endl;
    }

    return 0;
}
