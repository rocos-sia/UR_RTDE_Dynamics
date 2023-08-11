#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <chrono>
#include <thread>
#include <fstream>


using namespace ur_rtde;
using namespace std::chrono;

bool flag_loop = true;
void raiseFlag(int param)
{
    flag_loop = false;
}

int direction = 0;
std::ofstream csv_record{};

void thread_ur_record_data()
{

    RTDEReceiveInterface rtde_receive("192.168.3.101");
    csv_record.open("/home/k/UR_RTDE_Examples/ur_frcition.csv");

    if (!csv_record.is_open())
    {
        std::cout << "打开失败" << std::endl;
        flag_loop = false;
    }

    while (flag_loop)
    {
        auto t_start = steady_clock::now();

        auto current_pos = rtde_receive.getActualQ();
        auto current_vel = rtde_receive.getActualQd();
        auto current_cut = rtde_receive.getActualCurrent();
        csv_record << 0 << "\t," << 0 << "\t," << std::fixed << std::setprecision(10) << current_vel[3] << "\t," << current_cut[3] << "\t,"
                   << current_pos[0] << "\t," << current_pos[1] << "\t," << current_pos[2] << "\t," << current_pos[3] << "\t,"
                   << current_pos[4] << "\t," << current_pos[5] << "\n";

        auto t_stop = steady_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);
        if (t_duration.count() < 0.002)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.002 - t_duration.count()));
        }
    }
}

int main(int argc, char *argv[])
{
    const double deg2rad = 0.017453292519943295769236907684886127; // PI/180
    RTDEControlInterface rtde_control("192.168.3.101");

    std::vector<double> init_q = {
        0,
        0,
        -120 * deg2rad,
        -120 * deg2rad,
        -120 * deg2rad,
        -120 * deg2rad,
    };
    signal(SIGINT, raiseFlag);
    rtde_control.moveJ(init_q, 0.6, 1.4, false);

    sleep(2);
    std::thread recored_thread{thread_ur_record_data};

    direction = 0;
    for (; direction < 2 && flag_loop; direction++)
    {
        if (direction % 2 == 0)
        {

            init_q[0] = -120 * deg2rad;
            init_q[1] = -120 * deg2rad;
            init_q[2] = 0;
            init_q[3] = 0;
            init_q[4] = 0;
            init_q[5] = 0;
            // for (auto &iot : init_q)
            //     iot = -120 * deg2rad;
        }
        else
        {
            init_q[0] = 0;
            init_q[1] = 0;
            init_q[2] = -120 * deg2rad;
            init_q[3] = -120 * deg2rad;
            init_q[4] = -120 * deg2rad;
            init_q[5] = -120 * deg2rad;

            // for (auto &iot : init_q)
            //     iot = 0;
        }
        rtde_control.moveJ(init_q, 2 * deg2rad, 4 * deg2rad, false);
    }

    // Stop the RTDE control script
    rtde_control.stopScript();
    std::cout << "正在退出" << std::endl;
    recored_thread.join();
    csv_record.close();

    return 0;
}
