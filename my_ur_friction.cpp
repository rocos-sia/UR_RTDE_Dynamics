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

bool is_running = false;
int group = 1;
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

        if (is_running)
            csv_record << group << "\t," << direction << "\t," << std::fixed << std::setprecision(10) << current_pos[3] << "\t," << current_vel[3] << "\t," << current_cut[3]<< "\n";
        else
            csv_record << 0 << "\t," << 0 << "\t," << std::fixed << std::setprecision(10) << current_pos[3] << "\t," << current_vel[3] << "\t," << current_cut[3] << "\n";

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
        -90 * deg2rad,
        0,
        -80 * deg2rad,
        0 * deg2rad,
        0,
    };
    signal(SIGINT, raiseFlag);
    rtde_control.moveJ(init_q, 1.05, 1.4, false);
    double velocity = 0;

    std::thread recored_thread{thread_ur_record_data};

    for (; group <= 40 && flag_loop; group++)
    {
        velocity += 0.4250 * deg2rad;
        direction = 0;
        for (; direction < 2 && flag_loop; direction++)
        {
            if (direction % 2 == 0)
                init_q[3] = init_q[3] + 20 * deg2rad;
            else
                init_q[3] = init_q[3] - 20 * deg2rad;
            is_running = true;
            std::cout << group << "\t," << direction << "\t," << init_q[3] * 180 / 3.14159267 << "\t," << velocity * 180 / 3.14159267 << std::endl;
            rtde_control.moveJ(init_q, velocity, 400 * deg2rad, false);
            is_running = false;

            sleep(1);
        }
    }

    // Stop the RTDE control script
    rtde_control.stopScript();
    std::cout  << "正在退出" << std::endl;
    csv_record.close();

    sleep(2);
    return 0;
}
