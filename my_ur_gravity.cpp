#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <chrono>
#include <thread>
#include <fstream>
#include  "interpolate.h"

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
    csv_record.open("ur_frcition.csv");

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
        175 * deg2rad,
        175 * deg2rad,
        175 * deg2rad,
        175 * deg2rad,
        175 * deg2rad,
        175 * deg2rad,
    };
    signal(SIGINT, raiseFlag);
    rtde_control.moveJ(init_q, 0.6, 1.4, false);

    sleep(2);
    std::thread recored_thread{thread_ur_record_data};

    rocos::R_INTERP T_VEL;
    bool isplanned = T_VEL.planTrapezoidProfile(0, 175 * deg2rad, -175 * deg2rad, 0, 0, 1 * deg2rad, 3 * deg2rad);
    if (!isplanned)
    {
        std::cout << "规划失败" << std::endl;
        flag_loop = false;
        exit(-1);
    }
    double t_total_1 =  T_VEL.getDuration();

    double pos_lim = (175) * deg2rad;
    double vel_lim = (3) * deg2rad;

    double velocity       = 0.5;
    double acceleration   = 0.5;
    double servo_dt             = 1.0 / 500;  // 2ms
    double lookahead_time = 0.1;
    double gain           = 300;

    for(double dt = 0;dt<=t_total_1;dt+=0.002)
    {
        auto t_start = high_resolution_clock::now( );

        init_q[3] = T_VEL.pos(dt);
        double ref_pos = pos_lim * cos(vel_lim * dt / pos_lim);
        init_q[0] = ref_pos;
        init_q[1] = ref_pos;
        init_q[2] = ref_pos;
        init_q[4] = ref_pos;
        init_q[5] = ref_pos;
        rtde_control.servoJ( init_q, velocity, acceleration, servo_dt, lookahead_time, gain );
        auto t_stop     = high_resolution_clock::now( );
        auto t_duration = std::chrono::duration< double >( t_stop - t_start );
         if ( t_duration.count( ) < 0.002 )
        {
            std::this_thread::sleep_for( std::chrono::duration< double >(  0.002  - t_duration.count( ) ) );
        }
    }


     isplanned = T_VEL.planTrapezoidProfile(0,-175 * deg2rad, 175 * deg2rad, 0, 0, 1 * deg2rad, 3 * deg2rad);
    if (!isplanned)
    {
        std::cout << "规划失败" << std::endl;
        flag_loop = false;
        exit(-1);
    }
    double t_total_2 =  T_VEL.getDuration();

    for(double dt = t_total_1;dt<=t_total_1+t_total_2;dt+=0.002)
    {
        auto t_start = high_resolution_clock::now( );

        init_q[3] = T_VEL.pos(dt-t_total_1);
        double ref_pos = pos_lim * cos(vel_lim * dt / pos_lim);
        init_q[0] = ref_pos;
        init_q[1] = ref_pos;
        init_q[2] = ref_pos;
        init_q[4] = ref_pos;
        init_q[5] = ref_pos;
        rtde_control.servoJ( init_q, velocity, acceleration, servo_dt, lookahead_time, gain );
        auto t_stop     = high_resolution_clock::now( );
        auto t_duration = std::chrono::duration< double >( t_stop - t_start );
         if ( t_duration.count( ) < 0.002 )
        {
            std::this_thread::sleep_for( std::chrono::duration< double >(  0.002  - t_duration.count( ) ) );
        }
    }


    // Stop the RTDE control script
    rtde_control.stopScript();
    std::cout << "正在退出" << std::endl;
    recored_thread.join();
    csv_record.close();

    return 0;
}
