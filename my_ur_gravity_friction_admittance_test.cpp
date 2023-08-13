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

void raiseFlag(int param)
{
    flag_loop = false;
}

double &my_vector_at(std::vector<double> &buffer_data, int index)
{
    return buffer_data[index - 1];
}

double my_filter(const double &x, int windowsize, bool reset = false)
{
    static std::vector<double> buffer_data(windowsize);
    static int buffer_index = 0;
    static double sum = 0;
    static bool is_filled{false};

    double mean_value{0};
    if (reset)
    {
        buffer_data = std::vector<double>(windowsize);
        buffer_index = 0;
        sum = 0;
        is_filled = false;
        return 0;
    }

    buffer_index = buffer_index + 1;

    if (buffer_index == windowsize + 1)
        buffer_index = 1;

    if (is_filled)
    {
        sum = sum - my_vector_at(buffer_data, buffer_index);
        sum = sum + x;
        my_vector_at(buffer_data, buffer_index) = x;
        mean_value = sum / windowsize;
    }

    else
    {
        sum = sum + x;
        my_vector_at(buffer_data, buffer_index) = x;
        mean_value = sum / buffer_index;
    }

    if (!is_filled && buffer_index == windowsize)
        is_filled = true;

    return mean_value;
}

double my_sign(double x)
{
    if (x > 0)
        return 1.0;
    else if (x == 0) // 一定要是==
        return 0.0;
    else
        return -1.0;
}

double my_abs(double x)
{
    if (x > 0)
        return x;
    else if (x == 0)
        return 0;
    else
        return -x;
}

double joint_4_gravity_friction(double Q2, double Q3, double Q4, double Q5, double Q6, double x, double static_frition )
{
    static constexpr std::array<double, 8> slover = {0.2372, -0.0025, -0.0057, -0.0665, 0.0282, 0.0145, 0.0512, 0.0699};

    double gravity =
        sin(Q2 + Q3 + Q4) * slover[0] + cos(Q2 + Q3 + Q4) * sin(Q5) * slover[1] + cos(Q2 + Q3 + Q4) * slover[2] + cos(Q5) * cos(Q2 + Q3 + Q4) * slover[3] + cos(Q6) * sin(Q2 + Q3 + Q4) * slover[4] + sin(Q6) * sin(Q2 + Q3 + Q4) * slover[5] + cos(Q5) * cos(Q6) * cos(Q2 + Q3 + Q4) * slover[6] + cos(Q5) * cos(Q2 + Q3 + Q4) * sin(Q6) * slover[7];

    static constexpr double a = 0.2954;
    static constexpr double b = -0.07984;
    static constexpr double c = 24.78;
    static constexpr double d = 0.2347;

    double friction = (!my_sign(x)) * static_frition + my_sign(x) * (a + b * exp(-c * my_abs(x)) + d * my_abs(x));

    std::cout << "摩擦力 = " << friction << std::endl;
    std::cout << "重力 = " << gravity << std::endl;
    return friction + gravity;
}
double one_freedom_admittance(double force)
{
    static double force_vel_offset = 0;
    static double force_pos_offset = 0;
    static double force_last_acc_offset = 0;
    static double force_last_vel_offset = 0;
    static double dt = 0.002;
    static double K = 0;
    static double M = 0.5;
    static double B =1;

    double force_acc_offset = (force - B * force_vel_offset - K * force_pos_offset) / M;
    force_vel_offset = dt * (force_acc_offset + force_last_acc_offset) / 2 + force_vel_offset;
    force_pos_offset = dt * (force_vel_offset + force_last_vel_offset) / 2 + force_pos_offset;

    force_last_acc_offset = force_acc_offset;
    force_last_vel_offset = force_vel_offset;

    return force_pos_offset;
}

void thread_ur_record_data()
{
    RTDEReceiveInterface rtde_receive("192.168.3.101");
    static double static_frition = 0;

    csv_record.open("/home/k/UR_RTDE_Examples/ur_test.csv");

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
        double real_current = my_filter(current_cut[3], 500);

        csv_record << 0 << "\t,"
                   << 0 << "\t," << std::fixed << std::setprecision(10) << current_vel[3] << "\t," << current_cut[3] << "\t,"
                   << current_pos[0] << "\t," << current_pos[1] << "\t," << current_pos[2] << "\t," << current_pos[3] << "\t,"
                   << current_pos[4] << "\t," << current_pos[5] << "\t," << real_current << "\t,";

      

        double theory_current = joint_4_gravity_friction(current_pos[1], current_pos[2], current_pos[3], current_pos[4], current_pos[5], current_vel[3], static_frition);
        std::cout << "理论电流 = " << theory_current << std::endl;
        std::cout << "实际电流 = " << real_current << std::endl;
        double error_current = -real_current + theory_current;
        std::cout << "电流误差 = " << error_current << std::endl;

        if (( current_vel[3])== 0 && static_frition == 0)
            static_frition = -error_current;

        csv_record << theory_current << "\n";

        if (my_abs(error_current) < 0.2)
            error_current = 0;

        extern_force_Pos_offset = one_freedom_admittance(error_current);
  
        std::cout << "位移 = " << extern_force_Pos_offset * 180/3.1415826 << std::endl;

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
    RTDEControlInterface rtde_control("192.168.3.101");

  
    signal(SIGINT, raiseFlag);




    std::vector<double> init_q_2 = {
        -120 * deg2rad,
        -80 * deg2rad,
        -120 * deg2rad,
        80 * deg2rad,
        80 * deg2rad,
        80 * deg2rad,
    };

    // rtde_control.moveJ(init_q_2, 3 * deg2rad, 10 * deg2rad, false);
    rtde_control.moveJ(init_q_2, 30 * deg2rad, 80 * deg2rad, false);
    sleep(2);

    std::thread recored_thread{thread_ur_record_data};
     sleep(3);

    double velocity = 0.5;
    double acceleration = 0.5;
    double servo_dt = 1.0 / 500; // 2ms
    double lookahead_time = 0.1;
    double gain = 300;

    for (double dt = 0; dt <= 60 && flag_loop; dt += 0.002)
    {
        auto t_start = high_resolution_clock::now();

        init_q_2[3] = 80 * deg2rad +  extern_force_Pos_offset;
        rtde_control.servoJ(init_q_2, velocity, acceleration, servo_dt, lookahead_time, gain);

        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);
        if (t_duration.count() < 0.002)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(0.002 - t_duration.count()));
        }
    }

    // Stop the RTDE control script
    rtde_control.stopScript();
    std::cout << "正在退出" << std::endl;
    recored_thread.join();
    csv_record.close();

    return 0;
}
