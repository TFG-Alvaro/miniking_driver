#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <string>
#include <vector>
#include "MiniKing.h"

int degToGradians(double deg) {
    double grad = 200.0 + (deg * 400.0 / 360.0);
    while (grad < 0.0) grad += 400.0;
    while (grad >= 400.0) grad -= 400.0;
    return static_cast<int>(std::round(grad));
}

double gradiansToRosRad(float sonar_pos_grad) {
    double diff_grad = static_cast<double>(sonar_pos_grad) - 200.0;
    return diff_grad * (M_PI / 200.0);
}

Resolution parseResolution(const std::string& res_str) {
    if (res_str == "Low") return Low;
    if (res_str == "High") return High;
    if (res_str == "Ultimate") return Ultimate;
    return Medium;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "miniking_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string port, ensayo, resolution_str, frame_id, scan_topic;
    int baudrate, gain, ad_low, ad_span, range_scale, bins;
    double duracion, vos, range_min, left_lim_deg, right_lim_deg;
    bool continuous, inverted, verbose;

    pnh.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh.param<int>("baudrate", baudrate, 115200);
    pnh.param<std::string>("ensayo", ensayo, "prueba_miniking");
    pnh.param<double>("duracion", duracion, 0.0);
    pnh.param<int>("gain", gain, 45);
    pnh.param<int>("ad_low", ad_low, 20);
    pnh.param<int>("ad_span", ad_span, 180);
    pnh.param<double>("vos", vos, 1482.0);
    pnh.param<int>("range_scale", range_scale, 2);
    pnh.param<double>("range_min", range_min, 0.05);
    pnh.param<int>("bins", bins, 300);
    pnh.param<std::string>("resolution", resolution_str, "Medium");
    pnh.param<bool>("continuous", continuous, false);
    pnh.param<double>("left_lim_deg", left_lim_deg, -45.0);
    pnh.param<double>("right_lim_deg", right_lim_deg, 45.0);
    pnh.param<bool>("inverted", inverted, false);
    pnh.param<std::string>("frame_id", frame_id, "sonar_link");
    pnh.param<std::string>("scan_topic", scan_topic, "/sonar/scan");
    pnh.param<bool>("verbose", verbose, false);

    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 10);

    ROS_INFO("=== INICIANDO ENSAYO: %s (Duracion: %.1f s) ===", ensayo.c_str(), duracion);
    ROS_INFO("Conectando MiniKing C++ en %s (%d bps)...", port.c_str(), baudrate);

    std::vector<char> port_cstr(port.begin(), port.end());
    port_cstr.push_back('\0');

    MiniKing sonar(port_cstr.data(), baudrate);
    sonar.setVerboseMode(verbose);
    sonar.initSonar();

    sonar.setGain(gain);
    sonar.setADLow(ad_low);
    sonar.setADSpan(ad_span);
    sonar.setRange(range_scale);
    sonar.setBins(bins);
    sonar.setResolution(parseResolution(resolution_str));
    sonar.setContinuous(continuous);
    sonar.setInverted(inverted);

    int left_grad = degToGradians(left_lim_deg);
    int right_grad = degToGradians(right_lim_deg);
    sonar.setLeftLim(left_grad);
    sonar.setRightLim(right_grad);

    sonar.updateConfig();

    double vos_factor = vos / 1500.0;
    double effective_range_max = static_cast<double>(range_scale) * vos_factor;

    ROS_INFO("Configuracion aplicada -> Rango efectivo: %.2fm | Gain: %d%% | Sector: [%.1f deg a %.1f deg]",
             effective_range_max, gain, left_lim_deg, right_lim_deg);

    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        if (duracion > 0.0 && (ros::Time::now() - start_time).toSec() >= duracion) {
            ROS_INFO("Tiempo de ensayo alcanzado (%.1f s). Finalizando nodo.", duracion);
            break;
        }

        BYTE* raw_data = sonar.getScanLine();
        if (raw_data != nullptr) {
            int num_bins = sonar.getDataLength();
            if (num_bins > 0) {
                float pos_grad = sonar.getPosition();
                double angle_rad = gradiansToRosRad(pos_grad);

                sensor_msgs::LaserScan scan_msg;
                scan_msg.header.stamp = ros::Time::now();
                scan_msg.header.frame_id = frame_id;

                scan_msg.range_min = static_cast<float>(range_min);
                scan_msg.range_max = static_cast<float>(effective_range_max);

                scan_msg.angle_min = static_cast<float>(angle_rad);
                scan_msg.angle_max = static_cast<float>(angle_rad);
                scan_msg.angle_increment = 0.0f;

                scan_msg.ranges.resize(num_bins);
                scan_msg.intensities.resize(num_bins);

                double r_step = (effective_range_max - range_min) / static_cast<double>(num_bins);
                for (int i = 0; i < num_bins; ++i) {
                    scan_msg.ranges[i] = static_cast<float>(range_min + i * r_step);
                    scan_msg.intensities[i] = static_cast<float>(raw_data[i]);
                }

                scan_pub.publish(scan_msg);
            }
        }
        ros::spinOnce();
    }

    return 0;
}
