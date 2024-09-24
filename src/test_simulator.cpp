#include "sensor_simulator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_simulator_node");
    ros::NodeHandle nh;

    SensorSimulator sensor_simulator(nh);

    return 0;
}