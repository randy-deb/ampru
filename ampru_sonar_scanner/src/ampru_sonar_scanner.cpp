
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ampru_sonar_scanner");
    ros::NodeHandle nh;

    tf::TransformBroadcaster tf_broadcaster;
    ros::Publisher scan_publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 50);

    /*
     * HY-SRF05 Specs:
     *  Angle: 15degrees (=0.26rad)
     *  Distance: 2-450cm
     *
     * using 5 sensors, we have an 75 degree scan angle
     */
    uint32_t num_readings = 5;
    double laser_frequency = 40;

    int count = 0;
    ros::Rate loopRate(50.0);    
    while (nh.ok())
    {
        ros::Time scan_time = ros::Time::now();

        // Update the laser transorm
        tf_broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)), 
                scan_time, 
                "base_link", 
                "base_laser"
            )
        );

        // Populate the message
        sensor_msgs::LaserScan scanData;
        scanData.header.stamp = scan_time;
        scanData.header.frame_id = "base_laser";
        scanData.angle_min = -0.56;
        scanData.angle_max = 0.56;
        scanData.angle_increment = 0.10 / num_readings;
        scanData.time_increment = (1 / laser_frequency) / num_readings;
        scanData.range_min = 0.02; // meters
        scanData.range_max = 4.50; // meters
        scanData.ranges.resize(num_readings);
        scanData.intensities.resize(num_readings);
        for (uint32_t i = 0; i < num_readings; i++)
        {
            scanData.ranges[i] = 2.50; //TODO: We need this information from the arduino sensors
        }
        scan_publisher.publish(scanData);
        count++;

        loopRate.sleep();
    }
}