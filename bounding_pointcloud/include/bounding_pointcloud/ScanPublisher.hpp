#ifndef SCAN_PUBLISHER_H
#define SCAN_PUBLISHER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Header.h"

#include <string>
#include <random>

uint64_t get_rand_range(uint64_t min, uint64_t max)
{
    static std::mt19937_64 mt64(0);
    std::uniform_int_distribution<uint64_t> get_rand_uni_int(min, max);

    return get_rand_uni_int(mt64);
}

// message_filterの効果を実験するために、LaserScan型のトピックを異なる周期で発信するノード
class ScanPublisher
{
private:
    ros::NodeHandle nh;
    ros::Publisher scan_pub;

    uint32_t scan_count=0;


    const char* scan_topic_name;
    const int scan_topic_id;
    const float scan_topic_hz;

public:
    ScanPublisher(const char* topic_name, int topic_id, float topic_hz): scan_topic_name(topic_name), scan_topic_id(topic_id), scan_topic_hz(topic_hz)
    {
        scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
    }

    ~ScanPublisher()
    {
    }

    std_msgs::Header create_header()
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        scan_count++;

        switch(scan_topic_id){
            case 1:
                std::cout << "/scan1" << std::endl;
                header.frame_id = "/scan1";
                header.seq = scan_count;
                break;
            case 2:
                std::cout << "/scan2" << std::endl;
                header.frame_id = "/scan2";
                header.seq = scan_count;
                break;
            default:
                std::cout << "nor 1 or 2." << std::endl;
                header.frame_id = "unknown";
                header.seq = 0;
                break;
        }

        return header;
    }

    // float* create_ranges(int scan_num)
    // {
    //     float scan[scan_num];
    //     for(int i=0; i<scan_num; i++)
    //     {
    //         scan[i] = get_rand_range(0, 4.0);
    //     }

    //     return scan;
    // }

    sensor_msgs::LaserScan* create_scan()
    {
        sensor_msgs::LaserScan* scan = new sensor_msgs::LaserScan();
        scan->header = create_header();
        scan->angle_min = -1;
        scan->angle_max = 1;
        scan->angle_increment = 0.1;
        scan->scan_time = int(1.0/scan_topic_hz);
        scan->range_min = 0.0;
        scan->range_max = 4.0;

        int scan_num = int((scan->angle_max - scan->angle_min)/scan->angle_increment) + 1;
        scan->ranges.resize(scan_num);
        for(int i=0; i<scan_num; i++)
        {
            scan->ranges[i] = get_rand_range(0, 4.0);
        }

        
        return scan;
    }

    void mainloop()
    {
        ros::Rate loop_rate(scan_topic_hz);

        while(ros::ok())
        {
            std::cout << "loop start" << std::endl;
            std::cout << "start scan1" << std::endl;
            sensor_msgs::LaserScan* scan = create_scan();
            scan_pub.publish(*scan);

            loop_rate.sleep();
        }
    }
};

#endif