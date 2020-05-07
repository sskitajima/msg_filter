#include "bounding_pointcloud/ScanPublisher.hpp"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ScanPublihser_node");
    if(argc != 4){
        cout << "usage: rosrun ScanPublisher_node [publish_topic_name] [topic_id: 1 or 2] [Hz]";
    }
    const char* topic_name = argv[1];
    const float topic_id = atof(argv[2]);
    const float topic_hz = atof(argv[3]);
    
    cout << "topic name: " << topic_name << " topic_id: " << topic_id << "totpic hz: " << topic_hz << std::endl;
    ScanPublisher scanPub(topic_name, topic_id, topic_hz);
    scanPub.mainloop();
}