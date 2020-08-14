// あるディレクトリの中にあるファイルを、キーボード入力をイベントとしてpuclishするプログラム
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <boost/format.hpp>

using namespace std;

namespace bounding_pointcloud
{

// キー入力の受けつけ
// https://hotnews8.net/programming/tricky-code/c-code03
// int kbhit(void)
// {
//     struct termios oldt, newt;
//     int ch;
//     int oldf;

//     tcgetattr(STDIN_FILENO, &oldt);
//     newt = oldt;
//     newt.c_lflag &= ~(ICANON | ECHO);
//     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//     oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
//     fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

//     ch = getchar();

//     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
//     fcntl(STDIN_FILENO, F_SETFL, oldf);

//     if (ch != EOF) {
//         ungetc(ch, stdin);
//         return 1;
//     }

//     return 0;
// }




class imagePublisher
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_img_rgb, pub_img_depth;

    const char* pub_image_dir = "/home/kitajima/workspace/Data/save_image/";
    const char* pub_depth_dir = "/home/kitajima/workspace/Data/save_image_depth/";
    const char* image_name = "keyframe";
    const float hz_rate = 1.0;
    
    int num_max = 391;
    int num_min = 0;
    int count=0;
    int count_seq=0;

public:
    imagePublisher()
    : it_(nh_)
    {
        pub_img_rgb = it_.advertise("/camera/rgb/image_color", 10);
        pub_img_depth = it_.advertise("/camera/depth/image", 10);
    }

    ~imagePublisher()
    {
    }

    // char型の数字をint型の数字にする
    // minからmaxに収まらない場合、-1を返す
    // int ctoi(const char c)
    // {
    //     if (*Cnum_min <= c && c <= *Cnum_max)
    //         return (c - '0');
    //     return -1;
    // }

    std_msgs::Header make_header()
    {
        count_seq++;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "/openni_rgb_optical_frame";
        header.seq = count_seq;

        return header;
    }

    void publishLoop()
    {
        ros::Rate loop_rate(hz_rate);
        while (ros::ok())
        {
            if (count==num_max) count=num_min;

            const std::string save_path_rgb = (boost::format("%s%s_%03d.png") % pub_image_dir % image_name % count).str();
            const std::string save_path_depth = (boost::format("%s%s_%03d.png") % pub_depth_dir % image_name % count).str();
            
            // const std::string save_path_rgb = (boost::format("%s%03d.png") % pub_image_dir % count).str();
            // const std::string save_path_depth = (boost::format("%s%03d.png") % pub_depth_dir  % count).str();

            cv::Mat img_rgb = cv::imread(save_path_rgb,  cv::IMREAD_UNCHANGED);
            // cv::Mat img_depth = cv::imread(save_path_depth, cv::IMREAD_GRAYSCALE);
            cv::Mat img_depth = cv::imread(save_path_depth,  cv::IMREAD_UNCHANGED);
            cv::Mat img_32FC1;
            img_depth.convertTo(img_32FC1, CV_32FC1, 1/5000.0);


            // cout << "img_depth: " << img_depth << endl; 

            cv_bridge::CvImage cv_image_rgb, cv_image_depth;
            std_msgs::Header header = make_header();

            cv_image_rgb.header = header;
            cv_image_rgb.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image_rgb.image = img_rgb;

            cv_image_depth.header = header;
            cv_image_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            // cv_image_depth.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image_depth.image = img_32FC1;
            

            // std::cout << "  dims: " << img_depth.dims << ", depth(byte/channel):" << img_depth.elemSize1()
            //           << ", channels: " << img_depth.channels() << std::endl;

            pub_img_rgb.publish(cv_image_rgb.toImageMsg());
            pub_img_depth.publish(cv_image_depth.toImageMsg());

            count++;
            cout << "publish: " << save_path_rgb << endl;
            cout << "publish: " << save_path_depth << endl;
            cout << endl;
            // cout << "count: " << count << endl;

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageSaver_node");
    bounding_pointcloud::imagePublisher imagePublisher;
    imagePublisher.publishLoop();
}
