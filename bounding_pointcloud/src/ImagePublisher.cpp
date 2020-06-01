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
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}




class imagePublisher
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_img;

    const char* pub_image_dir = "/home/kitajima/workspace/save_image/";
    const char* image_name = "keyframe";
    
    int num_max = 39;
    int num_min = 0;
    int count=0;

public:
    imagePublisher()
    : it_(nh_)
    {
        pub_img = it_.advertise("/keyframe", 10);
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
        count++;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "/openni_rgb_optical_frame";
        header.seq = count;

        return header;
    }

    void test()
    {
        while (1)
        {
            cout << "入力: number  終了: -1" << endl;
            int input;
            cin >> input;

            cout << "input ; " << input << endl;

            if (input == -1)
            {
                cout << "quit" << endl;
                break;
            }
            else if (num_min <= input && input <= num_max)
            {
                cout << "valid input" << endl;
                const std::string save_path = (boost::format("%s%s_%d.png") % pub_image_dir % image_name % input).str();

                cout << save_path << endl;

                cv::Mat img = cv::imread(save_path);
                cout << "imread" << endl;
                cv_bridge::CvImage cv_image;
                cv_image.header = make_header();
                cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
                cv_image.image = img;

                std::cout << "  dims: " << img.dims << ", depth(byte/channel):" << img.elemSize1()
                          << ", channels: " << img.channels() << std::endl;

                pub_img.publish(cv_image.toImageMsg());
            }
            else
            {
                cout << "invalid input" << endl;
                break;
            }

            // if(input == 0) cout << "zero" <<endl;
            // else if(input == 1) cout << "one" << endl;
            // else break;

            cout << endl;

            // if (kbhit())
            // {
            //     const char key = getchar();
            //     printf("'%c'を押しました。\n", key);
                
            //     // int key_int = ctoi(key);
            //     // cout << Cnum_min << " " << Cnum_max << endl; 
            //     if(num_min <= key && key <= num_max){
            //         const std::string save_path = (boost::format("%s%s_%s.png") % pub_image_dir % image_name % key).str();
                    
            //         cout << save_path << endl;

            //         cv::Mat img = cv::imread(save_path);
            //         cout << "imread" << endl;
            //         cv_bridge::CvImage cv_image;
            //         cv_image.header = make_header();                            
            //         cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC3;
            //         cv_image.image = img;    

            //           std::cout << "  dims: " << img.dims << ", depth(byte/channel):" << img.elemSize1() \
	        //                 << ", channels: " << img.channels() << std::endl;

            //         // pub_img.publish(cv_ptr->toImageMsg());                       
            //     }
            //     else if(key == *"q")
            //       break;
            //     else
            //     {
            //         ROS_INFO("invalid number");
            //         // break;
            //     } 

            // }
        }

    }
};
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageSaver_node");
    bounding_pointcloud::imagePublisher imagePublisher;
    imagePublisher.test();
    ros::spin();
}
