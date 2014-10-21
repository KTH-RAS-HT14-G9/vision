#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/objdetect/objdetect.hpp>


const int _frequency=10;
cv_bridge::CvImagePtr _cv_ptr;
const int _height=240;
const int _width=320;
const int _small=5; //10 , 15
float _average;

namespace enc = sensor_msgs::image_encodings;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    float matrix[_width][_height];
    int mask[_small][_small]={1,4,7,4,1,
                             4,16,26,16,4,
                             7,26,41,26,7,
                             4,16,26,16,4,
                             1,4,7,4,1};

    //ROS_INFO("%d , %d", msg->width,msg->height);
    try
    {
        _cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);

        cv::Mat mat = _cv_ptr->image;


        int startx=(msg->width-_width)/2;
        int endx=startx+_width;
        int starty=(msg->height-_height)/2;
        int endy=starty+_height;

        float min=10;
        int i=startx,j=starty;
        for(int x=startx;x<endx;x++){
            for(int y=starty;y<endy;y++){
                float depth = _cv_ptr->image.at<float>(cv::Point(x,y));
                if(isnan(depth)) depth=10;
                else if(depth>10) depth=10;
                else if(depth<0.001) depth=10;
                else depth=depth;
                matrix[x-startx][y-starty]=depth;

                float aux=std::min(depth,min);
                if(aux < min){
                    min=aux;
                    i=x-startx;
                    j=y-starty;
                }
           }
        }

        ROS_INFO("Min at: %d %d",i,j);

        float sum=0;
        //int N=_small*_small;
        int N=273; // sum of weights of the mask
        for(int x=i-(_small-1)/2;x<i+(_small-1)/2;x++){
            for(int y=j-(_small-1)/2;y<j+(_small-1)/2;y++){
                if(!(x>=0 && x < _width && y >= 0 && y < _height)){
                    //ROS_ERROR("Out bounds: %d %d %d %d",x+startx-1,y+starty-1,x,y);
                    //N--;
                    N=N-mask[x-i+(_small-1)/2][y-j+(_small-1)/2];
                }
                else{

                    if(std::abs(matrix[x][y]) < 10 && std::abs(matrix[x][y])<=1.5*min)
                        //sum=sum+matrix[x][y];
                        sum=sum+mask[x-i+(_small-1)/2][y-j+(_small-1)/2]*matrix[x][y];
                    else{
                        //ROS_ERROR("Bad value:  %d %d %d %d",x+startx-1,y+starty-1,x,y);
                        //N--;
                        N=N-mask[x-i+(_small-1)/2][y-j+(_small-1)/2];
                    }

                }
            }
        }

         _average=sum/N;
         if(_average>2 ||  _average<0.3){
             _average=NAN;
         }
         if(!(_average>=0.8*matrix[i][j] && _average<=1.2*matrix[i][j])){
             _average=NAN;
         }
         //ROS_INFO("Average: %f",_average);
         std::cout << "Average: " << _average << std::endl;

       /* if(N > 0.2*_small*_small){
            _average=sum/N;
            ROS_INFO("new value");
            ROS_INFO("Average: %f",_average);
        }
        else{
            _average=NAN;
            ROS_ERROR("not able to calculate new");
            ROS_INFO("Average: %f",_average);
            }
         */

        //float depth = _cv_ptr->image.at<float>(cv::Point(50,50));

        //ROS_INFO("Depth: %f", depth);

        cv::namedWindow( "Window depth", cv::WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "Window depth", mat );
        cv::waitKey(0);


    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_depth");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub1 = it.subscribe("/camera/depth/image", 1, imageCallback);

    //image_transport::Subscriber sub2 = it.subscribe("/camera/rgb/image_raw", 1, imageRGBCallback);

    ros::Publisher depth_pub=n.advertise<std_msgs::Float64>("/object_distance",1000);

    ros::Rate loop_rate(_frequency);

    while(n.ok()){

        ros::spinOnce();

        std_msgs::Float64 distance;

        distance.data= _average;
        ROS_INFO("Average: %f",distance.data);

        depth_pub.publish(distance);

        loop_rate.sleep();

    }

    return 0;
}




