#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    //创建点云
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //generate the cloud data
    pcl::io::loadPCDFile("table_scene_lms400.pcd", cloud);

    //create a range image from the above pointcloud
    //角分辨率为弧度制1度
    float angularResolution = (float)(1.0f * (M_PI/180.0f));
    //水平方向最大采样角为弧度制360度
    float maxAngleWidth = (float)(360.0f * (M_PI/180.0f));
    //垂直方向最大采样角为弧度制180度
    float maxAngleHeight = (float)(180.0f * (M_PI/180.0f));
    //噪声程度，即周围点对深度值的影响程度
    float noiselevel = 0.00;
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)(Eigen::Translation3f(0.0f, 0.0f, 0.0f));
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float minRange = 0.0f;
    int borderSize = 1;

    //create range image
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight, 
                                    sensorPose, coordinate_frame, noiselevel, minRange, borderSize);
    
    cout << rangeImage << endl;


    return 0;
}