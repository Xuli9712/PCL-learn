//从深度图提取边界
#include <iostream>
#include <boost/thread/thread.hpp>

#include <pcl/range_image/range_image.h>  //深度图类
#include <pcl/io/pcd_io.h> //读写pcd
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h> //特征/提取深度图边界
#include <pcl/console/parse.h> //控制台输入参数解析

using namespace std;

//-----------------------------
//--------Parameters-----------
//-----------------------------
float angular_resolution = 0.5f;  //角分辨率
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;  //坐标系
bool setUnseenToMAxRange = false;

//-----------------------------
//--------Usages---------------
//-----------------------------
void printUsage(const char* progName)
{
    cout << "\n\nUsage:" << *progName << "[options] <scene.pcd>\n\n"
         << "Options:\n"
         << "------------------------------------------------\n"
         << "-a <float>  angular resolution in degrees"

}

//-------------------------------
//--------main-------------------
//-------------------------------
int main(int argc, char** argv)
{
    //----------------------------------------------------------------------------
    //---------parse the command line arguments of help and 3 parameters----------
    //----------------------------------------------------------------------------
    if (pcl::console::find_argument(argc, argv, "-h")>=0)
    {
        printUsage(argv[0]);
        return 0;
    }
    if (pcl::console::find_argument(argc, argv, '-a', angular_resolution)>=0)
    {
        cout << "setting angular resolution to: " << angular_resolution << "degree." << endl;
        //输入角度制，转成弧度制
        angular_resolution = pcl::deg2rad(angular_resolution);
    }
    int temp_coordinate_frame;
    if (pcl::console::find_argument(argc, argv, "-c", temp_coordinate_frame)>=0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(temp_coordinate_frame);
        cout << "Using coordinate frame: " << (int)coordinate_frame <<"." << endl;
    }
    if (pcl::console::find_argument(argc, argv, "-m")>=0)
    {
        setUnseenToMAxRange = true;
        cout << "Setting unseen values in range image to max range readings." << endl;
    }

    //-------------------------------------------------
    //--------------read the pcd file------------------
    //-------------------------------------------------



}



