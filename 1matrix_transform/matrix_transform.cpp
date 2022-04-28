#include <iostream>
#include <pcl/io/pcd_io.h>  //读写pcd文件
#include <pcl/io/ply_io.h>  //读写ply文件
#include <pcl/console/parse.h> //命令行控制台解析
#include <pcl/common/transforms.h> //矩阵坐标变换
#include <pcl/visualization/pcl_visualizer.h> //点云可视化
#include <pcl/point_cloud.h> //点云头文件

using namespace std;

//输入命令行程序名，显示命令行输入格式的帮助 -h和--help, 确保输入文件为pcd或者ply
void showHelp(char *program_name)
{
    cout << endl;
    cout << "Usage:" << program_name << "cloud_filename.[ply|pcd]" << endl;
    cout << "-h: Show this help" << endl;
}

int main(int argc, char** argv)
{
    //如果输入-h，则调用showHelp函数，函数的输入参数是命令行的第一个内容，也就是程序名，返回帮助信息，即打印出正确的命令格式
    if (pcl::console::find_switch(argc, argv, "-h")||pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    //匹配读取的是pcd还是ply文件
    vector<int> filenames;
    bool file_is_pcd = false;

    


}

