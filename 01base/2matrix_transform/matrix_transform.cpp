#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h> // 点云坐标变换
#include <pcl/visualization/pcl_visualizer.h>


/* show help 函数*/
void showHelp(char *program_name)
{
    std::cout << std::endl;     // 空行
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl; // 使用规范
    std::cout << "-h:  Show this help." << std::endl;                                   // -h/--help
}


// This is the main function
int main(int argc, char **argv)
{
    /* 1. 检测命令行中是否有'-h'或'--help'参数，以触发showHelp函数 */
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    /* 2. 从命令行参数中提取一个 .ply 或 .pcd 文件名。*/
    std::vector<int> filenames;     // 用来存储文件名参数
    bool file_is_pcd = false;       // 默认提取ply文件，如果为pcd文件需要标志位标识
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");        // 获取argv中所有ply文件
    if (filenames.size() != 1)          // ply文件需要一个，不是一个就去找pcd文件
    {
        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");    // 获取argv中所有pcd文件

        if (filenames.size() != 1)      // pcd文件需要一个，不是一个就报错
        {
            showHelp(argv[0]);          // argv[0]为可执行程序名称
            return -1;                  // 程序退出
        }
        else
        {
            file_is_pcd = true;         // 若确定为pcd文件，则pcd文件标志位标识
        }
    }

    /* 3. 加载ply/pcd文件 */ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (file_is_pcd)        // 加载pcd文件
    {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0) 
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }
    else                    // 加载ply文件
    {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }

    /* 4. 使用两种方法操作点云坐标变换
    Reminder: how transformation matrices work :
           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f ====================================第一种方法
        This is the "manual" method, perfect to understand but error prone !
    METHOD #2: Using a Affine3f ====================================第二种方法
        This method is easier and less error prone！
    */

    /* 4.1 通过4*4齐次变换矩阵操作点云坐标变换 */
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();      // 先初始化为单位矩阵
    // 定义旋转矩阵
    float theta = M_PI / 4; // The angle of rotation in radians
    transform_1(0, 0) = std::cos(theta);    // 根据旋转角度，填充旋转矩阵(0, 0)
    transform_1(0, 1) = -sin(theta);        // 根据旋转角度，填充旋转矩阵(0, 1)
    transform_1(1, 0) = sin(theta);         // 根据旋转角度，填充旋转矩阵(1, 0)
    transform_1(1, 1) = std::cos(theta);    // 根据旋转角度，填充旋转矩阵(1, 1)
    // 定义平移向量
    transform_1(0, 3) = 2.5;                // 根据x平移距离，填充平移向量(0, 3)
    // 打印齐次变换矩阵
    printf("Method #1: using a Matrix4f\n");    
    std::cout << transform_1 << std::endl;      // 打印齐次变换矩阵

    /* 4.2 通过Eigen::Affine3f操作点云坐标变换  */ 
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 2.5, 0.0, 0.0;                                 // 定义平移向量
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));     // 定义绕Z轴的旋转
    // 打印齐次变换矩阵
    printf("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;     // 打印变换矩阵。transform_1=transform_2.matrix()

    /* 4.3 执行坐标变换 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);  // Transform_1 和 transform_2 一样

    /* 5. 点云可视化 */
    printf("\nPoint cloud colors :  white  = original point cloud\n"
           "                        red  = transformed point cloud\n");
    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped())
    { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce();
    }

    return 0;
}