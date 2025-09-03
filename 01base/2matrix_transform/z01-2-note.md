# 点云矩阵变换

## 1. 做了个“-h”去show help
1. 使用的PCL函数：
   - `pcl::console::find_switch`：pcl中用来检测命令行参数的函数，主要用来判断命令行参数列表中是否包含指定的标志，返回bool值。
   其他函数：
   - 代码中`program_name`可以获取当前程序名称。


## 2. 从命令行参数中提取一个.ply或者.pcd文件名称
1. 使用的PCL函数：
   - `pcl::console::parse_file_extension_argument`：pcl中解析命令行参数，以获取具有给定扩展名向量的文件名，返回包含文件名索引的向量。


## 3. 加载ply/pcd文件
1. 使用的PCL函数：
   - `pcl::io::loadPCDFile`：pcl中根据文件名加载pcd点云的函数。
   - `pcl::io::loadPLYFile`：pcl中根据文件名加载ply点云的函数。


## 4. 使用两种方式操作点云坐标变换
### 4.1 使用Eigen::Matrix4f操作点云坐标变换
Eigen::Matrix4f声明一个齐次变换矩阵，手动计算旋转矩阵和平移向量，对该齐次变换矩阵进行填充，得到想要的齐次变换矩阵。

## 4.2 使用Eigen::Affine3f操作点云坐标变换
Eigen::Affine3f声明一个仿射变换类，指定其平移向量和旋转矩阵，不需要写公式计算，更加直观，得到的结果和Eigen::Matrix4f一样。通过`transform_2.matrix()`可以转换为`transform1`。

## 4.3 执行坐标变换
1. 使用的PCL函数：
   - `pcl::transformPointCloud`：对包含类型为PointXY的点的点云应用仿射变换。

## 5 可视化
-

