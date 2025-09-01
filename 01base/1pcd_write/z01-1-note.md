# pcd文件保存

## 1. 制造一份点云数据
- 声明`pcl::PointCloud<pcl::PointXYZ>`的模板类实例cloud，其类型参数为`pcl::PointXYZ`。
- 填充cloud必要的信息

## 2. 将点云保存为pcd文件（ASCII）
使用了`pcl::io::savePCDFileASCII()`

## 3. 遍历点云，打印每个点XYZ
-