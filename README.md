# ws_m1 & ws_m1_load
对m1进行空载和有载的工作区间分析

输入: urdf
输出： octomap::Octree的.bt文件, Newvisitpointcount的.txt文件，场景预览的osgViewer图片

# 基本信息
通过随机采样关节角然后通过FK得到末端执行器位置，如果力矩不超限则添加到octomap::OcTree中。如果FK时新访问点的比例低于processRatio则结束FK。

# 代码框架

用到的文件: 
CtrlC.h, CtrlC.cpp.   #处理ctrl+c中断

ws_m1.cpp和ws_m1_load.cpp的唯一不同就是urdf文件路径，保存OcTree的M1_full.bt文件路径，保存有效点个数的state.txt文件路径。
# 功能
整体思路： M1机器人一共有18个活动的关节，躯干4个，左臂和右臂各7个，均为revolute关节。

ProgramProcessMode_Process模式下的FK： 每次循环对躯干4个和右臂上的7个进行随机采样得到一组关节角配置，将该组关节角配置设定到M1中，得到末端执行器的位置；然后在octomap::OcTree中去搜索之前是否已经将该点更新到OcTree中并且设置为占据状态；如果已经有则跳过，如果之前没有则判断力矩是否超限，如果没有超限添加到OcTree中。循环的结束条件： 经过timeWithoutUpdate分钟的间隔没有新的点更新到OcTree中。

ProgramProcessMode_Visualize模式下的可视化：对指定的空间（3维空间或2维平面）内的点进行离散化，分辨率大小为resolution和OcTree保持一致。然后对该指定空间内的点进行遍历，判断能否在OcTree中搜索到该点，如果有则将该点添加到pointcloud中，颜色设置为蓝色；对边界上的点也要离散化并添加到点云中，颜色设置为红色。
（另一种思路：对OcTree中的叶子节点进行遍历然后添加到pointcloud中，但会出现叶子节点被母节点代表的情况（和OcTree八叉数的结构有关），在可视化时就无法看到所有的有效点）


运行可执行文件后会生成两个文件，一个为存储OcTree的treefilePath文件，另一个为保存有效点个数的filename文件。

每遍历1000个空间中的点会自动保存数据到上述两个文件。同时也提供了手动保存：按下Ctrl + c可以手动中断程序并保存数据到上述两个文件。

运行结束的时候会弹出osgViewer场景预览的图片

# 注意事项
1. CMakeLists.txt中可能需要手动指定某些库文件或头文件的路径，否则会报错。
2. 每次在ProgramProcessMode_Process下运行前注意检查是否设定好保存有效点个数的state.txt文件，存储OcTree的M1_full.bt的文件。

# 示例




# twoTCPIK

# 基本信息

因为DART中的多TCP逆运动学问题求解的范围有限，所以尝试自己实现通过伪逆法对2TCP逆运动学问题求解。

目前进度：

两只手不对称的情况下：误差会振荡，找不到逆解。

两只手对称的情况下：

    同时考虑位置和姿态： 在 两只手在初始位置下保持y不变，x,z改变同样的值，逆运动学能求解。（改变y值误差会振荡，无法求解）

    只考虑位置，手动忽略姿态误差： 在 两只手x,z改变相同的值，y改变相反的值 的情况下，逆运动学能求解。
