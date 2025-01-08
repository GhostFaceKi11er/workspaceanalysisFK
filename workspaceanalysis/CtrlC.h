// CtrlC.h
#pragma once

#include <iostream>
#include <fstream>
#include <octomap/octomap.h>

class CtrlC {
public:
    static int NewvisitPointCount;
    static octomap::OcTree tree;
    // 构造函数

    // 静态成员函数
    static void saveState(); // 保存状态
    static void restoreState(); // 恢复状态

};