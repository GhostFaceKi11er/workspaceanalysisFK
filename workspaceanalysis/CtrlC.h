// CtrlC.h
#pragma once

#include <iostream>
#include <fstream>
#include <octomap/octomap.h>

class CtrlC {
public:
    static int NewvisitPointCount;
    static octomap::OcTree tree;

    // 静态成员函数
    static void saveState(); // 保存状态
    static void restoreState(); // 恢复状态
private:
    static std::string m_treefilePath; // 也需要是静态的
    static std::string m_filename;
};