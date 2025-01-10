#include "CtrlC.h"
#include <filesystem> 
#include <iostream>

int CtrlC::NewvisitPointCount = 0;
octomap::OcTree CtrlC::tree(0.05);
std::string CtrlC::m_treefilePath = "/home/haitaoxu/workspaceanalysis/M1_full_load_10.bt";
std::string CtrlC::m_filename = "/home/haitaoxu/workspaceanalysis/state.txt";

//静态保存函数 保存Octree到treefilepath, NewvisitPointCount到filepath
void CtrlC::saveState() {
    tree.writeBinary(CtrlC::m_treefilePath);

    std::ofstream ofs(CtrlC::m_filename, std::ios::out | std::ios::trunc);
    if (ofs.is_open()) {
        ofs << "NewvisitPointCount=" << CtrlC::NewvisitPointCount << std::endl;
        std::cout << "NewvisitPointCount=" << CtrlC::NewvisitPointCount << std::endl;
        //ofs << "current_time_output=" << current_time_output << std::endl;
        std::cout << "State saved.\n";
    } else {
        std::cerr << "Failed to open file for saving state.\n";
    }

}

// 静态恢复状态函数
void CtrlC::restoreState() {
    if (std::filesystem::exists(CtrlC::m_treefilePath)){ // 如果树文件存在，则读取树文件
        tree.readBinary(CtrlC::m_treefilePath);
    }
    else{
        std::cout << "treefilePath not exists" << std::endl;
    }

    std::ifstream ifs(CtrlC::m_filename);
    if (ifs.is_open()) {
        std::string line;
        while (std::getline(ifs, line)) {
            if (line.find("NewvisitPointCount=") != std::string::npos) {
                CtrlC::NewvisitPointCount = std::stoi(line.substr(line.find('=') + 1));
            } 
        }
        std::cout << "Restored state. NewvisitPointCount: " << CtrlC::NewvisitPointCount << std::endl;
    } else {
        std::cout << "No previous state found. Starting fresh.\n";
        CtrlC::NewvisitPointCount = 0;
    }
}
