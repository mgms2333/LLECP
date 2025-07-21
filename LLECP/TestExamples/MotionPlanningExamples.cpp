#include"MotionPlanning.h"
int main()
{
    std::vector<ST_MotionTransitionFrame>v_TransitionMotionFrame;
    
    //添加起始点
    // v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(0,0));
    // //添加过度点
    // v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(200,50));
    // v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(600,20));
    // //添加结束点
    // v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(1000,0));
    // ST_KinematicLimit stKinematicLimit = {100,200,300,-200,-300};

    v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(0,0));
    //添加过度点
    v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(200,300));
    v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(600,200));
    //添加结束点
    v_TransitionMotionFrame.push_back(ST_MotionTransitionFrame(1000,0));

    ST_KinematicLimit stKinematicLimit = {500,1000,3000,-1000,-3000};

    std::vector<ST_MotionFrame>v_Trajectory;
    //需要判断帧速度小于速度约束
    TrajectoryCalculation(0.001,stKinematicLimit,v_TransitionMotionFrame,v_Trajectory);
    std::string filename = "data.csv";
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return -1;
    }

    // 写入表头
    file << "pos,vel,acc,jerk\n";

    // 写入每一行数据
    for (const auto& frame : v_Trajectory)
    {
        file << frame.pos << "," << frame.vel << "," << frame.acc << "," << frame.jerk << "\n";
    }

    file.close();
    std::cout << "保存成功: " << filename << std::endl;

    printf("hello world!");
}