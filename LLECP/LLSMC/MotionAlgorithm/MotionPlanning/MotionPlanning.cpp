#include"MotionPlanning.h"

MotionPlanning::MotionPlanning(/* args */)
{
}

MotionPlanning::~MotionPlanning()
{
}

int MotionPlanning::TrajectoryCalculation(double dCycle,ST_KinematicLimit stKinematicLimit,std::vector<ST_MotionTransitionFrame>& v_TransitionMotionFrame,std::vector<ST_MotionFrame>& v_Trajectory)
{
    //计算每段位移的长度
    v_double v_Segmentation_S;
    v_double v_time;
    for(uint8_t i =  0;i<v_TransitionMotionFrame.size() - 1 ;i++)
    {
        v_Segmentation_S.push_back(v_TransitionMotionFrame[i+1].pos - v_TransitionMotionFrame[i].pos);
    }
    //速度段
    v_double v_Segmentation_V;
    for(uint8_t i =  0;i<v_TransitionMotionFrame.size();i++)
    {
        v_Segmentation_V.push_back(v_TransitionMotionFrame[i].vel);
    }
    v_double v_Vmax;
    v_double v_Vel;
    SectionalSpeedVerification(v_Segmentation_S,v_Segmentation_V,stKinematicLimit,v_Vmax,v_Vel);
    uint32_t j = 0;
    for(uint32_t i = 0;i<v_Vel.size()-1;i++)
    {
        double v0 = v_Vel[i];        
        double v1 = v_Vel[i+1];
        //对异常进行强行校正
        // if(v0>v_Vmax[i])
        // {
        //     v0 = v_Vmax[i];
        // }
        v_double para ;
        AsymmetricSCurvesFixedLengthParametersCorrection(v0,v1,v_Vmax[i],stKinematicLimit,v_Segmentation_S[i],para);
        double T = para[0] + para[1] + para[2] + para[3] + para[4] + para[5] + para[6];        //每段曲线运动总时间
        for(double t = 0; t < T; t = t + dCycle)
        {
            v_time.push_back(dCycle * j);
            ST_MotionFrame stMotionFrame;
            AsymmetricSCurveTimeScaling(t,para,stMotionFrame);
            v_Trajectory.push_back(stMotionFrame);
            j++;
        }
        v_int pos_0;
        if (i >= 1)
        {
            //每一段S型曲线的时间t初值为0，所以两段S曲线衔接处会出现位移st为0 的元素
            //需要将st中为0的元素去除,将vel，acc,Jerk中的对应元素删掉，同时将时间time减去一个周期值
            for (size_t k = 0; k < v_Trajectory.size(); ++k) 
            {
                if (v_Trajectory[k].pos == 0) 
                {
                    pos_0.push_back(k);  // 注意：C++ 下标从 0 开始
                }
            }
            pos_0.erase(pos_0.begin());
            v_time.erase(v_time.end());
            //理论上pos_0的size为1，两端轨迹拼接删除第二段轨迹第一个点
            if(pos_0.size()!=1)
            {
                printf("Situations beyond calculation\n");
                printf("Situations beyond calculation\n");
                printf("Situations beyond calculation\n");
                printf("Situations beyond calculation\n");
            }
            v_Trajectory.erase(v_Trajectory.begin()+ pos_0[0]);
            j = j - 1;
        }
    }
    //由于是分段进行s型速度规划，所以要对每一段的位移q进行累加,速度，加速度，加加速度不需要
    v_int pos_1;pos_1.push_back(0);

    for(int32_t i = 0;i<v_Trajectory.size() -1;i++)
    {
        if (v_Trajectory[i].pos > v_Trajectory[i+1].pos)
        {
            pos_1.push_back(i);
        }
    }
    for(int32_t i = 1;i<pos_1.size()-1;i++)
    {
        for(int32_t j = pos_1[i]+1;j<=pos_1[i+1];j++)
        {
            v_Trajectory[j].pos += v_Trajectory[pos_1[i]].pos;
        }
    }
    for(int32_t i = pos_1.back()+1;i<v_Trajectory.size();i++)
    {
        v_Trajectory[i].pos += v_Trajectory[pos_1.back()].pos;
    }
    int s = v_Trajectory.size();
    //最后一个元素加
    v_Trajectory[v_Trajectory.size()].pos += v_Trajectory[pos_1.back()].pos;
    return 0;
}

int MotionPlanning::SectionalSpeedVerification(v_double v_Segmentation_S,v_double v_Segmentation_V,ST_KinematicLimit stKinematicLimit,v_double& v_Vmax,v_double& v_Vel)
{
    v_Vmax.clear();
    //记录每一段可达的最大速度
    for(uint8_t i = 0;i<v_Segmentation_S.size();i++)
    {
        v_Vmax.push_back(stKinematicLimit.Vel);
    }
    //从最后一段曲线开始反向计算
    for(uint8_t i = v_Segmentation_V.size() - 1;i!=1;i--)
    {
        double V_s = v_Segmentation_V[i - 1];
        double V_e = v_Segmentation_V[i];
        double S_seg = v_Segmentation_S[i-1];
        double V_max = v_Vmax[i - 1];
        //判断给定S_seg条件下速度能否从V_s减速到V_e 
        if (V_s > V_e )
        {
            stKinematicLimit.A_dec_max = abs(stKinematicLimit.A_dec_max);
            stKinematicLimit.J_dec_max = abs(stKinematicLimit.J_dec_max);
            if((V_s - V_e)<= pow(stKinematicLimit.A_dec_max, 2)/stKinematicLimit.J_dec_max)//加减速 --> 减减速阶段  
            {
                double S_dec_min = (V_s + V_e) * sqrt((V_s - V_e) / stKinematicLimit.J_dec_max);
                if (S_seg < S_dec_min)
                {
                    //调整V_s，V_max
                    double y1 = 4 * pow(V_e,3) + 1.5 * (sqrt(81 * pow(stKinematicLimit.J_dec_max,2) * pow(S_seg,4) + 96 * pow(V_e,3) * 
                        abs(stKinematicLimit.J_dec_max) * pow(S_seg,2)) - 8 * pow(V_e,3) - 9 * abs(stKinematicLimit.J_dec_max) * pow(S_seg,2));

                    double y2 = 4 * pow(V_e,3) - 1.5 * (sqrt(81 * pow(stKinematicLimit.J_dec_max,2) * pow(S_seg,4)  + 96 * pow(V_e,3) *
                        abs(stKinematicLimit.J_dec_max) * pow(S_seg,2)) + 8 * pow(V_e,3) + 9 * abs(stKinematicLimit.J_dec_max) * pow(S_seg,2));
                    //V_s_dot为修正后的起点速度
                    double V_s_dot = ( -V_e - (Prescription(y1, 3) + Prescription(y2, 3))) / 3;
                    v_Vmax[i - 1] = V_s_dot;
                    V_s = V_s_dot;
                    v_Segmentation_V[i - 1] = V_s;         //更新曲线段初速度
                }
            }
            else // 加减速 --> 匀减速 --> 减减速，论文中遗漏了该情况<全类型非对称七段式S型曲线加减速控制算法研究>
            {
                double T5 = stKinematicLimit.A_dec_max / stKinematicLimit.J_dec_max;
                double T6 = (V_s - V_e) / stKinematicLimit.A_dec_max - T5;
                double T7 = T5;
                //从V_s 加减速 --> 匀减速 --> 减减速 到V_e所需的最短距离         
                double S_dec_min = V_s * (2 * T5 + T6) - 0.5 * stKinematicLimit.J_dec_max * pow(T5, 2) * (T5 + T6) - 0.5 * stKinematicLimit.J_dec_max * T5 * pow((T5 + T6),2);
                if (S_seg < S_dec_min)
                {
                    //调整V_e，V_max,草草草草草草草草草草草草草草草草草草草草草草草草草草草草草草草草草草
                    //《高速加工数控系统NURBS曲线前瞻直接插补关键算法研究与实现》3.3.2 修正起点速度
                    //按存在匀减速段，计算起点速度
                    double a = 1 / stKinematicLimit.A_dec_max;
                    double b = stKinematicLimit.A_dec_max / stKinematicLimit.J_dec_max;
                    double c = stKinematicLimit.A_dec_max * V_e / stKinematicLimit.J_dec_max - pow(V_e,2) / stKinematicLimit.A_dec_max - S_seg;
                    double V_s_dot_1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                    double V_s_dot_2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
                    //按不存在匀减速段，计算起点速度
                    double y1 = 4 * pow(V_e,3) + 1.5 * (sqrt(81 * pow(stKinematicLimit.J_dec_max,2) * pow(S_seg,4) + 96 * pow(V_e,3) * abs(stKinematicLimit.J_dec_max) * pow(S_seg,2)) - 8 * pow(V_e,3) - 9 * abs(stKinematicLimit.J_dec_max) * pow(S_seg,2));
                    double y2 = 4 * pow(V_e,3) - 1.5 * (sqrt(81 * pow(stKinematicLimit.J_dec_max,2) * pow(S_seg,4) + 96 * pow(V_e,3) * abs(stKinematicLimit.J_dec_max) * pow(S_seg,2)) + 8 * pow(V_e,3) + 9 * abs(stKinematicLimit.J_dec_max) * pow(S_seg,2));
                    double V_s_dot_3 = ( -V_e - (Prescription(y1, 3) + Prescription(y2, 3))) / 3;
                    double V_s_tmp[3] = {V_s_dot_1, V_s_dot_2, V_s_dot_3};

                    //取结果中的最大值作为修正后的起点速度
                    double V_s = *std::max_element(V_s_tmp, V_s_tmp + 3);
                    v_Vmax[i - 1] = V_s;
                    v_Segmentation_V[i - 1] = V_s;
                }
            }
        }
    }
    //从第一段曲线的起始速度开始正向计算
    for(uint8_t i = 0;i<v_Segmentation_V.size() - 1;i++)
    {
        double V_s = v_Segmentation_V[i];
        double V_e = v_Segmentation_V[i+1];
        double S_seg = v_Segmentation_S[i];
        if(V_s < V_e)//判断给定S_seg条件下速度能否从V_s加速到V_e 
        {
            if((V_e - V_s) <= pow(stKinematicLimit.A_acc_max, 2) / stKinematicLimit.J_acc_max)//加加速 --> 减加速阶段   
            {
                //从V_s加加速 --> 减加速 到V_e所需的最短距离
                double S_acc_min = (V_s + V_e) * sqrt((V_e - V_s) / stKinematicLimit.J_acc_max);
                if(S_seg < S_acc_min)
                {
                    //调整V_e，V_max
                    double x1 = 4 * pow(V_s,3) + 1.5 * (sqrt(81 * pow(stKinematicLimit.J_acc_max,2) * pow(S_seg,4) + 96 * pow(V_s,3) * stKinematicLimit.J_acc_max * pow(S_seg,2)) - 8 * pow(V_s,3) - 9 * stKinematicLimit.J_acc_max * pow(S_seg,2));
                    double x2 = 4 * pow(V_s,3) - 1.5 * (sqrt(81 * pow(stKinematicLimit.J_acc_max,2) * pow(S_seg,4) + 96 * pow(V_s,3) *stKinematicLimit.J_acc_max * pow(S_seg,2)) + 8 * pow(V_s,3) + 9 * stKinematicLimit.J_acc_max * pow(S_seg,2));
                    //V_e_dot为实际可达终点速度
                    double V_e_dot = ( -V_s - (Prescription(x1, 3) + Prescription(x2, 3))) / 3;
                    v_Vmax[i] = V_e_dot;
                    V_e = V_e_dot;
                    v_Segmentation_V[i + 1] = V_e;         //更新曲线段末速度
                }
            }
            //2025.7.20 4：20  老子脑袋要炸了
            else  //加加速 --> 匀加速 --> 减加速，论文中遗漏了该情况
            {
                double T1 = stKinematicLimit.A_acc_max / stKinematicLimit.J_acc_max;
                double T3 = T1;
                double T2 = (V_e - V_s) / stKinematicLimit.A_acc_max - T1;
                //从V_s 加加速 --> 匀加速 --> 减加速 到V_e所需的最短距离        
                double S_acc_min = V_s * (2 * T1 + T2) + 0.5 * stKinematicLimit.J_acc_max * pow(T1, 2) * (T1 + T2) + 0.5 * stKinematicLimit.J_acc_max * T1 * pow(T1 + T2, 2);
                if(S_seg < S_acc_min)
                {
                    //《高速加工数控系统NURBS曲线前瞻直接插补关键算法研究与实现》3.3.2  修正终点速度
                    //按存在匀加速段修正终点速度
                    double a = 1 / stKinematicLimit.A_acc_max;
                    double b = stKinematicLimit.A_acc_max / stKinematicLimit.J_acc_max;
                    double c = stKinematicLimit.A_acc_max * V_s / stKinematicLimit.J_acc_max - V_s * V_s / stKinematicLimit.A_acc_max - S_seg;
                    double V_e_dot_1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                    double V_e_dot_2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
                    //按不存在匀加速段修正终点速度
                    double x1 = 4 * pow(V_s,3) + 1.5 * (sqrt(81 * pow(stKinematicLimit.J_acc_max,2) * pow(S_seg,4) + 96 * pow(V_s,3) * stKinematicLimit.J_acc_max * pow(S_seg,2)) - 8 * pow(V_s,3) - 9 * stKinematicLimit.J_acc_max * pow(S_seg,2));
                    double x2 = 4 * pow(V_s,3) - 1.5 * (sqrt(81 * pow(stKinematicLimit.J_acc_max,2) * pow(S_seg,4) + 96 * pow(V_s,3) * stKinematicLimit.J_acc_max * pow(S_seg,2)) + 8 * pow(V_s,3) + 9 * stKinematicLimit.J_acc_max * pow(S_seg,2));
                    double V_e_dot_3 = ( -V_s - (Prescription(x1, 3) + Prescription(x2, 3))) / 3;
                    double V_s_tmp[3] = {V_e_dot_1, V_e_dot_2, V_e_dot_3};
                    v_Vmax[i] = *std::max_element(V_s_tmp, V_s_tmp + 3);;
                    v_Segmentation_V[i+1] = v_Vmax[i];
                }
            }
        }
    }
    v_Vel = v_Segmentation_V;
    return 0;
}

int MotionPlanning::AsymmetricSCurvesFixedLengthParametersCorrection(double V_s, double V_e,double V_max,ST_KinematicLimit stKinematicLimit, double S_seg,v_double& para)
{
    stKinematicLimit.A_dec_max = abs(stKinematicLimit.A_dec_max);
    stKinematicLimit.J_dec_max = abs(stKinematicLimit.J_dec_max);
        double T1 = 0; 
        double T2 = 0; 
        double T3 = 0;
        double T4 = 0;
        double T5 = 0; 
        double T6 = 0; 
        double T7 = 0;  
        double S_a = 0;
        double S_d =0;
    //步骤1  校验全程最大速度匀速运行可能性
    if(abs(V_s - V_max) < 1e-6 && abs(V_s - V_e) < 1e-6)
    {
        T1 = 0; 
        T2 = 0; 
        T3 = 0;
        T4 = S_seg / V_max;
        T5 = 0; 
        T6 = 0; 
        T7 = 0;    
        para = {T1, T2, T3, T4, T5, T6, T7, V_s, V_e, V_max, S_seg, stKinematicLimit.J_acc_max * T1, stKinematicLimit.J_acc_max, stKinematicLimit.J_dec_max * T5, stKinematicLimit.J_dec_max};
        return 0;
    }
    //步骤2  校验加速运动段和减速运动段的最大加速度和最大减速度可达性
    if (V_max - V_s > pow(stKinematicLimit.A_acc_max, 2) / stKinematicLimit.J_acc_max)   // 存在匀加速段，能够达到A_dec_max
    {
        T1 = stKinematicLimit.A_acc_max / stKinematicLimit.J_acc_max;
        T2 = (V_max - V_s) / stKinematicLimit.A_acc_max - T1;
        T3 = T1;
        S_a = (V_max + V_s) * ((V_max - V_s) / stKinematicLimit.A_acc_max + stKinematicLimit.A_acc_max / stKinematicLimit.J_acc_max);
    }
    else  //不存在匀加速段，达不到A_acc_max
    {
        T1 = sqrt(abs((V_max - V_s) / stKinematicLimit.J_acc_max));
        T2 = 0;
        T3 = T1;
        stKinematicLimit.A_acc_max = sqrt((V_max - V_s) * stKinematicLimit.J_acc_max);
        S_a = (V_max + V_s) * sqrt((V_max - V_s) / stKinematicLimit.J_acc_max);
    }
    if ((V_max - V_e) > abs(pow(stKinematicLimit.A_dec_max, 2) / stKinematicLimit.J_dec_max)) // 存在匀减速段，能够达到A_dec_max	
    {		
		T5 = stKinematicLimit.A_dec_max / stKinematicLimit.J_dec_max;
		T6 = (V_max - V_e) / abs(stKinematicLimit.A_dec_max) - T5;
        T7 = T5;
        S_d = (V_max + V_e) * ((V_max - V_e) / stKinematicLimit.A_acc_max + stKinematicLimit.A_acc_max / stKinematicLimit.J_dec_max);
    }
    else     //不存在匀减速段，达不到A_dec_miax		
    {
		T5 = sqrt((V_max - V_e) / abs(stKinematicLimit.J_dec_max));
		T6 = 0;
        T7 = T5;
        stKinematicLimit.A_dec_max = sqrt((V_max - V_e) * stKinematicLimit.J_dec_max);
        S_d = (V_max + V_e) * sqrt((V_max - V_e) / stKinematicLimit.J_dec_max);
    }
    //步骤3  校验最大速度可达性
    if (S_seg > (S_a + S_d))   //存在匀速段，实际最大速度能达到给定最大速度V_max
    {
        //计算匀速运动段时间
        T4 = (S_seg - S_a - S_d) / V_max;
        para = {T1, T2, T3, T4, T5, T6, T7, V_s, V_e, V_max, S_seg, stKinematicLimit.J_acc_max * T1, stKinematicLimit.J_acc_max, stKinematicLimit.J_dec_max * T5, stKinematicLimit.J_dec_max};
        return 0;
    }
    //步骤4
    //若S_seg < S_a + S_d，则不存在匀速段，实际最大速度不能达到给定最大速度V_max（即T4 = 0），
	//此时需跳转到步骤5，重新计算在给定S_seg条件下所能达到的实际最大速度
    //步骤5：二分法修正最大速度 
    double sigma = 0.001;  // 二分搜索条件（最大允许误差）
    double V_max_dot = std::max(V_s, V_e);
    double V_max_dot_dot = V_max; 
    uint32_t n = 1;
    while (abs(S_seg - S_a - S_d) > sigma && n < 100)
    {
        //重新计算在给定S_seg条件下所能达到的实际最大速度
        V_max = (V_max_dot + V_max_dot_dot) / 2.0;
        if ((V_max - V_s) > abs(pow(stKinematicLimit.A_acc_max, 2) / stKinematicLimit.J_acc_max))
        {
            T1 = stKinematicLimit.A_acc_max / stKinematicLimit.J_acc_max;
			T2 = (V_max - V_s) / stKinematicLimit.A_acc_max - T1;
            T3 = T1;
        }
		else 
        {
			T1 = sqrt((V_max - V_s) / stKinematicLimit.J_acc_max);
			T2 = 0;
            T3 = T1;
        }
		if ((V_max - V_e) > abs(pow(stKinematicLimit.A_dec_max, 2) / stKinematicLimit.J_dec_max))
        {
			T5 = stKinematicLimit.A_dec_max / stKinematicLimit.J_dec_max;
			T6 = (V_max - V_e) / abs(stKinematicLimit.A_dec_max) - T5;
			T7 = T5;
        }
		else 
        {
			T5 = sqrt(abs(((V_max - V_e) / stKinematicLimit.J_dec_max)));
			T6 = 0;
			T7 = T5;
        }
        S_a = V_s * (2 * T1 + T2) + 0.5 * stKinematicLimit.J_acc_max * pow(T1, 2) * (T1 + T2) + 0.5 * stKinematicLimit.J_acc_max * T1 * pow(T1 + T2, 2);
		S_d = V_max * (2 * T5 + T6) - 0.5 * stKinematicLimit.J_dec_max * pow(T5, 2) * (T5 + T6) - 0.5 * stKinematicLimit.J_dec_max * T5 * pow(T5 + T6, 2);

        if (S_seg > S_a + S_d + sigma)
        {
			V_max_dot = V_max;
			V_max_dot_dot = V_max_dot_dot;
        }
        else if( S_seg < S_a + S_d - sigma)
        {
			V_max_dot = V_max_dot;
			V_max_dot_dot = V_max;
        }
        n = n + 1;
    }//end while
    T4 = 0;
    para = {T1, T2, T3, T4, T5, T6, T7, V_s, V_e, V_max, S_seg, stKinematicLimit.J_acc_max * T1, stKinematicLimit.J_acc_max, stKinematicLimit.J_dec_max * T5, stKinematicLimit.J_dec_max};
    return 0;
}

int MotionPlanning::AsymmetricSCurveTimeScaling(double t,v_double para,ST_MotionFrame& stMotionFrame)
{
    double Tf = para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4] + para[en_t5] + para[en_t6] + para[en_t7];    //% 总时间
    para[en_J_dec_max] = abs(para[en_J_dec_max]);
    para[en_A_dec_max] = abs(para[en_A_dec_max]);
    //%% 各阶段起始速度
    double vst = para[en_V_s];
    double v1 = vst + 0.5 * para[en_J_acc_max] * pow(para[en_t1],2);
    double v2 = v1 + para[en_A_acc_max] * para[en_t2];
    double v3 = v2 + para[en_A_acc_max] * para[en_t3] - 0.5 * para[en_J_acc_max] * pow(para[en_t3],2);
	double v4 = v3;
	double v5 = v4 - 0.5 * para[en_J_dec_max] * pow(para[en_t5],2);
	double v6 = v5 - para[en_A_dec_max] * para[en_t6];
	double v7 = v6 - para[en_A_dec_max] * para[en_t7] + 0.5 * para[en_J_dec_max] * pow(para[en_t7],2);
    //起始点到各阶段点的位移
    double s1 = para[en_V_s] * para[en_t1] + 1.0 / 6.0 * para[en_J_acc_max]  * pow(para[en_t1], 3);
	double s2 = s1 + v1 * para[en_t2]  + 0.5 * para[en_A_acc_max] * pow(para[en_t2], 2);
	double s3 = s2 + v2 * para[en_t3]  + 0.5 * para[en_A_acc_max] * pow(para[en_t3], 2) - 1.0 / 6 * para[en_J_acc_max] * pow(para[en_t3], 3);
	double s4 = s3 + v3 * para[en_t4] ;
	double s5 = s4 + v4 * para[en_t5]  - 1.0 / 6 * para[en_J_dec_max]  * pow(para[en_t5], 3);
	double s6 = s5 + v5 * para[en_t6]  - 0.5 * para[en_A_dec_max]  * pow(para[en_t6], 2);
    if ((t >= 0) && (t < para[en_t1]))
    {
		stMotionFrame.pos = vst * t + 1.0 / 6 * para[en_J_acc_max] * pow(t, 3);
		stMotionFrame.vel = vst + 0.5 * para[en_J_acc_max] * pow(t, 2);
		stMotionFrame.acc = para[en_J_acc_max] * t;
		stMotionFrame.jerk = para[en_J_acc_max];
    }
    else if ((t >= para[en_t1]) && (t < para[en_t1] + para[en_t2]))
    {
		t = t - para[en_t1];
		stMotionFrame.pos = s1 + v1 * t + 0.5 * para[en_A_acc_max] * pow(t, 2);
		stMotionFrame.vel = v1 + para[en_A_acc_max] * t;
		stMotionFrame.acc = para[en_A_acc_max];
		stMotionFrame.jerk = 0;
    }
    else if( (t >= para[en_t1] + para[en_t2]) && (t < para[en_t1] + para[en_t2] + para[en_t3]))
    {
		t = t - para[en_t1] - para[en_t2];
		stMotionFrame.pos = s2 + v2 * t + 0.5 * para[en_A_acc_max] * pow(t, 2) - 1.0 / 6 * para[en_J_acc_max] * pow(t, 3);
		stMotionFrame.vel = v2 + para[en_A_acc_max] * t - 0.5 * para[en_J_acc_max] * pow(t, 2);
		stMotionFrame.acc = para[en_A_acc_max] - para[en_J_acc_max] * t;
		stMotionFrame.jerk = - para[en_J_acc_max];
    }
    else if ((t >= para[en_t1] + para[en_t2] + para[en_t3]) && (t < para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4]))
    {
		t = t - para[en_t1] - para[en_t2] - para[en_t3];
		stMotionFrame.pos = s3 + v3 * t;
		stMotionFrame.vel = v3;
		stMotionFrame.acc = 0;
		stMotionFrame.jerk = 0;
    }
    else if ((t >= para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4]) && (t < para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4] + para[en_t5]))
    {
		t = t - para[en_t1] - para[en_t2] - para[en_t3] - para[en_t4];
		stMotionFrame.pos = s4 + v4 * t - 1.0 / 6 * para[en_J_dec_max] * pow(t, 3);
		stMotionFrame.vel = v4 - 0.5 * para[en_J_dec_max] * pow(t, 2);
		stMotionFrame.acc = -para[en_J_dec_max] * t;
		stMotionFrame.jerk = -para[en_J_dec_max];
    }
    else if ((t >= para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4] + para[en_t5]) && (t < para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4] + para[en_t5] + para[en_t6]))
    {
		t = t - para[en_t1] - para[en_t2] - para[en_t3] - para[en_t4] - para[en_t5];
		stMotionFrame.pos = s5 + v5 * t - 0.5 * para[en_A_dec_max] * pow(t, 2);
		stMotionFrame.vel = v5 - para[en_A_dec_max] * t;
		stMotionFrame.acc = -para[en_A_dec_max];
		stMotionFrame.jerk = 0;
    }
    else if ((t >= para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4] + para[en_t5] + para[en_t6]) && (t < para[en_t1] + para[en_t2] + para[en_t3] + para[en_t4] + para[en_t5] + para[en_t6] + para[en_t7]))
    {
		t = t - para[en_t1] - para[en_t2] - para[en_t3] - para[en_t4] - para[en_t5] - para[en_t6];
		stMotionFrame.pos = s6 + v6 * t - 0.5 * para[en_A_dec_max] * pow(t, 2) + 1.0 / 6 * para[en_J_dec_max] * pow(t, 3);
		stMotionFrame.vel = v6 - para[en_A_dec_max]  * t + 0.5 * para[en_J_dec_max] * pow(t, 2);
		stMotionFrame.acc = -para[en_A_dec_max] + para[en_J_dec_max] * t;
		stMotionFrame.jerk = para[en_J_dec_max];
    }
    return 0;
}
