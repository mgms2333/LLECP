#include "MotionPlanning.h"

// 用于控制内存申请，设置允许的最大路径点个数
#define MAX_PATH_NUM_ALLOWED 100000
#define CalculationAccuracy 0.0000001
void free_trajectory_segment(trajectory_segment* traj) {
    if (traj->t != NULL) {
        free(traj->t);
        traj->t = NULL;
    }

    if (traj->q != NULL) {
        free(traj->q);
        traj->q = NULL;
    }

    if (traj->v != NULL) {
        free(traj->v);
        traj->v = NULL;
    }

    if (traj->a != NULL) {
        free(traj->a);
        traj->a = NULL;
    }

    if (traj->j != NULL) {
        free(traj->j);
        traj->j = NULL;
    }
}


STMotionFrame s_curve_generator_RT(double q0, double q1, double v0, double v1, double v_max, double a_max, double j_max, double t_start, double dt,double FrameTime)
{
    double v_min, a_min, j_min, v_max_origin, v_min_origin, a_max_origin, a_min_origin, j_max_origin, j_min_origin, v_m;
    int direction, n;
    double T_1, T_2, T_3, T_4, T_5, T_6, T_7, T_j1, T_j2, T_j, T_a, T_v, T_d;
    double t1, t2, t3, t4, t5, t6, t7;
    double delta, step, cur_t;
    STMotionFrame stMotionFrame;
    double cur_start_time, last_v, last_q;

    // 数据有效性判断
    if (abs(v0) > v_max) {
        printf("给定的起始速度超出可行速度范围！！！");
    }

    if (abs(v1) > v_max) {
        printf("给定的目标速度超出可行速度范围！！！");
    }

    v_min = -v_max;
    a_min = -a_max;
    j_min = -j_max;

    // 输入归一化处理
    v_max_origin = v_max;
    v_min_origin = v_min;
    a_max_origin = a_max;
    a_min_origin = a_min;
    j_max_origin = j_max;
    j_min_origin = j_min;

    direction = ((q1 - q0) > 0) ? 1 : -1;
    q0 = direction * q0;
    q1 = direction * q1;
    v0 = direction * v0;
    v1 = direction * v1;

    v_max = (direction + 1) / 2 * v_max_origin + (direction - 1) / 2 * v_min_origin;
    v_min = (direction + 1) / 2 * v_min_origin + (direction - 1) / 2 * v_max_origin;
    a_max = (direction + 1) / 2 * a_max_origin + (direction - 1) / 2 * a_min_origin;
    a_min = (direction + 1) / 2 * a_min_origin + (direction - 1) / 2 * a_max_origin;
    j_max = (direction + 1) / 2 * j_max_origin + (direction - 1) / 2 * j_min_origin;
    j_min = (direction + 1) / 2 * j_min_origin + (direction - 1) / 2 * j_max_origin;

    if ((v_max - v0) * j_max < a_max * a_max) {
        T_j1 = sqrt((v_max - v0) / j_max);
        T_a = 2 * T_j1;
    }
    else {
        T_j1 = a_max / j_max;
        T_a = T_j1 + (v_max - v0) / a_max;
    }

    if ((v_max - v1) * j_max < a_max * a_max) {
        T_j2 = sqrt((v_max - v1) / j_max);
        T_d = 2 * T_j2;
    }
    else {
        T_j2 = a_max / j_max;
        T_d = T_j2 + (v_max - v1) / a_max;
    }

    T_v = (q1 - q0) / v_max - T_a / 2 * (1 + v0 / v_max) - T_d / 2 * (1 + v1 / v_max);

    if (T_v >= 0) {
        // 最大速度可达
        v_m = v_max;
        //printf("最大可行速度可达, 最大速度: [%f]\n", v_m);
        T_1 = T_j1;
        T_2 = T_a - 2 * T_j1;
        T_3 = T_j1;
        T_4 = T_v;
        T_5 = T_j2;
        T_6 = T_d - 2 * T_j2;
        T_7 = T_j2;
    } else {
        // 最大速度不可达
        //printf("最大可行速度不可达\n");
        T_j1 = a_max / j_max;
        T_j2 = a_max / j_max;
        T_j = T_j1;
        delta = pow(a_max, 4) / (j_max * j_max) + 2 * (v0 * v0 + v1 * v1) + a_max * (4 * (q1 - q0) - 2 * a_max / j_max * (v0 + v1));
        T_a = (a_max * a_max / j_max - 2 * v0 + sqrt(delta)) / (2 * a_max);
        T_d = (a_max * a_max / j_max - 2 * v1 + sqrt(delta)) / (2 * a_max);
        T_v = 0;

        T_1 = T_j1;
        T_2 = T_a - 2 * T_j1;
        T_3 = T_j1;
        T_4 = T_v;
        T_5 = T_j2;
        T_6 = T_d - 2 * T_j2;
        T_7 = T_j2;

        if ((T_a >= 2 * T_j) && (T_d >= 2 * T_j)) {
            v_m = v0 + j_max * T_j1 * T_j1 + j_max * T_j1 * T_2;
            //printf("最大速度: [%f]\n", v_m);
        }
        else {
            //printf("递归计算可达到的最大加速度\n");
            step = CalculationAccuracy;
            for (double i = 1 - step; i = i - step; i >= step) {
                double temp = i * a_max;
                T_j = temp / j_max;
                delta = pow(temp, 4) / (j_max * j_max) + 2 * (v0 * v0 + v1 * v1) + temp * (4 * (q1 - q0) - 2 * temp / j_max * (v0 + v1));
                T_a = (temp * temp / j_max - 2 * v0 + sqrt(delta)) / (2 * temp);
                T_d = (temp * temp / j_max - 2 * v1 + sqrt(delta)) / (2 * temp);
                if ((T_a >= 2 * T_j) && (T_d >= 2 * T_j)) {
                    T_1 = T_j;
                    T_2 = T_a - 2 * T_j;
                    T_3 = T_j;
                    T_4 = 0;
                    T_5 = T_j;
                    T_6 = T_d - 2 * T_j;
                    T_7 = T_j;

                    v_m = v0 + j_max * T_j * T_j + j_max * T_j * T_2;
                    //printf("最大加速度不可达, 最大速度: %f\n", v_m);
                    break;
                }
                else if ((T_a < 0) && (v0 > v1)) {
                    //printf("无加速段，仅单一减速段\n");
                    T_a = 0;
                    T_d = 2 * (q1 - q0) / (v1 + v0);
                    T_j2 = (j_max * (q1 - q0) - sqrt(j_max * (j_max * (q1 - q0) * (q1 - q0) + (v1 + v0) * (v1 + v0) * (v1 - v0)))) / (j_max * (v1 + v0));

                    T_1 = 0;
                    T_2 = 0;
                    T_3 = 0;
                    T_4 = 0;
                    T_5 = T_j2;
                    T_6 = T_d - 2 * T_j2;
                    T_7 = T_j2;

                    v_m = v0;
                    //printf("最大速度:[%f]\n", v_m);
                    break;
                }
                else if ((T_d < 0) && (v0 < v1)) {
                    //printf("无减速段，仅单一加速段\n");
                    T_d = 0;
                    T_a = 2 * (q1 - q0) / (v1 + v0);
                    T_j1 = (j_max * (q1 - q0) - sqrt(j_max * (j_max * (q1 - q0) * (q1 - q0) - (v1 + v0) * (v1 + v0) * (v1 - v0)))) / (j_max * (v1 + v0));

                    T_1 = T_j1;
                    T_2 = T_a - 2 * T_j1;
                    T_3 = T_j1;
                    T_4 = 0;
                    T_5 = 0;
                    T_6 = 0;
                    T_7 = 0;

                    v_m = v0 + j_max * T_j1 * T_j1 + j_max * T_j1 * T_2;
                    //printf("最大速度: [% f] \n", v_m);
                    break;
                }
            }
        }
    }

    // 总时间
    double T = T_a + T_v + T_d;

    // 时间点划分
    t1 = t_start + T_1;
    t2 = t1 + T_2;
    t3 = t2 + T_3;
    t4 = t3 + T_4;
    t5 = t4 + T_5;
    t6 = t5 + T_6;
    t7 = t6 + T_7;
    stMotionFrame.r_t = t7 - FrameTime;
    // 生成时间序列
    n = (int)(ceil((t7 - t_start) / dt)) + 1;
    last_v = v0;
    last_q = q0;
    cur_t = FrameTime;
    if ((cur_t <= t1) && (T_1 > 0)) {
        cur_start_time = t_start;
        last_v = v0;
        last_q = q0;
        // 加加速阶段
        stMotionFrame.j = j_max;
        stMotionFrame.a = j_max * (cur_t - cur_start_time);
        stMotionFrame.v = last_v + (j_max * (cur_t - cur_start_time) * (cur_t - cur_start_time)) / 2;
        stMotionFrame.q = last_q + ((cur_t - cur_start_time) * (j_max * cur_t * cur_t - 2 * j_max * cur_t * cur_start_time + j_max * cur_start_time * cur_start_time + 6 * last_v)) / 6;
    }
    else if ((cur_t <= t2) && (T_2 > 0)) {
        cur_start_time = t_start + T_1;
        last_v = v0 + (j_max * T_1 * T_1) / 2;
        last_q = q0 + ((T_1) * (j_max * (t_start + T_1) * (t_start + T_1) - 2 * j_max * (t_start + T_1) * t_start + j_max * t_start * t_start + 6 * v0)) / 6;
        // 匀加速阶段
        stMotionFrame.j = 0;
        stMotionFrame.a = j_max * T_1;
        stMotionFrame.v = last_v + T_1 * j_max * (cur_t - cur_start_time);
        stMotionFrame.q = last_q + ((cur_t - cur_start_time) * (2 * last_v + T_1 * cur_t * j_max - T_1 * cur_start_time * j_max)) / 2;
    }
    else if ((cur_t <= t3) && (T_3 > 0)) {
        cur_start_time = t_start + T_1 + T_2;
        last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0;
        last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + v0 * T_1 + v0 * T_2 + q0;
        // 减加速阶段
        stMotionFrame.j = -j_max;
        stMotionFrame.a = j_max * T_1 - j_max * (cur_t - cur_start_time);
        stMotionFrame.v = last_v + (j_max * (cur_t - cur_start_time) * (2 * T_1 - cur_t + cur_start_time)) / 2;
        stMotionFrame.q = last_q + ((cur_t - cur_start_time) * (-j_max * cur_t * cur_t + 2 * j_max * cur_t * cur_start_time + 3 * T_1 * j_max * cur_t - j_max * cur_start_time * cur_start_time - 3 * T_1 * j_max * cur_start_time + 6 * last_v)) / 6;
    }
    else if ((cur_t <= t4) && (T_4 > 0)) {
        cur_start_time = t_start + T_1 + T_2 + T_3;
        last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2;
        last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + (j_max * T_1 * T_3 * T_3) / 2 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 + v0 * T_3 + q0;
        // 匀速阶段
        stMotionFrame.j = 0;
        stMotionFrame.a = 0;
        stMotionFrame.v = last_v;
        stMotionFrame.q = last_q + last_v * (cur_t - cur_start_time);
    }
    else if ((cur_t <= t5) && (T_5 > 0)) {
        cur_start_time = t_start + T_1 + T_2 + T_3 + T_4;
        last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2;
        last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (T_4 * j_max * T_1 * T_1) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + T_4 * j_max * T_1 * T_2 + (j_max * T_1 * T_3 * T_3) / 2 + T_4 * j_max * T_1 * T_3 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 - (T_4 * j_max * T_3 * T_3) / 2 + v0 * T_3 + q0 + T_4 * v0;
        // 加减速阶段
        stMotionFrame.j = j_min;
        stMotionFrame.a = j_min * (cur_t - cur_start_time);
        stMotionFrame.v = last_v + (j_min * (cur_t - cur_start_time) * (cur_t - cur_start_time)) / 2;
        stMotionFrame.q = last_q + ((cur_t - cur_start_time) * (j_min * cur_t * cur_t - 2 * j_min * cur_t * cur_start_time + j_min * cur_start_time * cur_start_time + 6 * last_v)) / 6;
    }
    else if ((cur_t <= t6) && (T_6 > 0)) {
        cur_start_time = t_start + T_1 + T_2 + T_3 + T_4 + T_5;
        last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2 + (j_min * T_5 * T_5) / 2;
        last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (j_max * T_1 * T_1 * T_5) / 2 + (T_4 * j_max * T_1 * T_1) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + j_max * T_1 * T_2 * T_5 + T_4 * j_max * T_1 * T_2 + (j_max * T_1 * T_3 * T_3) / 2 + j_max * T_1 * T_3 * T_5 + T_4 * j_max * T_1 * T_3 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 - (j_max * T_3 * T_3 * T_5) / 2 - (T_4 * j_max * T_3 * T_3) / 2 + v0 * T_3 + (j_min * pow(T_5, 3)) / 6 + v0 * T_5 + q0 + T_4 * v0;
        // 匀减速阶段
        stMotionFrame.j = 0;
        stMotionFrame.a = j_min * T_5;
        stMotionFrame.v = last_v + T_5 * j_min * (cur_t - cur_start_time);
        stMotionFrame.q = last_q + ((cur_t - cur_start_time) * (2 * last_v + T_5 * cur_t * j_min - T_5 * cur_start_time * j_min)) / 2;
    }
    else if ((T_7 > 0)) {
        cur_start_time = t_start + T_1 + T_2 + T_3 + T_4 + T_5 + T_6;
        last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2 + (j_min * T_5 * T_5) / 2 + T_5 * j_min * T_6;
        last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (j_max * T_1 * T_1 * T_5) / 2 + (j_max * T_1 * T_1 * T_6) / 2 + (T_4 * j_max * T_1 * T_1) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + j_max * T_1 * T_2 * T_5 + j_max * T_1 * T_2 * T_6 + T_4 * j_max * T_1 * T_2 + (j_max * T_1 * T_3 * T_3) / 2 + j_max * T_1 * T_3 * T_5 + j_max * T_1 * T_3 * T_6 + T_4 * j_max * T_1 * T_3 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 - (j_max * T_3 * T_3 * T_5) / 2 - (j_max * T_3 * T_3 * T_6) / 2 - (T_4 * j_max * T_3 * T_3) / 2 + v0 * T_3 + (j_min * pow(T_5, 3)) / 6 + (j_min * T_5 * T_5 * T_6) / 2 + (j_min * T_5 * T_6 * T_6) / 2 + v0 * T_5 + v0 * T_6 + q0 + T_4 * v0;
        // 减减速阶段
        stMotionFrame.j = -j_min;
        stMotionFrame.a = j_min * T_5 - j_min * (cur_t - cur_start_time);
        stMotionFrame.v = last_v + (j_min * (cur_t - cur_start_time) * (2 * T_5 - cur_t + cur_start_time)) / 2;
        stMotionFrame.q = last_q + ((cur_t - cur_start_time) * (-j_min * cur_t * cur_t + 2 * j_min * cur_t * cur_start_time + 3 * T_5 * j_min * cur_t - j_min * cur_start_time * cur_start_time - 3 * T_5 * j_min * cur_start_time + 6 * last_v)) / 6;
    }
    stMotionFrame.j = direction * stMotionFrame.j;
    stMotionFrame.a = direction * stMotionFrame.a;
    stMotionFrame.v = direction * stMotionFrame.v;
    stMotionFrame.q = direction * stMotionFrame.q;
    return stMotionFrame;
}

trajectory_segment s_curve_generator(double q0, double q1, double v0, double v1, double v_max, double a_max, double j_max, double t_start, double dt) {
    double v_min, a_min, j_min, v_max_origin, v_min_origin, a_max_origin, a_min_origin, j_max_origin, j_min_origin, v_m;
    int direction, n;
    double T_1, T_2, T_3, T_4, T_5, T_6, T_7, T_j1, T_j2, T_j, T_a, T_v, T_d;
    double t1, t2, t3, t4, t5, t6, t7;
    double delta, step, cur_t;
    trajectory_segment res;
    double cur_start_time, last_v, last_q;

    // 数据有效性判断
    if (abs(v0) > v_max) {
        printf("给定的起始速度超出可行速度范围！！！");
    }

    if (abs(v1) > v_max) {
        printf("给定的目标速度超出可行速度范围！！！");
    }

    v_min = -v_max;
    a_min = -a_max;
    j_min = -j_max;

    // 输入归一化处理
    v_max_origin = v_max;
    v_min_origin = v_min;
    a_max_origin = a_max;
    a_min_origin = a_min;
    j_max_origin = j_max;
    j_min_origin = j_min;

    direction = ((q1 - q0) > 0) ? 1 : -1;
    q0 = direction * q0;
    q1 = direction * q1;
    v0 = direction * v0;
    v1 = direction * v1;

    v_max = (direction + 1) / 2 * v_max_origin + (direction - 1) / 2 * v_min_origin;
    v_min = (direction + 1) / 2 * v_min_origin + (direction - 1) / 2 * v_max_origin;
    a_max = (direction + 1) / 2 * a_max_origin + (direction - 1) / 2 * a_min_origin;
    a_min = (direction + 1) / 2 * a_min_origin + (direction - 1) / 2 * a_max_origin;
    j_max = (direction + 1) / 2 * j_max_origin + (direction - 1) / 2 * j_min_origin;
    j_min = (direction + 1) / 2 * j_min_origin + (direction - 1) / 2 * j_max_origin;

    /**
    * v_m:   运动过程中的最大速度
    * T_j1 : 加速段中加加速度恒定（j_max或j_min）恒定的时长
    * T_j2 : 减速段中加加速度恒定（j_max或j_min）恒定的时长
    * T_a :  加速阶段时长
    * T_v :  匀速阶段时长
    * T_d :  减速阶段时长
    * T :    轨迹总时长（T_a + T_v + T_d）
    */

    if ((v_max - v0) * j_max < a_max * a_max) {
        T_j1 = sqrt((v_max - v0) / j_max);
        T_a = 2 * T_j1;
    }
    else {
        T_j1 = a_max / j_max;
        T_a = T_j1 + (v_max - v0) / a_max;
    }

    if ((v_max - v1) * j_max < a_max * a_max) {
        T_j2 = sqrt((v_max - v1) / j_max);
        T_d = 2 * T_j2;
    }
    else {
        T_j2 = a_max / j_max;
        T_d = T_j2 + (v_max - v1) / a_max;
    }

    T_v = (q1 - q0) / v_max - T_a / 2 * (1 + v0 / v_max) - T_d / 2 * (1 + v1 / v_max);

    if (T_v >= 0) {
        // 最大速度可达
        v_m = v_max;
        printf("最大可行速度可达, 最大速度: [%f]\n", v_m);
        T_1 = T_j1;
        T_2 = T_a - 2 * T_j1;
        T_3 = T_j1;
        T_4 = T_v;
        T_5 = T_j2;
        T_6 = T_d - 2 * T_j2;
        T_7 = T_j2;
    } else {
        // 最大速度不可达
        printf("最大可行速度不可达\n");
        T_j1 = a_max / j_max;
        T_j2 = a_max / j_max;
        T_j = T_j1;
        delta = pow(a_max, 4) / (j_max * j_max) + 2 * (v0 * v0 + v1 * v1) + a_max * (4 * (q1 - q0) - 2 * a_max / j_max * (v0 + v1));
        T_a = (a_max * a_max / j_max - 2 * v0 + sqrt(delta)) / (2 * a_max);
        T_d = (a_max * a_max / j_max - 2 * v1 + sqrt(delta)) / (2 * a_max);
        T_v = 0;

        T_1 = T_j1;
        T_2 = T_a - 2 * T_j1;
        T_3 = T_j1;
        T_4 = T_v;
        T_5 = T_j2;
        T_6 = T_d - 2 * T_j2;
        T_7 = T_j2;

        if ((T_a >= 2 * T_j) && (T_d >= 2 * T_j)) {
            v_m = v0 + j_max * T_j1 * T_j1 + j_max * T_j1 * T_2;
            printf("最大速度: [%f]\n", v_m);
        }
        else {
            printf("递归计算可达到的最大加速度\n");
            step = CalculationAccuracy;
            for (double i = 1 - step; i = i - step; i >= step) {
                double temp = i * a_max;
                T_j = temp / j_max;
                delta = pow(temp, 4) / (j_max * j_max) + 2 * (v0 * v0 + v1 * v1) + temp * (4 * (q1 - q0) - 2 * temp / j_max * (v0 + v1));
                T_a = (temp * temp / j_max - 2 * v0 + sqrt(delta)) / (2 * temp);
                T_d = (temp * temp / j_max - 2 * v1 + sqrt(delta)) / (2 * temp);
                if ((T_a >= 2 * T_j) && (T_d >= 2 * T_j)) {
                    T_1 = T_j;
                    T_2 = T_a - 2 * T_j;
                    T_3 = T_j;
                    T_4 = 0;
                    T_5 = T_j;
                    T_6 = T_d - 2 * T_j;
                    T_7 = T_j;

                    v_m = v0 + j_max * T_j * T_j + j_max * T_j * T_2;
                    printf("最大加速度不可达, 最大速度: %f\n", v_m);
                    break;
                }
                else if ((T_a < 0) && (v0 > v1)) {
                    printf("无加速段，仅单一减速段\n");
                    T_a = 0;
                    T_d = 2 * (q1 - q0) / (v1 + v0);
                    T_j2 = (j_max * (q1 - q0) - sqrt(j_max * (j_max * (q1 - q0) * (q1 - q0) + (v1 + v0) * (v1 + v0) * (v1 - v0)))) / (j_max * (v1 + v0));

                    T_1 = 0;
                    T_2 = 0;
                    T_3 = 0;
                    T_4 = 0;
                    T_5 = T_j2;
                    T_6 = T_d - 2 * T_j2;
                    T_7 = T_j2;

                    v_m = v0;
                    printf("最大速度:[%f]\n", v_m);
                    break;
                }
                else if ((T_d < 0) && (v0 < v1)) {
                    printf("无减速段，仅单一加速段\n");
                    T_d = 0;
                    T_a = 2 * (q1 - q0) / (v1 + v0);
                    T_j1 = (j_max * (q1 - q0) - sqrt(j_max * (j_max * (q1 - q0) * (q1 - q0) - (v1 + v0) * (v1 + v0) * (v1 - v0)))) / (j_max * (v1 + v0));

                    T_1 = T_j1;
                    T_2 = T_a - 2 * T_j1;
                    T_3 = T_j1;
                    T_4 = 0;
                    T_5 = 0;
                    T_6 = 0;
                    T_7 = 0;

                    v_m = v0 + j_max * T_j1 * T_j1 + j_max * T_j1 * T_2;
                    printf("最大速度: [% f] \n", v_m);
                    break;
                }
            }
        }
    }

    // 总时间
    res.T = T_a + T_v + T_d;

    // 时间点划分
    t1 = t_start + T_1;
    t2 = t1 + T_2;
    t3 = t2 + T_3;
    t4 = t3 + T_4;
    t5 = t4 + T_5;
    t6 = t5 + T_6;
    t7 = t6 + T_7;

    // 生成时间序列
    n = (int)(ceil((t7 - t_start) / dt)) + 1;

    res.t = (double*)malloc(n * sizeof(double));
    if (res.t == NULL) {
        printf("内存申请失败\n");
        return res;
    }

    for (int i = 0; i < n; i++) {
        res.t[i] = t_start + i * dt;
    }

    // 初始化输出
    res.q = (double*)malloc(n * sizeof(double));
    res.v = (double*)malloc(n * sizeof(double));
    res.a = (double*)malloc(n * sizeof(double));
    res.j = (double*)malloc(n * sizeof(double));

    res.path_num = n;

    // cur_start_time:  当前阶段的起始时刻
    // last_v : 上一阶段的末速度，也即当前阶段的起始速度
    // last_q : 上一阶段的末位置，也即当前阶段的起始位置

    last_v = v0;
    last_q = q0;
    // 分段计算轨迹
    for (int i = 0; i < n; i++) {
        cur_t = res.t[i];
        if ((cur_t <= t1) && (T_1 > 0)) {
            cur_start_time = t_start;
            last_v = v0;
            last_q = q0;
            // 加加速阶段
            res.j[i] = j_max;
            res.a[i] = j_max * (cur_t - cur_start_time);
            res.v[i] = last_v + (j_max * (cur_t - cur_start_time) * (cur_t - cur_start_time)) / 2;
            res.q[i] = last_q + ((cur_t - cur_start_time) * (j_max * cur_t * cur_t - 2 * j_max * cur_t * cur_start_time + j_max * cur_start_time * cur_start_time + 6 * last_v)) / 6;
        }
        else if ((cur_t <= t2) && (T_2 > 0)) {
            cur_start_time = t_start + T_1;
            last_v = v0 + (j_max * T_1 * T_1) / 2;
            last_q = q0 + ((T_1) * (j_max * (t_start + T_1) * (t_start + T_1) - 2 * j_max * (t_start + T_1) * t_start + j_max * t_start * t_start + 6 * v0)) / 6;
            // 匀加速阶段
            res.j[i] = 0;
            res.a[i] = j_max * T_1;
            res.v[i] = last_v + T_1 * j_max * (cur_t - cur_start_time);
            res.q[i] = last_q + ((cur_t - cur_start_time) * (2 * last_v + T_1 * cur_t * j_max - T_1 * cur_start_time * j_max)) / 2;
        }
        else if ((cur_t <= t3) && (T_3 > 0)) {
            cur_start_time = t_start + T_1 + T_2;
            last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0;
            last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + v0 * T_1 + v0 * T_2 + q0;
            // 减加速阶段
            res.j[i] = -j_max;
            res.a[i] = j_max * T_1 - j_max * (cur_t - cur_start_time);
            res.v[i] = last_v + (j_max * (cur_t - cur_start_time) * (2 * T_1 - cur_t + cur_start_time)) / 2;
            res.q[i] = last_q + ((cur_t - cur_start_time) * (-j_max * cur_t * cur_t + 2 * j_max * cur_t * cur_start_time + 3 * T_1 * j_max * cur_t - j_max * cur_start_time * cur_start_time - 3 * T_1 * j_max * cur_start_time + 6 * last_v)) / 6;
        }
        else if ((cur_t <= t4) && (T_4 > 0)) {
            cur_start_time = t_start + T_1 + T_2 + T_3;
            last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2;
            last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + (j_max * T_1 * T_3 * T_3) / 2 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 + v0 * T_3 + q0;
            // 匀速阶段
            res.j[i] = 0;
            res.a[i] = 0;
            res.v[i] = last_v;
            res.q[i] = last_q + last_v * (cur_t - cur_start_time);
        }
        else if ((cur_t <= t5) && (T_5 > 0)) {
            cur_start_time = t_start + T_1 + T_2 + T_3 + T_4;
            last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2;
            last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (T_4 * j_max * T_1 * T_1) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + T_4 * j_max * T_1 * T_2 + (j_max * T_1 * T_3 * T_3) / 2 + T_4 * j_max * T_1 * T_3 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 - (T_4 * j_max * T_3 * T_3) / 2 + v0 * T_3 + q0 + T_4 * v0;
            // 加减速阶段
            res.j[i] = j_min;
            res.a[i] = j_min * (cur_t - cur_start_time);
            res.v[i] = last_v + (j_min * (cur_t - cur_start_time) * (cur_t - cur_start_time)) / 2;
            res.q[i] = last_q + ((cur_t - cur_start_time) * (j_min * cur_t * cur_t - 2 * j_min * cur_t * cur_start_time + j_min * cur_start_time * cur_start_time + 6 * last_v)) / 6;
        }
        else if ((cur_t <= t6) && (T_6 > 0)) {
            cur_start_time = t_start + T_1 + T_2 + T_3 + T_4 + T_5;
            last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2 + (j_min * T_5 * T_5) / 2;
            last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (j_max * T_1 * T_1 * T_5) / 2 + (T_4 * j_max * T_1 * T_1) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + j_max * T_1 * T_2 * T_5 + T_4 * j_max * T_1 * T_2 + (j_max * T_1 * T_3 * T_3) / 2 + j_max * T_1 * T_3 * T_5 + T_4 * j_max * T_1 * T_3 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 - (j_max * T_3 * T_3 * T_5) / 2 - (T_4 * j_max * T_3 * T_3) / 2 + v0 * T_3 + (j_min * pow(T_5, 3)) / 6 + v0 * T_5 + q0 + T_4 * v0;
            // 匀减速阶段
            res.j[i] = 0;
            res.a[i] = j_min * T_5;
            res.v[i] = last_v + T_5 * j_min * (cur_t - cur_start_time);
            res.q[i] = last_q + ((cur_t - cur_start_time) * (2 * last_v + T_5 * cur_t * j_min - T_5 * cur_start_time * j_min)) / 2;
        }
        else if ((T_7 > 0)) {
            cur_start_time = t_start + T_1 + T_2 + T_3 + T_4 + T_5 + T_6;
            last_v = (j_max * T_1 * T_1) / 2 + T_2 * j_max * T_1 + v0 + (j_max * T_3 * (2 * T_1 - T_3)) / 2 + (j_min * T_5 * T_5) / 2 + T_5 * j_min * T_6;
            last_q = (j_max * pow(T_1, 3)) / 6 + (j_max * T_1 * T_1 * T_2) / 2 + (j_max * T_1 * T_1 * T_3) / 2 + (j_max * T_1 * T_1 * T_5) / 2 + (j_max * T_1 * T_1 * T_6) / 2 + (T_4 * j_max * T_1 * T_1) / 2 + (j_max * T_1 * T_2 * T_2) / 2 + j_max * T_1 * T_2 * T_3 + j_max * T_1 * T_2 * T_5 + j_max * T_1 * T_2 * T_6 + T_4 * j_max * T_1 * T_2 + (j_max * T_1 * T_3 * T_3) / 2 + j_max * T_1 * T_3 * T_5 + j_max * T_1 * T_3 * T_6 + T_4 * j_max * T_1 * T_3 + v0 * T_1 + v0 * T_2 - (j_max * pow(T_3, 3)) / 6 - (j_max * T_3 * T_3 * T_5) / 2 - (j_max * T_3 * T_3 * T_6) / 2 - (T_4 * j_max * T_3 * T_3) / 2 + v0 * T_3 + (j_min * pow(T_5, 3)) / 6 + (j_min * T_5 * T_5 * T_6) / 2 + (j_min * T_5 * T_6 * T_6) / 2 + v0 * T_5 + v0 * T_6 + q0 + T_4 * v0;
            // 减减速阶段
            res.j[i] = -j_min;
            res.a[i] = j_min * T_5 - j_min * (cur_t - cur_start_time);
            res.v[i] = last_v + (j_min * (cur_t - cur_start_time) * (2 * T_5 - cur_t + cur_start_time)) / 2;
            res.q[i] = last_q + ((cur_t - cur_start_time) * (-j_min * cur_t * cur_t + 2 * j_min * cur_t * cur_start_time + 3 * T_5 * j_min * cur_t - j_min * cur_start_time * cur_start_time - 3 * T_5 * j_min * cur_start_time + 6 * last_v)) / 6;
        }
    }

    printf("S速度规划各阶段时长:[%f, %f, %f, %f, %f, %f, %f]\n", T_1, T_2, T_3, T_4, T_5, T_6, T_7);
    printf("加速、匀速、减速时长:[%f, %f, %f], 总时长:[%f]\n", T_a, T_v, T_d, res.T);
    printf("路径点个数:[%d]\n", res.path_num);

    // 输出归一化处理
    for (int i = 0; i < n; i++) {
        res.q[i] = direction * res.q[i];
        res.v[i] = direction * res.v[i];
        res.a[i] = direction * res.a[i];
        res.j[i] = direction * res.j[i];
    }

    return res;
}

trajectory_segment multi_s_curve_generator(double* q_points, double* v_points, double v_max, double a_max, double j_max, double start_time, double dt, int N) {
    double cur_t_start = start_time;
    double q0, q1, v0, v1;
    trajectory_segment temp_res, res;
    int cur_path_num = 0, last_path_num = 0;
    double T = 0;

    temp_res.t = (double*)malloc(MAX_PATH_NUM_ALLOWED * sizeof(double));
    temp_res.q = (double*)malloc(MAX_PATH_NUM_ALLOWED * sizeof(double));
    temp_res.v = (double*)malloc(MAX_PATH_NUM_ALLOWED * sizeof(double));
    temp_res.a = (double*)malloc(MAX_PATH_NUM_ALLOWED * sizeof(double));
    temp_res.j = (double*)malloc(MAX_PATH_NUM_ALLOWED * sizeof(double));

    for (int i = 0; i < N - 1; i++) {
        q0 = q_points[i];
        q1 = q_points[i + 1];
        v0 = v_points[i];
        v1 = v_points[i + 1];

        trajectory_segment cur_trajectory_segment = s_curve_generator(q0, q1, v0, v1, v_max, a_max, j_max, cur_t_start, dt);
        cur_t_start = cur_t_start + cur_trajectory_segment.T;
        if (i == 0) {
            cur_path_num = cur_path_num + cur_trajectory_segment.path_num;
        }
        else {
            cur_path_num = cur_path_num + cur_trajectory_segment.path_num - 1;
        }

        T += cur_trajectory_segment.T;

        if (cur_path_num <= MAX_PATH_NUM_ALLOWED) {
            if (i == 0) {
                for (int j = 0; j < cur_trajectory_segment.path_num; j++) {
                    temp_res.t[j] = cur_trajectory_segment.t[j];
                    temp_res.q[j] = cur_trajectory_segment.q[j];
                    temp_res.v[j] = cur_trajectory_segment.v[j];
                    temp_res.a[j] = cur_trajectory_segment.a[j];
                    temp_res.j[j] = cur_trajectory_segment.j[j];
                }
            } else {
                for (int j = 0; j < cur_trajectory_segment.path_num; j++) {
                    temp_res.t[last_path_num - 1 + j] = cur_trajectory_segment.t[j];
                    temp_res.q[last_path_num - 1 + j] = cur_trajectory_segment.q[j];
                    temp_res.v[last_path_num - 1 + j] = cur_trajectory_segment.v[j];
                    temp_res.a[last_path_num - 1 + j] = cur_trajectory_segment.a[j];
                    temp_res.j[last_path_num - 1 + j] = cur_trajectory_segment.j[j];
                }
            }

            // 释放每段S速度规划申请的内存
            free_trajectory_segment(&cur_trajectory_segment);
        } else {
            free_trajectory_segment(&temp_res);
            // 释放每段S速度规划申请的内存
            free_trajectory_segment(&cur_trajectory_segment);
            return temp_res;
        }

        last_path_num = cur_path_num;
    }

    temp_res.path_num = cur_path_num;
    temp_res.T = T;

    res.path_num = cur_path_num;
    res.T = T;

    res.t = (double*)malloc(cur_path_num * sizeof(double));
    res.q = (double*)malloc(cur_path_num * sizeof(double));
    res.v = (double*)malloc(cur_path_num * sizeof(double));
    res.a = (double*)malloc(cur_path_num * sizeof(double));
    res.j = (double*)malloc(cur_path_num * sizeof(double));

    for (int i = 0; i < cur_path_num; i++) {
        res.t[i] = temp_res.t[i];
        res.q[i] = temp_res.q[i];
        res.v[i] = temp_res.v[i];
        res.a[i] = temp_res.a[i];
        res.j[i] = temp_res.j[i];
    }

    free_trajectory_segment(&temp_res);
    printf("多点间实现S型加减速规划, 规划后轨迹点个数:[%d], 轨迹总时长:[%f], 控制周期:[%f]\n", res.path_num, res.T, dt);

    return res;
}

trajectory_segment multi_s_curve_generator_based_on_path(double* q_points, double v_start, double v_end, double v_max, double a_max, double j_max, double start_time, double dt, int N) {
    double* v_points  = (double*)malloc(N * sizeof(double));

    v_points[0] = v_start;
    v_points[N - 1] = v_end;

    for (int i = 1; i < N - 1; i++) {
        v_points[i] = v_start + i / (N - 1) * (v_end - v_start);
    }

    trajectory_segment res = multi_s_curve_generator(q_points, v_points, v_max, a_max, j_max, start_time, dt, N);

    if (v_points != NULL) {
        free(v_points);
        v_points = NULL;
    }

    return res;
}