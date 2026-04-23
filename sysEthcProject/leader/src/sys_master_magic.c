/*** 
 * @Description: 
 * @Author: lyq
 * @Date: 2024-06-07 07:18:48
 * @LastEditTime: 2024-06-14 00:27:45
 * @LastEditors: lyq
 */
/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_loger.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:日志模块，系统运行的情况记录在这个模块中。
*
********************************************************************************/
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>

#include "sys_master_magic.h"
#include "sys_leader_defs.h"
#include "sys_master.h"
#include "sys_share_conf.h"
#include "sys_share_daemon.h"
#include "sys_share_ipc.h"
#include "sys_share_messages.h"

//静态函数
static void magic_vertical_weight(MAGIC_THREAD_INFO *pTmagic,float cycTime);
static void magic_follow(MAGIC_THREAD_INFO *pTmagic,float x_offset, float y_offset, float cycTime);
static void magic_vertical(MAGIC_THREAD_INFO *pTmagic,float cycTime);
static float deadZone(float mIndead, float mthr);
//static int generate_sim_follow_camxy(MAGIC_THREAD_INFO *pTmagic,float *x_offset, float *y_offset);
//滤波器进行数据滤波
static int32_t filtering(ARMS_MASTER_THREAD_INFO *pTM, int id);
static float   smooth_filter(float *pvalues, float newValue,int avgSize);
static void    sensors_read_i_to_f(ARMS_MASTER_THREAD_INFO* pTModule,int armsid);

//计算附加显示的信息。比如速度命令和执行的差等等
static void  magic_calcu_others(MAGIC_THREAD_INFO *pTmagic,float cycTime);


/******************************************************************************
* 功能：tsnorm函数将timespec中的纳秒，妙格式化。按照1m = 1000000000ns，
* @param ts : ts是时间结构体，包含：妙 纳秒
* @return Descriptions
******************************************************************************/
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

/******************************************************************************
* 功能：采集的传感器数据转换成float类型真实值
* @return Descriptions
******************************************************************************/
static void  sensors_read_i_to_f(ARMS_MASTER_THREAD_INFO* pTModule,int armsid)
{
    //伺服相关的，放在bus线程中。
    /*index: 0 1  倾角仪数据。 2    拉力计数据 3 4 5增量式磁栅尺XYZ*/
    //倾角仪，单位值m/s2
    pTModule->incln_acc_XY[armsid][0] = pTModule->mVisionWorker.inclinxy[armsid][0]; 
    pTModule->incln_acc_XY[armsid][1] = pTModule->mVisionWorker.inclinxy[armsid][1];
    
    //XY向的磁栅尺换成相机检测的数据,camXY已经滤波过
    pTModule->sikoXYZ[armsid][0]      = pTModule->camXY[armsid][0];
    pTModule->sikoXYZ[armsid][1]      = pTModule->camXY[armsid][1];
    //气缸内的Z轴编码器接在驱动器Z轴的外部编码器接口
    pTModule->sikoXYZ[armsid][2]      =  pTModule->zExtPos[armsid];

    //磁栅尺0位置补偿
    pTModule->sikoXYZ[armsid][0] += pTModule->sikoXYZ_ZERO[armsid][0];
    pTModule->sikoXYZ[armsid][1] += pTModule->sikoXYZ_ZERO[armsid][1];
    pTModule->sikoXYZ[armsid][2] += pTModule->sikoXYZ_ZERO[armsid][2];
 
    //差分Z磁栅尺速度、加速度
    pTModule->sikoZ_V[armsid]      = (pTModule->sikoXYZ[armsid][2] - pTModule->sikoZ_Last[armsid])*200.0;
    pTModule->sikoZ_A[armsid]      = (pTModule->sikoZ_V[armsid] - pTModule->sikoZ_V_Last[armsid])*200.0;
    pTModule->sikoZ_Last[armsid]   = pTModule->sikoXYZ[armsid][2];
    pTModule->sikoZ_V_Last[armsid] = pTModule->sikoZ_V[armsid];

    /**EC-3045采集的传感器 绝压传感器,由于绝压采集频率低所以20hz的采集*/
    //绝压传感器转换,pa
    pTModule->abs_stress[armsid] = ((float)pTModule->EC_3045_abs_stress[armsid])*16.0; 
}

static void  magic_calcu_others(MAGIC_THREAD_INFO *pTmagic,float cycTime)
{
    ARMS_MASTER_THREAD_INFO* pMaster = pTmagic->pMaster;
    int ai = pTmagic->mArmsId;
/******************************四轴的速度命令和执行速度差*******************************/
    pTmagic->cmd_vel_error[0] = pMaster->cmd_velocity[ai][0] - pMaster->xMIntSpeed[ai];
    pTmagic->cmd_vel_error[1] = pMaster->cmd_velocity[ai][1] - pMaster->xSIntSpeed[ai];
    pTmagic->cmd_vel_error[2] = pMaster->cmd_velocity[ai][2] - pMaster->ySpeed[ai];
    pTmagic->cmd_vel_error[3] = pMaster->cmd_velocity[ai][3] - pMaster->zSpeed[ai];

/*******************************计算四轴的均方差***************************************/
    //存储20个数据,进行移动
    for (int i = 0; i < 4; i++)
    {
        double add_mean_error = 0.0;
        for (int j = 0; j < CMD_VEL_MEAN_ERROR_SIZE-1; j++)
        {
            pTmagic->mean_errors[i][j] = pTmagic->mean_errors[i][j+1];
            add_mean_error += pTmagic->mean_errors[i][j];
        }
        //计算当前周期内的速度方差
        pTmagic->mean_errors[i][CMD_VEL_MEAN_ERROR_SIZE-1] = pTmagic->cmd_vel_error[i]*pTmagic->cmd_vel_error[i];
        
        add_mean_error += pTmagic->mean_errors[i][CMD_VEL_MEAN_ERROR_SIZE-1];
        add_mean_error /= (double)(CMD_VEL_MEAN_ERROR_SIZE);
        pTmagic->cmd_vel_mean_error_n[i] = (float)sqrt(add_mean_error);
    }
}

static int32_t filtering(ARMS_MASTER_THREAD_INFO *pTM, int id)
{
    /*伺服电机速度 位置滤波*/
    FILTERING_PARAM * pf = (FILTERING_PARAM *)&pTM->mF_Inf[id];

    /*
    pTM->xMIntSpeed[id] = smooth_filter(pf->xM_vel_arr,pTM->xMIntSpeed[id],pf->F_vel_n);
    pTM->xSIntSpeed[id] = smooth_filter(pf->xS_vel_arr,pTM->xSIntSpeed[id],pf->F_vel_n);
    pTM->ySpeed[id]  = smooth_filter(pf->y_vel_arr,pTM->ySpeed[id],pf->F_vel_n);
    pTM->zSpeed[id]  = smooth_filter(pf->z_vel_arr,pTM->zSpeed[id],pf->F_vel_n);
    */
    /*index: 0 1  倾角仪数据。 2    拉力计数据 3 4 5增量式磁栅尺XYZ*/
    //倾角仪，单位值m/s2
    /*伺服电机速度 位置滤波*/
    pTM->incln_acc_XY[id][0] = smooth_filter(pf->incln_X_vec, pTM->incln_acc_XY[id][0],pf->F_incln_n);
    pTM->incln_acc_XY[id][1] = smooth_filter(pf->incln_Y_vec, pTM->incln_acc_XY[id][1],pf->F_incln_n);
    //磁栅尺，单位是m. MR200分辨率是1um.暂时不适用磁栅尺
    //pTM->sikoXYZ[id][0]=smooth_filter(pf->sikoX_arr, pTM->sikoXYZ[id][0],pf->F_siko_n);
    //pTM->sikoXYZ[id][1]=smooth_filter(pf->sikoY_arr, pTM->sikoXYZ[id][1],pf->F_siko_n);
    //printf("XXXXXXXXX %f %f\n",pTM->sikoXYZ[id][0],pTM->sikoXYZ[id][1]);

    pTM->sikoXYZ[id][2]=smooth_filter(pf->sikoZ_arr, pTM->sikoXYZ[id][2],pf->F_siko_n);

    pTM->sikoZ_V[id]=smooth_filter(pf->sikoZ_V_arr, pTM->sikoZ_V[id],pf->F_siko_n);
    pTM->sikoZ_A[id]=smooth_filter(pf->sikoZ_A_arr, pTM->sikoZ_A[id],pf->F_siko_n);

    //相机检测到绳索偏移单位是m 分辨率是1um
    //pTM->camXY[armsid][0]=smooth_filter(pf->camX_vec,pTM->camXY[armsid][0],pf->F_camTimes);
    //pTM->camXY[armsid][1]=smooth_filter(pf->camY_vec,pTM->camXY[armsid][1],pf->F_camTimes);
    
    //绝压传感器滤波
    pTM->abs_stress[id] = smooth_filter(pTM->mF_Inf[id].abs_press_arr,pTM->abs_stress[id],pTM->mF_Inf[id].F_abs_n);
    return 0;
}

static float smooth_filter(float *pvalues, float newValue,int avgSize)
{
    float iRet = 0.0f;

    if(avgSize<=1)
        return newValue;
    if(avgSize>F_ARR_SIZE)
        avgSize = F_ARR_SIZE-1;

    //先移动数据
    for (int i = 0; i < avgSize; i++)
    {
        pvalues[i] = pvalues[i+1];
    }
    pvalues[avgSize] = newValue;
       
    for (int i = 0; i < avgSize; i++)
    {
        iRet += pvalues[i];
    }
    iRet /= avgSize;

    return iRet;
}

/******************************************************************************
* 功能：此函数销毁初始化模块，释放线程指针里的内容
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int moduleEndUp(MAGIC_THREAD_INFO *pTModule)
{

  return 0;
}

static float deadZone(float mIndead, float mthr)
{
  float iRet = mIndead;
  if(fabs(iRet) <= fabs(mthr))
  {
    iRet = 0;
  }
  else if(iRet > fabs(mthr))
  {
    iRet -= fabs(mthr);
  }
  else if(iRet < -fabs(mthr))
  {
    iRet += fabs(mthr);
  }
  return iRet;
}

//随动算法
static void magic_follow(MAGIC_THREAD_INFO *pTmagic,float x_offset, float y_offset,float cycTime)
{
    ARMS_MASTER_THREAD_INFO* pMaster = pTmagic->pMaster;
    int ai = pTmagic->mArmsId;
    ARM_PARAM *param = &pMaster->mAparams[ai];
    CHASSIS_MASTER_THREAD_INFO *pTChassis = pMaster->pTChassis;

    pTmagic->magic_follow_runcnt++;
    //对偏差倒数进行滤波
    FILTERING_PARAM * pf = (FILTERING_PARAM *)&pMaster->mF_Inf[ai];
    #if 0
    //当前主辅轴位置偏差
    float x_intpos_now_error = pMaster->xMIntPos[ai] - pMaster->xSIntPos[ai];
    if(fabs(x_intpos_now_error-pTmagic->x_intpos_start_error) > 0.0002)
    {
        printf("******X主副轴位置偏差过大:(%f %f) error:%f \n",pTmagic->x_intpos_start_error,x_intpos_now_error,x_intpos_now_error-pTmagic->x_intpos_start_error);
        pTmagic->mRunState = R_STOP;
        pTmagic->target_vel_i = 0;
        memset(pTmagic->target_vel_x_inters,0,sizeof(float)*100);
        memset(pTmagic->target_vel_y_inters,0,sizeof(float)*100);
        magic_stop_i(pMaster,ai);
        return ;
    }
    #endif
/*******************************XY向算法使用的参数*********************************/
    float rope_end_x, rope_end_y;
    float kp = ((double)pTmagic->magic_follow_runcnt)/200.0;
    float ki = ((double)pTmagic->magic_follow_runcnt)/5000.0;
    LIMIT_MAX(kp,param->follow_kp);
    LIMIT_MAX(ki,param->follow_ki);
    
    //计算磁栅尺转换到末端的数据,m单位。通过XY向系数换算到末端底部
    pTmagic->sikoX_end = x_offset*param->vision_coefficient_x;
    pTmagic->sikoY_end = y_offset*param->vision_coefficient_y;

    //死区限制
    rope_end_x = deadZone(pTmagic->sikoX_end,0.0002);
    rope_end_y = deadZone(pTmagic->sikoY_end,0.0002);

    
    float d_rope_x = (rope_end_x-pTmagic->rope_end_last_x)/cycTime; 
    float d_rope_y = (rope_end_y-pTmagic->rope_end_last_y)/cycTime; 

    d_rope_x = smooth_filter(pf->d_rope_x_arr,d_rope_x,10);
    d_rope_y = smooth_filter(pf->d_rope_y_arr,d_rope_y,10);

/*****************************************舵轮算法*******************************************/
#if 0
    //cal_dd_L_dd_R_simple
    CHASSIS_MASTER_THREAD_INFO *pTChassis = pMaster->pTChassis;
    float D_R= pMaster->yPos[ai];               //D_R 卸载装置的伸长量
    float d_D_R=pTmagic->speed_y;          //d_D_R  卸载装置伸长量的导数（也就是伸长速度）
    float L_R=pMaster->xMIntPos[ai];            //       L_R 卸载装置在本体坐标x轴上的坐标
    float d_L_R=pTmagic->speed_x;          //
    float dd_xo1_R = pTChassis->dd_xo1_R;
    float dd_yo1_R = pTChassis->dd_yo1_R; 
    float fai_R_1 =pTChassis->fai_R_1;
    float d_fai_R1 =pTChassis->d_fai_R1;
    float dd_fai_R1 =pTChassis->dd_fai_R1;
    float fai_R_2 =pTChassis->fai_R_2;

    double mid1=dd_xe_R-dd_xo1_R;
    mid1=mid1+L_R*d_fai_R1*d_fai_R1;
    mid1=mid1+2*d_D_R*d_fai_R1*sin(fai_R_2)+D_R*d_fai_R1*d_fai_R1*cos(fai_R_2);
    mid1=mid1+D_R*dd_fai_R1*sin(fai_R_2);
    double mid2=dd_ye_R-dd_yo1_R-2*d_L_R*d_fai_R1;
    mid2=mid2-L_R*dd_fai_R1;
    mid2=mid2-2*d_D_R*d_fai_R1*cos(fai_R_2)+D_R*d_fai_R1*d_fai_R1*sin(fai_R_2);
    mid2=mid2-D_R*dd_fai_R1*cos(fai_R_2);
    //计算出吊杆控制加速度
    double dd_L_R_return=(mid1*sin(fai_R_2)-mid2*cos(fai_R_2))/sin(fai_R_2);  
    double dd_D_R_return=mid2/sin(fai_R_2);
#endif

/*******************************************X向速度命令************************************************/
    //基座的加速度去掉
    if(pTChassis->enable_flag == 0)
    {
        pTChassis->dd_x01_real = 0;
        pTChassis->dd_y01_real = 0;
    }

    float x_m_v =pTmagic->cmd_last_xv + kp*(rope_end_x - pTmagic->rope_end_last_x ) + ki*rope_end_x - pTChassis->dd_x01_real*cycTime;
    float yv    =pTmagic->cmd_last_yv + kp*(rope_end_y - pTmagic->rope_end_last_y ) + ki*rope_end_y - pTChassis->dd_y01_real*cycTime;

    //存储当前命令速度值
    pTmagic->cmd_last_xv = x_m_v;
    pTmagic->cmd_last_yv = yv;

    //存储末端绳索值
    pTmagic->rope_end_last_x = rope_end_x;
    pTmagic->rope_end_last_y = rope_end_y;

    //算法限速
    LIMIT_MAX(x_m_v, fabs(param->magic_max_x_speed));
    LIMIT_MIN(x_m_v,-fabs(param->magic_max_x_speed));
    //速度限值
    LIMIT_MAX(yv, fabs(param->magic_max_y_speed));
    LIMIT_MIN(yv,-fabs(param->magic_max_y_speed));

    x_m_v = smooth_filter(pMaster->mF_Inf[ai].m_x_cmd_arr,x_m_v,pMaster->mF_Inf[ai].F_xy_cmd_n);
    yv = smooth_filter(pMaster->mF_Inf[ai].m_y_cmd_arr,yv,pMaster->mF_Inf[ai].F_xy_cmd_n);

    //低通滤波器yn = yn-1 + a(xn-yn-1);
    float fir_a = 1;
    float fir_xv = pTmagic->cmd_fir_last_xv + fir_a*(x_m_v - pTmagic->cmd_fir_last_xv);
    pTmagic->cmd_fir_last_xv = fir_xv;

    //算法计算得到的速度值
    x_m_v = fir_xv;

    //根据相机检测偏移做速度前馈，
    float drope_kp = 0.001;
    x_m_v += (d_rope_x*drope_kp);
    yv    += (d_rope_y*drope_kp);

    //X向的主副轴偏置力矩补偿
    float quetor_error = ((pMaster->xMIntPos[ai]-pTmagic->xStartPos)-(pMaster->xSIntPos[ai]-pTmagic->xSStartPos))*1000.0;
    pMaster->motors[ai][2].torque_offset =  1000*quetor_error;
    pMaster->motors[ai][1].torque_offset =  1000*quetor_error;
    if(x_m_v*1000.0 == 0.0)
    {
        pMaster->motors[ai][2].torque_offset = 0;
        pMaster->motors[ai][1].torque_offset = 0;
    }

    //设置力矩Y向偏差
    if(yv*1000.0 >0)
        pMaster->motors[ai][0].torque_offset =  100 ;
    else if(yv*1000.0<0)
        pMaster->motors[ai][0].torque_offset = -100 ;
    else
        pMaster->motors[ai][0].torque_offset = 0;

/*******************************************XY向速度输出************************************************/
    //速度细分到每个刷新周期
    float starVx = pTmagic->target_vel_x_inters[9];
    float starVy = pTmagic->target_vel_y_inters[9];
    for (int i = 0; i < 10; i++)
    {
        pTmagic->target_vel_x_inters[i] = starVx + (i+1)*(x_m_v-starVx)/10;
        pTmagic->target_vel_y_inters[i] = starVy + (i+1)*(yv-starVy)/10;
    }
    pTmagic->target_vel_i = 0;
}

static void magic_vertical_weight(MAGIC_THREAD_INFO *pTmagic,float cycTime)
{
    static uint32_t rntCnt = 0;

    ARMS_MASTER_THREAD_INFO* pMaster = pTmagic->pMaster;
    int armI = pTmagic->mArmsId;
    ARM_PARAM *param = &pMaster->mAparams[armI];

    //气浮套参数
    float aaS = 0.0048, V=0.06,aR = 8314.0/29.0,K = 0.25,T = 293,q11= 0.00183;

    float vb   = pMaster->sikoZ_V[armI];
    float dvb  = pMaster->sikoZ_A[armI];
    float abs_stress = pMaster->abs_stress[armI];
    float weight_ebsl = param->floating_weight_ebsl;
    float weight_wn = param->floating_weight_wn;
    //当前绝压和 卸载的目标压力差
    float P1 = abs_stress - param->floating_air_target_press; 
    float dP1 = (P1 - pTmagic->floating_last_p1)/cycTime;

    float airKd = (vb*aaS - 2*weight_ebsl*weight_wn*V)/(q11*K*aR*T);
    float airKp = (dvb*aaS - 2*weight_ebsl*aaS*vb - weight_wn*weight_wn*V)/(q11*K*aR*T);

    float mCmdDu = (airKp*P1 + airKd*dP1);
    
    float median_cmd = mCmdDu*cycTime + pTmagic->last_cmd_medium;

    if(median_cmd > fabsf(param->median_ratio_max_u))
        median_cmd = fabsf(param->median_ratio_max_u);
    else if(median_cmd < -fabsf(param->median_ratio_max_u))
        median_cmd = -fabsf(param->median_ratio_max_u);

    pTmagic->last_cmd_medium = median_cmd;
    pTmagic->floating_last_p1 = P1;

    //if (pTmagic->enable_flag == 1)
    pMaster->proportion_cmd[armI][3] = median_cmd;

    rntCnt++;
}
//垂直卸载算法
static void magic_vertical(MAGIC_THREAD_INFO *pTmagic,float cycTime)
{
    ARMS_MASTER_THREAD_INFO* pMaster = pTmagic->pMaster;
    int armI = pTmagic->mArmsId;
    ARM_PARAM *param = &pMaster->mAparams[armI];
    /*************************************微重力算法中气缸控制*************************************/
    //气浮套参数
    float aaS = 0.0048, V=0.06,aR = 8314.0/29.0,K = 0.25,T = 293,q11= 0.00183;

    float vb   = pMaster->sikoZ_V[armI];
    float dvb  = pMaster->sikoZ_A[armI];
    float abs_stress = pMaster->abs_stress[armI];
    float air_ebsl = param->floating_air_ebsl;
    float air_wn = param->floating_air_wn;
    //当前绝压和 卸载的目标压力差
    //float P1 = abs_stress - pTmagic->cmd_target_abs; 
    float P1 = abs_stress - param->floating_air_target_press; 
    float dP1 = (P1 - pTmagic->floating_last_p1)/cycTime;

    float airKd = (vb*aaS - 2*air_ebsl*air_wn*V)/(q11*K*aR*T);
    float airKp = (dvb*aaS - 2*air_ebsl*aaS*vb - air_wn*air_wn*V)/(q11*K*aR*T);

    float mCmdDu = (airKp*P1 + airKd*dP1);
    
    float median_cmd = mCmdDu*cycTime + pTmagic->last_cmd_medium;

    if(median_cmd > fabsf(param->median_ratio_max_u))
        median_cmd = fabsf(param->median_ratio_max_u);
    else if(median_cmd < -fabsf(param->median_ratio_max_u))
        median_cmd = -fabsf(param->median_ratio_max_u);

    pTmagic->last_cmd_medium = median_cmd;
    pTmagic->floating_last_p1 = P1;

    pMaster->proportion_cmd[armI][3] = median_cmd;

    /*************************************end微重力算法中气缸控制*************************************/

    /*************************************微重力算法中卷扬机控制*************************************/
    //airTMg：气缸质量，mAirAxisR：卷扬桶半径
    /*
    float airTco = 0.8,airTg=9.8,airTL=1.0,airTMg=17,mAirAxisR=0.06;
    float airTMm = pTmagic->cmd_target_abs*0.0042/9.8 - 3.82;
    float airTK  = 2*airTco*sqrt((airTg*airTMm)/(airTL*airTMg));
    */

    float airTL=5.8,mAirAxisR=0.06;

    float zpos = pMaster->sikoXYZ[pTmagic->mArmsId][2];

    float Yg = zpos - param->cylinder_median_pos;
    float dYg = pMaster->sikoZ_V[armI];
    float ddYg = pMaster->sikoZ_A[armI];

    //当前绳索的实际加速度,天津算法未使用
    float ddLastLcontrl = 0;// = pddL
    float w_ebsl = param->floating_windlass_ebsl;
    float w_wn = param->floating_windlass_wn;
    float mCmdDDL = (2*w_ebsl*w_wn*dYg + w_wn*w_wn*Yg + ddYg*0.0)*(Yg+airTL)/airTL + ddLastLcontrl;

    LIMIT_MAX(mCmdDDL, 0.6);
    LIMIT_MIN(mCmdDDL,-0.6);

    //计算当前要控制的电机转速
    float zv = (mCmdDDL/mAirAxisR)*cycTime + pTmagic->cmd_z_Last_speed;
    //float zv = (mCmdDDL/mAirAxisR)*cycTime + pMaster->zSpeed[armI]/1000.0;

    //限速
    LIMIT_MAX(zv, fabs(param->magic_max_z_speed));
    LIMIT_MIN(zv,-fabs(param->magic_max_z_speed));
   
    //控制命令滤波
    zv = smooth_filter(pMaster->mF_Inf[armI].m_z_cmd_arr,zv,pMaster->mF_Inf[armI].F_z_cmd_n);

    pTmagic->cmd_z_Last_speed = zv;
    //随动算法命令输出

    //垂直限位处理  6.16  -zjy
    if (Is_Zlimit_Open) {
        if (Is_Zout_limit[pTmagic->mArmsId][1]) {
            if (zv < 0)
            {
                zv = 0;
            }
        } 
        else if (Is_Zout_limit[pTmagic->mArmsId][0])
        {
            if (zv > 0)
            {
                zv = 0;
            }
        }
    }

    armsSetSpeedZ(pMaster,pTmagic->mArmsId,zv*1000.0);
}

////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @return Descriptions
******************************************************************************/
void* magic_threadEntry(void* pModule)
{
    MAGIC_THREAD_INFO *pTmagic = (MAGIC_THREAD_INFO *) pModule;
    ARMS_MASTER_THREAD_INFO* pMaster = (ARMS_MASTER_THREAD_INFO*)pTmagic->pMaster;
    int aid = pTmagic->mArmsId;

    unsigned int nDelay = 1000;        /* usec */
    double cycT = (double)nDelay;
    //timer 定时器
    struct timespec  next, interval;
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    clock_gettime(CLOCK_MONOTONIC, &next);

    int ret = 0;
    uint32_t runCnt = 0;
    pTmagic->mRunState = R_NULL;

    while(pTmagic->mWorking)
    {
        next.tv_sec  += interval.tv_sec;
        next.tv_nsec += interval.tv_nsec;
        tsnorm(&next);
        if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)))
        {
            if (ret != EINTR)
                printf("clock_nanosleep failed. errno:");
            continue;
        }


        //数据转换及滤波，0.005s
        if(runCnt%5 == 0)
        {
            //转换及滤波
            if(pMaster->slaves_inf[aid].childSlaveSize>4)
            {
                //机械臂单元内传感器数据转换成国际单位
                sensors_read_i_to_f(pMaster,aid);
                
                //计算速度误差等信息
                magic_calcu_others(pTmagic,aid);

                //速度等信息滤波
                filtering(pMaster,aid);
            }
        }

        //执行算法,10ms周期
        cycT = 0.01;
        if(runCnt%10 == 0)
        {
            switch (pTmagic->mRunState)
            {
                case R_STOP:{
                    //发送执行机构停止，然后跳转到NULL 状态
                    pTmagic->mRunState = R_NULL;
                    break;
                }
                case R_VERTICAL_WEIGHT:{
                    //垂直称重算法。 状态
                    magic_vertical_weight(pTmagic,(float)cycT);
                    break;
                }
                case R_FOLLOW:
                {
                    //发送执行机构停止，然后跳转到NULL 状态
                    //float x_offset = pMaster->sikoXYZ[aid][0]/(0.45/4.0);
                    //float y_offset = pMaster->sikoXYZ[aid][1]/(0.45/4.0);
                    magic_follow(pTmagic,pMaster->sikoXYZ[aid][0],pMaster->sikoXYZ[aid][1],(float)cycT);
                    break;
                }
                case R_VERTICAL:{
                    //发送执行机构停止，然后跳转到NULL 状态
                    magic_vertical(pTmagic,(float)cycT);
                    break;
                }
                case R_FOLLOW_VERTICAL:{
                    //发送执行机构停止，然后跳转到NULL 状态
                    //float x_offset = pMaster->sikoXYZ[aid][0]/(0.45/4.0);
                    //float y_offset = pMaster->sikoXYZ[aid][1]/(0.45/4.0);
                    magic_follow(pTmagic,pMaster->sikoXYZ[aid][0],pMaster->sikoXYZ[aid][1],(float)cycT);
                    magic_vertical(pTmagic,(float)cycT);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
 
        //1ms发送速度
        if(pTmagic->mRunState == R_FOLLOW || pTmagic->mRunState == R_FOLLOW_VERTICAL)
        {
            if(pTmagic->target_vel_i>=10)
                pTmagic->target_vel_i = 9;

            float x_m_v = pTmagic->target_vel_x_inters[pTmagic->target_vel_i];
            float yv = pTmagic->target_vel_y_inters[pTmagic->target_vel_i];
            armsSetSpeedXY(pMaster,pTmagic->mArmsId, x_m_v*1000.0, x_m_v*1000.0, yv*1000.0);
            pTmagic->target_vel_i++;
        }

        //循环算法
        struct timespec  timeNow;
        clock_gettime(CLOCK_MONOTONIC, &timeNow);
        //printf("master%d %s,time:(%d %d) %lu\n\r",pTModule->mthreadMasterId,pTModule->mThreadName , timeNow.tv_sec,timeNow.tv_nsec/1000,pthread_self());
        if(runCnt % 10 == 0)
        {
            //printf("%s %f %f %f %d\n",pTmagic->mThreadName,pTmagic->cylinder_median_pos,pTmagic->floating_weight_kp,pTmagic->floating_weight_kd,pMaster->mFilterInf[pTmagic->mArmsId].F_SpeedTimes);
            //if(aid == 0)
                //printf("%.1f \n",pMaster->tension[0]);
        }
        runCnt++;
    }
    sleep(1);
    moduleEndUp(pTmagic);
}

