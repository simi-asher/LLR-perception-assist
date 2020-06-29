/*****************************************************************************************************************************
******************************************************************************************************************************
認知アシストに必要な関数をまとめたソースファイル
urg.cppで計算した障害物までの距離，高さと
calculate.cppで計算した足先位置から認知アシストに必要な力を計測


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <stdio.h>
#include "urg.h"
#include "Calculate_Position.h"
#include "Parameter.h"


/************************************************************
					マクロの定義
************************************************************/

#define DETECT_MOTION_THRESHOLD (0.1)

#define VIRWALL_DEPTH (0.2)		//仮想壁の奥行き
#define OVER_HEIGHT	  (0.03)	//段差より上の高さ
#define TARGET_HEIGHT (0.1)		//目標値の設定における仮想壁の傾きを上方に平行移動させる量


/************************************************************
					構造体
************************************************************/
//enum{
//	FOOT_MOTION=0, GAIT_MOTION, UPSTAIR_MOTION,
//	CONST_CTRL=0, P_CTRL, PD_CTRL,
//};
//
//struct VirWall_data{
//	double grad;
//	double intr;
//	double target;
//	double depth;
//	double force[2];
//};
//
////認知アシストに必要なデータを格納
//struct Per_data{
//	double distance;
//	double height;
//	double depth;
//	int    motion;
//	bool   flag;
//
//	struct VirWall_data virWall;
//};




/************************************************************
					プロトタイプの宣言
************************************************************/
//メインループで呼び出す関数
void Calculate_Perception_Assist(struct Obstacle_data *obstacleData, struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData, int judge);
void FromGroundToOrigin(struct Obstacle_data *obstacleData, struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData);
void DetectMotion(struct body_posi *jointPos, struct Per_data *perData);
void SetVirtualWall(struct Per_data *perData, int judge);
void DetectDanger(struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData, int judge);
void PerceptionForce(struct body_posi *jointPos, const struct leg_state *Leg, int mode, struct Per_data *perData, double x_dist, int judge);
