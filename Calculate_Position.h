/*****************************************************************************************************************************
******************************************************************************************************************************
姿勢計算に必要な関数をまとめたヘッダファイル



******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <stdio.h>
#include <math.h>
#include "Parameter.h"
#include "struct.h"
/************************************************************
					マクロの定義
************************************************************/
//寸法




/************************************************************
					構造体
************************************************************/
//enum{
//	Stand=0, SwingR, SwingL,
//	HipBase=0, AnkleBase,
//};

//インピーダンス制御による目標加速度・速度・位置

//struct motion_data{
//	double pos[2];						// current position
//	double vel[2];						// current velocity
//	double acc[2];						// current accelaretion
//
//	double prePos[2];						// current position
//	double preVel[2];						// current velocity
//	double preAcc[2];						// current accelaretion
//};
//
//struct leg_state{
//	int support;
//	int swing;
//};


////身体各部の座標(R,L X,Y)
//struct body_posi{
//	double Hip[2][2];
//	double Knee[2][2];
//	double Ankle[2][2];
//	double Sole[2][2];
//	double Toe[2][2];
//	double Heel[2][2];
//	double Urg[2][2];
//	
//	//身体各部位の重心の状態(R,L)
//	struct motion_data UppThigh[2];		//上腿の位置,速度,加速度
//	struct motion_data LowThigh[2];		//下腿の位置,速度,加速度
//	struct motion_data Foot[2];			//足の位置,速度,加速度
//
//	struct motion_data Body;				//胴体の重心座標(X,Y)
//
//	double CoG[2];							//Center of Gravity (X,Y)
//	double XZmp;							//ZMP(X方向のみ、Y=0:床面上）
//};





/************************************************************
					プロトタイプの宣言
************************************************************/
void Calculate_Position(double enc_data[2][3], struct leg_state *Leg, struct body_posi *body_posi, float *Yaw1, float *Roll, float *Pitch2, float *Yaw2); //simi MPU removed float *Pitch1,
void LegState( int state, struct leg_state *Leg );
double Calculation_Body_Orientation( double enc_data[3] );
void Calculation_Swing_Angle(double enc_data[2][3], struct leg_state *Leg, double theta[3], double body_ang); //, double body_ang
void Calculation_Body_Part_Pos(int mode, double enc_data[2][3], struct leg_state *Leg,	struct body_posi *body_posi, float *Yaw1, float *Roll, float *Pitch2, float *Yaw2); //simi MPU removed float *Pitch1,