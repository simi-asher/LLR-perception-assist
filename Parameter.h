/*****************************************************************************************************************************
******************************************************************************************************************************
物理量などのマクロ定義のためのヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include "Calculate_Position.h"
/************************************************************
					マクロの定義
************************************************************/
#define ASSIST_RATE	(3.0)
#define MAX_STOCK (100000)

//時間関係
#define MAIN_TIME	(720)	//MAIN_TIME != PRE_TIMEにしましょう（同じだと実行時の時間の表示が変わります）+2 (If it is the same, the display of the execution time will change) +2
#define PRE_TIME	(3)
#define MAIN_TASK	(MAIN_TIME*SAMPLING_FREQ)
#define PRE_TASK	(PRE_TIME*SAMPLING_FREQ)

#define SAMPLING_FREQ (2000)	//imp 2000		//だいたいのサンプリング周波数[Hz] // prev 2000 because EMG needs 2000Hz
#define SAMPLING_TIME ( (double)1.0/SAMPLING_FREQ )

#define DRAW_FREQ (20)			//だいたいのサンプリング周波数[Hz]
#define DRAW_TIME ( (double)1.0/DRAW_FREQ )

//質量関係パラメータ
//#define PERSONAL_MASS 65.0			//装着者体重
#define PERSONAL_MASS 60 //simi	55 for yokomizo		//装着者体重

//EMG関係

//#define NUM_RMS	(int)(200)

//ファジィニューロ

//*** 物理定数 ***//
#define PI	(3.14159265358979)
#define GRAVITY  (9.80665)
#define DtoR ( PI/180.0 )		//degree→rad
#define RtoD ( 180.0/PI )		//rad→degree


//*** ロボット関係 ***//
//力センサの取り付け位置
#define LEN_H_HF (0.05)							// length between hip joint and hip force sensor
#define LEN_K_KF (0.05)							// length between knee joint and knee force sensor 
#define LEN_A_AF (0.05)							// length between Ancle joint and Anlcle force sensor



/************************************************************
					構造体
************************************************************/
//enum num_def{
//	R=0, L=1,						//R:right L:left
//	X=0, Y=1, Z=2,
//	OFF=0, ON=1,
//	H=0, K=1, A=2, U=3,				//H:hip K:knee: A:ankle U:URG
//};
//
//
//struct Record_data{
//	double enc[2][3];
//	double force[2][3];
//	double torq[2][3];
//	double torq_pow[2][3];
//	double torq_per[2][3];
//	double bump[6];
//	body_posi jointPos;
//};
//

const int GEAR_RATIO[3] = { 150,113,81 };

#define FILE_READ_OPEN( x, y )  if( ( x=fopen(y,"r") )==NULL ){ fprintf(stderr, "%s file not open\n",y); exit(1); }
#define FILE_WRITE_OPEN( x, y ) if( ( x=fopen(y,"w") )==NULL ){ fprintf(stderr, "%s file not open\n",y); exit(1); } //simi changed w to a for append




