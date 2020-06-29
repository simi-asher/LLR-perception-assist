/*****************************************************************************************************************************
******************************************************************************************************************************
描画などのマクロ定義のためのヘッダファイル



******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

//#include <stdlib.h>
//#include <Windows.h>
//#include <gl/glut.h>
//#include <AntTweakBar.h>
//#include "Parameter.h"

/************************************************************
					マクロの定義
************************************************************/



/************************************************************
					構造体
************************************************************/
//typedef enum{ MODE_3D=0, MODE_2D, MODE_URG}DrawMode;
//typedef enum{ NULL_DRAW=0, ANGLE_DRAW=1, FOOT_DRAW, TORQ_OUT_DRAW, TORQ_POW_DRAW, TORQ_PER_DRAW}DrawGraph;
//typedef enum{ NULL_GRAPH=0, ANGLE_GRAPH=1, FOOT_GRAPH, TORQ_OUT_GRAPH, TORQ_POW_GRAPH, TORQ_PER_GRAPH}ChangeGraph;
//typedef enum{ DISTANCE=0,HEIGHT,DEPTH,VIR_GRAD,VIR_INTR,VIR_DEPTH}BumpMode;

////絶対座標系の操作に関するツールバー
//typedef struct{
//	float Zoom;				// Shapes scale
//	float Rotation[4];		// Shape orientation (stored as a quaternion)
//	int AutoRotate;			// Auto rotate
//	int RotateTime;
//	float RotateStart[4];
//	float MatAmbient[4];	// Shapes material
//	float MatDiffuse[4];
//	float LightMultiplier;	// Light parameter
//	float LightDirection[3];
//
//	float Color[3][4];
//
//}worldParam;
//
////描画モードの変更に関するツールバー
//typedef struct{
//	DrawMode    drawMode;
//	DrawGraph   drawGraph;
//	ChangeGraph changeGraph[2];
//
//}operationParam;
//
////値の表示に関するツールバー
//typedef struct{
//	bool   recordFlag;
//	double Foot[2];
//	double ForceSensor[2][3];
//	double ZmpX;
//	int    time;
//	double Freq;
//	double DrawFreq;
//	double Bump[6];
//	double Torq[2][3];
//
//}valueParam;
//
////ツールバーの設定に関する構造体をまとめた構造体
//typedef struct{
//	worldParam	   wldParam;
//	operationParam opeParam;
//	valueParam	   valParam;
//}tweakParam;
//
////各ツールバーのハンドルをまとめた構造体
//typedef struct{
//	TwBar *worldBar;
//	TwBar *valueBar;
//	TwBar *operationBar;
//	TwBar *graphUpBar;
//	TwBar *graphDownBar;
//}tweakHandle;
//
////描画用の関節のための構造体
//typedef struct{
//	double Hip[2][3];
//	double Knee[2][3];
//	double Ankle[2][3];
//	double Toe[2][3];
//	double Heel[2][3];
//	double Body[3];
//}draw_joint;
//
//struct drawURG_struct{
//	int    dataSize;
//	int    dataBumpSize;
//	double length[2];
//	double position[2];
//	double smooth[2];
//};
//
//
////読み込んだデータを格納するための構造体
//typedef struct{
//	valueParam *element;
//	draw_joint *joint;
//	draw_joint drawJoint;
//	double     bump[6];
//	int        length;
//	int        time;
//	double     freq;
//	drawURG_struct *urg;
//}data_struct;


/************************************************************
					プロトタイプの宣言
************************************************************/
