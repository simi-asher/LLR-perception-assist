/*****************************************************************************************************************************
******************************************************************************************************************************
レーザーレンジファインダ(URG-04LX)使用に必要な関数をまとめたヘッダファイル
ソフトウェア：urg_library-1.2.0使用


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once


#include <stdio.h>
#include <math.h>

#pragma comment(lib,"laserLarngeFinder.lib")
#include <URG/c/urg_sensor.h>
#include <URG/c/urg_utils.h>
#include "struct.h"
/************************************************************
					マクロの定義
************************************************************/
//URGの初期設定用		

#define ScanTimes    1
#define SkipScan     0			//指定スキャン数おきに距離を出力する
#define URG_FirstDeg -100 //simi prev -120L,0R		//<=0 センサ正面より先に計測（正面0[deg]）
#define URG_FrontDeg 0 //simi prev 0
#define URG_LastDeg  100 //simi prev 0L,120R			//>=0 現在0以外の値を入れるとおかしくなると思われる(length_data_sizeをfront中心に変更する必要あり)
#define SkipStep     2			//x個のデータをまとめる（最小値を出力）(1～


//Split and Merge法のための定義
#define OBS_THRESHOLE   (0.1) //simi prev 0.1 0.07
#define SPLIT_THRESHOLD (0.03) //prev 0.05 0.03 //below 0.015 reduces the depth of obs
#define HEIGHT_THRESHOLD (0.15) //simi prev 0.12
#define WIDE_THRESHOLD (0.15)



/************************************************************
					構造体
************************************************************/
//enum{
//	URG_OFF=0,URG_ON,
//	NonFilter=0, AveragingFilter, GaussianFilter, DelayFilter,
//};
//
////urgの座標データを格納
//struct URG_data {
//	double rawX;
//	double rawY;
//	double x;
//	double y;
//};
//
////urgのデータを元に物体を検出する時に使用
//struct Obstacle_data {
//	double x;	//点のx座標
//	double y;	//点のy座標
//	double bumpX;
//	double bumpY;
//	double height;
//	double distance;
//	double depth;
//	double grad;	//直線の傾き
//	double intr;	//切片
//	double bump_p;
//	int size;
//	bool flag;
//};
//
////直線検出時に使用(Split and Merge法の処理の中)
//struct Line_data{
//	bool flag;
//	int data_num;
//	double x;
//	double y;
//	double grad;
//	double intr;
//};

/************************************************************
					プロトタイプの宣言
************************************************************/
//メインループで呼び出す関数
void Calculate_URG(urg_t *urg, long *length_data, int *length_data_size, int *urgTime, struct URG_data *urgData, struct Obstacle_data *obstacleData);

//URGの初期設定，計測，終了処理
void Init_URG(urg_t *urg, const long connect_baudrate, const char connect_device[]);	//初期化時に呼び出す
void Measure_URG(urg_t *urg, long *length_data, int *length_data_size, int *time);	//ループ内で呼び出す
void KeepData(struct URG_data *beforeData, const int length_data_size, struct URG_data *afterData);
void EndMeasure(urg_t *urg);  //終了時に呼び出す

//生データの処理
void ChangeZeroPoint(long *length_data, const int length_data_size);
void ChangeCoordinate(urg_t *urg, long *length_data, const int length_data_size, struct URG_data *urgData);
void DeleateOfNotMeasureData(const int length_data_size, struct URG_data *urgData);
void FromSensorToGround(const int flag, urg_t *urg, const int length_data_size, struct URG_data *urgData);
void Smoothing_Data(const int filterMode, const int length_data_size, struct URG_data *urgData);
void ChangeData(struct URG_data *urgData, urg_t *urg, const int length_data_size, struct Obstacle_data *obstacleData); //simi urg_t *urg,

//Split and Merge法
void DetectLine_SAM(struct Obstacle_data *obstacleData, int data_size); //Split and Merge法を使用する場合はこの関数を呼び出す
void UpdataValue(struct Line_data *line, struct Line_data *new_line, const int data_num);
void DetectObject(struct Obstacle_data *obstacleData, int data_size, int *FINISH_POINT);
void InitDetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, int *point_num, const int START_POINT,const int FINISH_POINT);
void DetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, struct Line_data *new_line, int *point_num, const int data_num);
void FittingLine(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num);
void CalculateIntersection(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num);
void BumpRecognition(struct Obstacle_data *obstacleData, int point_num);