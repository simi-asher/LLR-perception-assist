/*****************************************************************************************************************************
******************************************************************************************************************************
パワーアシストの計算に必要な関数をまとめたヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <math.h>
#include "fuzzy.h"
#include "Parameter.h"
#include <iostream>
/************************************************************
					マクロの定義
************************************************************/

//力センサベース，EMGベースのパワーアシストを行うか（使用しない場合はコメントアウト）
#define USE_EMG
//#define USE_FORCESENSOR

//力センサベースのパワーアシストにおけるP制御のゲイン
//#define GAIN_P_H (-1300)
//#define GAIN_P_K (20)
//#define GAIN_P_A (1)
//
//#define GAIN_D_H (-40)
//#define GAIN_D_K (0.1)
//#define GAIN_D_A (1)

#define GAIN_P_H (500)
#define GAIN_P_K (2)
#define GAIN_P_A (1)


#define COEFF_COMPENSATION_FORCE_RH (3.515)		//力センサの補正式（取り付け方やセンサを変えたら補正式を見なおす必要り）[V]→[kg]
#define COEFF_COMPENSATION_FORCE_RK (2.052)		//力センサの補正式 [V]→[kg]
#define COEFF_COMPENSATION_FORCE_RA (0)			//力センサの補正式 [V]→[kg] まだ補正してないよ！あ
#define COEFF_COMPENSATION_FORCE_LH (3.435)		//力センサの補正式 [V]→[kg]
#define COEFF_COMPENSATION_FORCE_LK (1.273)		//力センサの補正式 [V]→[kg] まだ補正してないよ！
#define COEFF_COMPENSATION_FORCE_LA (0)			//力センサの補正式 [V]→[kg] まだ補正してないよ！



/************************************************************
					構造体
************************************************************/



/************************************************************
					プロトタイプの宣言
************************************************************/
void Calculate_Power_Assist(const int learnFlag, _weight *weight_fuzzy, double encData[2][3], double weight_emg[3][EMG_CH], double emg[2][EMG_CH], double forceData[2][3][3], double torq[2][3]);
void ForceSensorBaseAssist(double forceData[2][3][3], double torq[2][3]);
void Compensation_ForceSensor(double forceData[2][3][3]);
void EMG_BaseAssist(double weight[3][EMG_CH], double emg_rms[2][EMG_CH], double torq[2][3]);
void Sum_ForceSensorAndEMG_BaseAssist(const double weight[2], double torq_forceSensor[2][3], double torq_emg[2][3], double torq[2][3]);

inline double Weight_ForceSensorAndEMG(double emg);
inline double P_Control(double gain, double currentPos, double targetPos);
double PD_Control(double gain[2], double current[2], double target[2]);

