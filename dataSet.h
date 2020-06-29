/*****************************************************************************************************************************
******************************************************************************************************************************
データセットのためのヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#pragma once


#include "DrawParam.h"
#include "struct.h"

/************************************************************
					マクロの定義
************************************************************/

//ファイルの書き込みに関するマクロ (ON:書き込む	OFF：書き込まない)
#define REC_FORCE_SENSOR (OFF)
#define REC_ENCODER      (OFF)//temp
#define REC_EMG			 (OFF)//temp
#define REC_TORQ		 (OFF) //simi //since power assist is off for exp
#define REC_POSITION	 (OFF) //temp
#define REC_VIR_WALL	 (OFF) //simi
#define REC_PVA			 (OFF) //simi //temp
#define REC_CUSTOM		 (ON) //simi
#define REC_NN			 (OFF) //simi neural network //change
/************************************************************
					構造体-
************************************************************/



/************************************************************
					プロトタイプの宣言
************************************************************/
int OutputFILEs(int w, int v, Record_data recordData[MAX_STOCK],int urg_flag);
void ReadData(data_struct *data);
void SetData_ToolBar(int t, data_struct data, tweakParam *barParam);
int Adjust_SamplingTime(int t, double *timeAvg);