/*****************************************************************************************************************************
******************************************************************************************************************************
トルク計算に必要な関数をまとめたヘッダファイル



******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Parameter.h"

/************************************************************
					マクロの定義
************************************************************/

//*** ファジィニューロ調整器関係 ***//
//#define RULE 32								// number of rules (4×4×2)現在てきとう
#define EPSILON_FUZZY 0.0000001
#define L_RATE_HIP 1.56e-5					// learning rate for parameters of hip joint
#define L_RATE_KNE 1.56e-5					// learning rate for parameters of knee joint
#define L_RATE_ANK 1.56e-4					// learning rate for parameters of ankle joint




/************************************************************
					構造体
************************************************************/
//typedef enum { PO_B = 0, PO_S, ZERO, NE_S } fuzzy_set;

//typedef struct{
//	double dev[3][4];				// Deviation value for EMG
//	double cen[3][4];				// Center value for EMG
//	double r_d[3][EMG_CH][RULE];	// Weight of between Rule and Defuzzifier layer
//	double ori[3][EMG_CH];			// initial weight matrix
//}_weight;



/************************************************************
					プロトタイプの宣言
************************************************************/
int FuzzyNeuro(const int learnFlag, _weight *weight, double ang[3], double rms[EMG_CH], double force[3][3], double weight_p[3][EMG_CH]);
void LearningWeight(double d_layer_sum, double r_layer[RULE], double rms[EMG_CH], double error_force[3], _weight *weight);
void Error_Force( double forceData[3][3], double error_force[3] );
void Read_Weight_Parameters( _weight *weight );
void Write_Weight_Parameters( _weight *weight );

inline double Sig_Func( double theta, double w_o, double w_i );
inline double Gau_Func( double theta, double w_o, double w_i );