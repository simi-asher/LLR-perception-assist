/*****************************************************************************************************************************
******************************************************************************************************************************

ニューラルネットに必要な関数をまとめたヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#pragma once

#include <stdio.h>
#include <math.h>
#include <stdlib.h>


/************************************************************
					マクロの定義
************************************************************/
#define LAYER_NUM (5)  //layerの層数(入力層＆出力層を含む) //change


#define ETA      (0.001)	//学習係数
#define EPSILON  (0.1)		//終了条件
#define sigmoid  (0)		
#define tanh  (1)		
#define relu  (2)		
#define leakly_relu  (3)		
#define softmax  (4)		
#define linear  (5)		



/************************************************************
					構造体
************************************************************/
enum{
	SIGMOID=0,GAUSSIAN,RELU,
};

typedef struct{
	
	double **layer;	// ウェイト値
	double *bias;  //バイアスからのウェイト値
	double **memo_layer;
	double *memo_bias;
	double **delta;
	double *delta_bias;

	
}_weights;


/************************************************************
					プロトタイプの宣言
************************************************************/
void NeuralNet(double *input, _weights *weights, int num, const int layer1_num, const int layer2_num, double *output, double *disOutput, int func);
void Test_NeuralNet(double *input, _weights *weights, int num, const int layer1_num, const int layer2_num, double *output, int func);
void CalculateError(const int OUTPUT_NUM, double *layer, double *teach, double *error);
void BP(int num, int layer_num, int *NEURON_NUM, double *error, double **disLayer, double **layer, _weights *weights,double ***keep0,double ****keep1);
void BP_weights_Change(int layer_num, int *NEURON_NUM,  _weights *weights,int q);
double Sigmoid_Func( double input );
double dif_Sig_Func(double input);
double Tanh_Func( double input );
double dif_Tanh_Func( double input );
double ReLU(double input, double slope);
double dif_ReLU(double input, double slope);
double Softmax(double *val,double layer_num, int j);
double dif_Softmax(double *val,double layer_num, int j);