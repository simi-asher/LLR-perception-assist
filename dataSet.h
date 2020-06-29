/*****************************************************************************************************************************
******************************************************************************************************************************
�f�[�^�Z�b�g�̂��߂̃w�b�_�t�@�C��


******************************************************************************************************************************
*****************************************************************************************************************************/

#pragma once


#include "DrawParam.h"
#include "struct.h"

/************************************************************
					�}�N���̒�`
************************************************************/

//�t�@�C���̏������݂Ɋւ���}�N�� (ON:��������	OFF�F�������܂Ȃ�)
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
					�\����-
************************************************************/



/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
int OutputFILEs(int w, int v, Record_data recordData[MAX_STOCK],int urg_flag);
void ReadData(data_struct *data);
void SetData_ToolBar(int t, data_struct data, tweakParam *barParam);
int Adjust_SamplingTime(int t, double *timeAvg);