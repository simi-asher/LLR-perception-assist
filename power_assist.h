/*****************************************************************************************************************************
******************************************************************************************************************************
�p���[�A�V�X�g�̌v�Z�ɕK�v�Ȋ֐����܂Ƃ߂��w�b�_�t�@�C��


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <math.h>
#include "fuzzy.h"
#include "Parameter.h"
#include <iostream>
/************************************************************
					�}�N���̒�`
************************************************************/

//�̓Z���T�x�[�X�CEMG�x�[�X�̃p���[�A�V�X�g���s�����i�g�p���Ȃ��ꍇ�̓R�����g�A�E�g�j
#define USE_EMG
//#define USE_FORCESENSOR

//�̓Z���T�x�[�X�̃p���[�A�V�X�g�ɂ�����P����̃Q�C��
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


#define COEFF_COMPENSATION_FORCE_RH (3.515)		//�̓Z���T�̕␳���i���t������Z���T��ς�����␳�������Ȃ����K�v��j[V]��[kg]
#define COEFF_COMPENSATION_FORCE_RK (2.052)		//�̓Z���T�̕␳�� [V]��[kg]
#define COEFF_COMPENSATION_FORCE_RA (0)			//�̓Z���T�̕␳�� [V]��[kg] �܂��␳���ĂȂ���I��
#define COEFF_COMPENSATION_FORCE_LH (3.435)		//�̓Z���T�̕␳�� [V]��[kg]
#define COEFF_COMPENSATION_FORCE_LK (1.273)		//�̓Z���T�̕␳�� [V]��[kg] �܂��␳���ĂȂ���I
#define COEFF_COMPENSATION_FORCE_LA (0)			//�̓Z���T�̕␳�� [V]��[kg] �܂��␳���ĂȂ���I



/************************************************************
					�\����
************************************************************/



/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
void Calculate_Power_Assist(const int learnFlag, _weight *weight_fuzzy, double encData[2][3], double weight_emg[3][EMG_CH], double emg[2][EMG_CH], double forceData[2][3][3], double torq[2][3]);
void ForceSensorBaseAssist(double forceData[2][3][3], double torq[2][3]);
void Compensation_ForceSensor(double forceData[2][3][3]);
void EMG_BaseAssist(double weight[3][EMG_CH], double emg_rms[2][EMG_CH], double torq[2][3]);
void Sum_ForceSensorAndEMG_BaseAssist(const double weight[2], double torq_forceSensor[2][3], double torq_emg[2][3], double torq[2][3]);

inline double Weight_ForceSensorAndEMG(double emg);
inline double P_Control(double gain, double currentPos, double targetPos);
double PD_Control(double gain[2], double current[2], double target[2]);

