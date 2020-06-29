/*****************************************************************************************************************************
******************************************************************************************************************************
�p���v�Z�ɕK�v�Ȋ֐����܂Ƃ߂��w�b�_�t�@�C��



******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <stdio.h>
#include <math.h>
#include "Parameter.h"
#include "struct.h"
/************************************************************
					�}�N���̒�`
************************************************************/
//���@




/************************************************************
					�\����
************************************************************/
//enum{
//	Stand=0, SwingR, SwingL,
//	HipBase=0, AnkleBase,
//};

//�C���s�[�_���X����ɂ��ڕW�����x�E���x�E�ʒu

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


////�g�̊e���̍��W(R,L X,Y)
//struct body_posi{
//	double Hip[2][2];
//	double Knee[2][2];
//	double Ankle[2][2];
//	double Sole[2][2];
//	double Toe[2][2];
//	double Heel[2][2];
//	double Urg[2][2];
//	
//	//�g�̊e���ʂ̏d�S�̏��(R,L)
//	struct motion_data UppThigh[2];		//��ڂ̈ʒu,���x,�����x
//	struct motion_data LowThigh[2];		//���ڂ̈ʒu,���x,�����x
//	struct motion_data Foot[2];			//���̈ʒu,���x,�����x
//
//	struct motion_data Body;				//���̂̏d�S���W(X,Y)
//
//	double CoG[2];							//Center of Gravity (X,Y)
//	double XZmp;							//ZMP(X�����̂݁AY=0:���ʏ�j
//};





/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
void Calculate_Position(double enc_data[2][3], struct leg_state *Leg, struct body_posi *body_posi, float *Yaw1, float *Roll, float *Pitch2, float *Yaw2); //simi MPU removed float *Pitch1,
void LegState( int state, struct leg_state *Leg );
double Calculation_Body_Orientation( double enc_data[3] );
void Calculation_Swing_Angle(double enc_data[2][3], struct leg_state *Leg, double theta[3], double body_ang); //, double body_ang
void Calculation_Body_Part_Pos(int mode, double enc_data[2][3], struct leg_state *Leg,	struct body_posi *body_posi, float *Yaw1, float *Roll, float *Pitch2, float *Yaw2); //simi MPU removed float *Pitch1,