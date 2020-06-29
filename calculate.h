/*****************************************************************************************************************************
******************************************************************************************************************************
���l�v�Z�ɕK�v�Ȋ֐����܂Ƃ߂��w�b�_�t�@�C��



******************************************************************************************************************************
*****************************************************************************************************************************/

#pragma once

#include <stdio.h>
#include <math.h>
#include "Calculate_Position.h"
#include "Parameter.h"
#include "struct.h"
#include "myFilter.h" //simi

/************************************************************
					�}�N���̒�`
************************************************************/
#define MASS_BODY  (PERSONAL_MASS * 0.63)		// ��̂̎��ʁi�S�̏d��63%�j
#define MASS_THIGH (PERSONAL_MASS * 0.105)		// ��ڂ̎��ʁi�S�̏d��10.5%�j
#define MASS_LOWER (PERSONAL_MASS * 0.05)		// ���ڂ̎��ʁi�S�̏d��5%�j
#define MASS_FOOT  (PERSONAL_MASS * 0.015)      // ���̎��ʁi�S�̏d��1.5%�j


//���{�b�g�̑��d�� ��21[kg]?
//���ځ{����4.8[kg]
#define MASS_R_BODY	 3.0						// robot's mass of body (����e�L�g�[�j
#define MASS_R_THIGH 4.0						// robot's mass of thigh part
#define MASS_R_LOWER 3.0						// robot's mass of lower thigh part
#define MASS_R_FOOT  2.0

#define SUM_BODY (MASS_BODY+MASS_R_BODY)		//���{�b�g�{������
#define SUM_THIGH (MASS_THIGH*MASS_R_THIGH)		//���{�b�g�{������
#define SUM_LOWER (MASS_LOWER*MASS_R_LOWER)
#define SUM_FOOT  (MASS_FOOT*MASS_R_FOOT)

#define MASS_H_ALL (MASS_BODY+(MASS_LOWER+MASS_THIGH+MASS_FOOT)*2)		//���{�b�g�̑�����[kg]
#define MASS_R_ALL (MASS_BODY+(MASS_R_LOWER+MASS_R_THIGH+MASS_R_FOOT)*2)	//�l�̑�����[kg]
#define MASS_ALL (MASS_H_ALL+MASS_R_ALL)

#define ROBOT_I1 (SUM_THIGH*LEN_H_K*LEN_H_K/3)		//���v�̍���ڊ������[�����g
#define ROBOT_I2 (SUM_LOWER*LEN_K_A*LEN_K_A/3)		//���v�̉��ڊ������[�����g
#define ROBOT_I3 (SUM_LOWER*LEN_A_T*LEN_A_T/3)		//���v�̑��������[�����g


#define ROBOT_I4 (MASS_THIGH*LEN_H_K*LEN_H_K/3)		//�l�̑�ڊ������[�����g
#define ROBOT_I5 (MASS_LOWER*LEN_K_A*LEN_K_A/3)		//�l�̉��ڊ������[�����g
#define ROBOT_I6 (MASS_LOWER*LEN_A_T*LEN_A_T/3)		//�l�̑��������[�����g


#define AGE (21)



/************************************************************
					�\����
************************************************************/
//struct jacob_data{
//	double trans[3][2];
//	double inv_trans[2][3];
//};


/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
void Calculate_Joint_VelAcc(const int count, double q[3][2][3]);
void Calculate_VelocityAcceleration(const int count, double pos[2], struct motion_data *motion, float buf[2][2][2][FILTER_LEN]);
double Calculation_ZMP( struct body_posi *body_posi);
int Calculation_Center_of_Gravity( struct body_posi *body_posi );
void Calculate_ZMP_COG(int t,struct body_posi *body_posi);
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state *Leg, struct jacob_data *jacobData);
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state* Leg, struct jacob_body * jacobData);
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state *Leg, struct jacob_hip *jh,struct jacob_body *jb);

//�璷�����p
void Calculate_UsingRedundancy2(double jointVal[3][2][3], double force[3], double error[3][3], struct jacob_data *J, struct dynamic_data *D, double torq[3], double torq2[3]);
void Calculate_NullSpace(double jointVal[3][2][3], struct jacob_data *jacobData, double nullSpaceVector[3]);

void Calculate_UsingRedundancy(double jointVal[3][2][3], double targetAcc[3], struct jacob_data *jacobData, double redundancyAcc[3]);
void NullSpace(struct jacob_data *jacobData);
void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]);
void Calculate_NullSpaceVector(double null_space[3][3], double eta[3], double vector[3]);
void Calculate_JointAcc_UsingRedundancy(double targetAcc[3], double jointVel[3], struct jacob_data *jacobData, double nullSpaceVector[3], double jointAcc[3]);

void EnergyMinimization(double enc_data[2][3], struct leg_state *Leg, struct jacob_data *jacobData, double torq[3], double force[3]);
int Cal_JointLimit(double enc[2][3], double theta_ELim[2][3], double theta_FLim[2][3]);
void CalculateEnergyMinimization(double enc_data[2][3], double eta[6][2]);
void NullSpace_TargetFunc(double enc_data[3], double eta[3]);
void NullSpaceVector(double force_null[3], struct jacob_data jacobData, double eta[3]);

//���͊w�v�Z�p
void Calculate_Dynamics(double angData[3][2][3], double targetAcc[3], struct transfer_data *transferData, struct dynamic_data *dynamicData, double torq[3]);
void SetDynamicsParam(double encData[2][3], struct transfer_data *transfer_data);
void Matrix44_Transfer(struct transfer_data *T);
void Matrix44_Diff(struct transfer_data *T);
void Matrix44_Diff2(struct transfer_data *T);
void Matrix_Mass(struct transfer_data *T, struct dynamic_data *D);
void Vector_H(double angData[3][2][3], struct transfer_data *T, struct dynamic_data *D);
void Vector_Gravity(double encData[2][3], struct transfer_data *T, struct dynamic_data *D);

//�s��v�Z�p�֐�
void Matrix_Unit(double I[3][3]);
void Matrix33_Trans(double M[3][3], double M_trans[3][3]);
double Matrix33_Det(double M[3][3]);
void Matrix22_Trans(double M[2][2], double M_trans[2][2]);
double Matrix22_Inv(double M[3][3], double M_inv[3][3]);
int Matrix33_Inv(double M[3][3], double M_inv[3][3]);
void Matrix33_Product(double A[3][3], double B[3][3], double M[3][3]);
int Matrix33_PseudoInverse(double M[3][3], double M_pse[3][3]);
void Matrix44_Trace(double M[4][4], double *M_trace);
void Matrix44_Trans(double M[4][4], double M_trans[4][4]);
void Matrix44_Product(double A[4][4], double B[4][4], double M[4][4]);
void Matrix66_Trans(double M[6][6], double M_trans[6][6]);

void Calculate_Gravity_Compensation(int LegState, struct torq_data *torq, double enc_ang[2][3]);