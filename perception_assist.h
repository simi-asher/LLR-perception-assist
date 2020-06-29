/*****************************************************************************************************************************
******************************************************************************************************************************
�F�m�A�V�X�g�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��
urg.cpp�Ōv�Z������Q���܂ł̋����C������
calculate.cpp�Ōv�Z��������ʒu����F�m�A�V�X�g�ɕK�v�ȗ͂��v��


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <stdio.h>
#include "urg.h"
#include "Calculate_Position.h"
#include "Parameter.h"


/************************************************************
					�}�N���̒�`
************************************************************/

#define DETECT_MOTION_THRESHOLD (0.1)

#define VIRWALL_DEPTH (0.2)		//���z�ǂ̉��s��
#define OVER_HEIGHT	  (0.03)	//�i������̍���
#define TARGET_HEIGHT (0.1)		//�ڕW�l�̐ݒ�ɂ����鉼�z�ǂ̌X��������ɕ��s�ړ��������


/************************************************************
					�\����
************************************************************/
//enum{
//	FOOT_MOTION=0, GAIT_MOTION, UPSTAIR_MOTION,
//	CONST_CTRL=0, P_CTRL, PD_CTRL,
//};
//
//struct VirWall_data{
//	double grad;
//	double intr;
//	double target;
//	double depth;
//	double force[2];
//};
//
////�F�m�A�V�X�g�ɕK�v�ȃf�[�^���i�[
//struct Per_data{
//	double distance;
//	double height;
//	double depth;
//	int    motion;
//	bool   flag;
//
//	struct VirWall_data virWall;
//};




/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
//���C�����[�v�ŌĂяo���֐�
void Calculate_Perception_Assist(struct Obstacle_data *obstacleData, struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData, int judge);
void FromGroundToOrigin(struct Obstacle_data *obstacleData, struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData);
void DetectMotion(struct body_posi *jointPos, struct Per_data *perData);
void SetVirtualWall(struct Per_data *perData, int judge);
void DetectDanger(struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData, int judge);
void PerceptionForce(struct body_posi *jointPos, const struct leg_state *Leg, int mode, struct Per_data *perData, double x_dist, int judge);
