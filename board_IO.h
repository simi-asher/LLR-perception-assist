/*****************************************************************************************************************************
******************************************************************************************************************************
JW�{�[�h�̓��o�͂ɕK�v�Ȋ֐����܂Ƃ߂��w�b�_�t�@�C��
JWRIBPCI�t�@�C���e��Cboard�t�@�C���CParameter�t�@�C�����K�v
�܂�JW�{�[�h���ڂ��������pPC�Ŏg���K�v������


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <math.h>
#include "board.h"
#include "myFilter.h"
#include "Parameter.h"

/************************************************************
					�}�N���̒�`
************************************************************/

//�g�p����JW�{�[�h�̎w��
#define BOARD_ID_1 (1235)		// board1 ID	���[�^�E�G���R�[�_�E�̓Z���T�g�p	
#define BOARD_ID_2 (1200)		// board2 ID	EMG�g�p

//////// �`�����l���֌W�̐ݒ�p�}�N�� //////////
//�G���R�[�_�̃`�����l���w��
//R:�E�@L:��
//H:�ҁ@K:�G�@A:����
#define CH_ENC_RH (0)
#define CH_ENC_RK (1)
#define CH_ENC_RA (2)
#define CH_ENC_LH (3)
#define CH_ENC_LK (4)
#define CH_ENC_LA (5)

//�̓Z���T�̃`�����l���̎w��
#define CH_FORCE_RH_X (4)		// hip  force sensor channel(x axis)
#define CH_FORCE_RH_Y (5)		// hip  force sensor channel(y axis)
#define CH_FORCE_RK_X (6)
#define CH_FORCE_RK_Y (7)
#define CH_FORCE_RA_X (8)
#define CH_FORCE_RA_Y (9)

#define CH_FORCE_LH_X (10)
#define CH_FORCE_LH_Y (11)
#define CH_FORCE_LK_X (12)
#define CH_FORCE_LK_Y (13)
#define CH_FORCE_LA_X (14)
#define CH_FORCE_LA_Y (15)

//���[�^�̃`�����l���w��
#define CH_MOT_RH (0)
#define CH_MOT_RK (2)
#define CH_MOT_RA (4)
#define CH_MOT_LH (6)
#define CH_MOT_LK (8)
#define CH_MOT_LA (10)

//���[�^�̉�]�����w��
#define CH_SW_RH (1)
#define CH_SW_RK (3)
#define CH_SW_RA (5)
#define CH_SW_LH (7)
#define CH_SW_LK (9)
#define CH_SW_LA (11)


//////// �G���R�[�_�̕���\�ݒ�p�}�N�� //////////
#define RES_ENC_RH (614400.0)		//( 614400=1024*150*4 )
#define RES_ENC_RK (462848.0)		//( 462848=1024*113*4 )
#define RES_ENC_RA (331776.0)		//( 331776=1024*81*4 )
#define RES_ENC_LH (614400.0)		//( 614400=1024*150*4 )
#define RES_ENC_LK (462848.0)		//( 462848=1024*113*4 )
#define RES_ENC_LA (331776.0)		//( 331776=1024*81*4 )

/************************************************************
					�\����
************************************************************/
//struct EMG_data{
//	double raw[2][EMG_CH];
//	double rms[2][EMG_CH];
//};
//
//struct Sensor_data{
//	double enc_data[2][3];
//	double force_data[2][3][2];
//	EMG_data emg;
//
//};


/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/

void Calibrate_Sensor(BOARD *_board, double force_avg[2][3][2]);
void Read_Sensor(const int count, BOARD *_board, double force_cal[2][3][2], struct Sensor_data *data, float *Yaw1);
void Calculate_RMS(const int t, struct EMG_data *emgData);
void Calculate_RMS_L(const int t, struct EMG_data *emgData);