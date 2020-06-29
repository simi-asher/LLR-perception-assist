/*****************************************************************************************************************************
******************************************************************************************************************************
���[�U�[�����W�t�@�C���_(URG-04LX)�g�p�ɕK�v�Ȋ֐����܂Ƃ߂��w�b�_�t�@�C��
�\�t�g�E�F�A�Furg_library-1.2.0�g�p


******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once


#include <stdio.h>
#include <math.h>

#pragma comment(lib,"laserLarngeFinder.lib")
#include <URG/c/urg_sensor.h>
#include <URG/c/urg_utils.h>
#include "struct.h"
/************************************************************
					�}�N���̒�`
************************************************************/
//URG�̏����ݒ�p		

#define ScanTimes    1
#define SkipScan     0			//�w��X�L�����������ɋ������o�͂���
#define URG_FirstDeg -100 //simi prev -120L,0R		//<=0 �Z���T���ʂ���Ɍv���i����0[deg]�j
#define URG_FrontDeg 0 //simi prev 0
#define URG_LastDeg  100 //simi prev 0L,120R			//>=0 ����0�ȊO�̒l������Ƃ��������Ȃ�Ǝv����(length_data_size��front���S�ɕύX����K�v����)
#define SkipStep     2			//x�̃f�[�^���܂Ƃ߂�i�ŏ��l���o�́j(1�`


//Split and Merge�@�̂��߂̒�`
#define OBS_THRESHOLE   (0.1) //simi prev 0.1 0.07
#define SPLIT_THRESHOLD (0.03) //prev 0.05 0.03 //below 0.015 reduces the depth of obs
#define HEIGHT_THRESHOLD (0.15) //simi prev 0.12
#define WIDE_THRESHOLD (0.15)



/************************************************************
					�\����
************************************************************/
//enum{
//	URG_OFF=0,URG_ON,
//	NonFilter=0, AveragingFilter, GaussianFilter, DelayFilter,
//};
//
////urg�̍��W�f�[�^���i�[
//struct URG_data {
//	double rawX;
//	double rawY;
//	double x;
//	double y;
//};
//
////urg�̃f�[�^�����ɕ��̂����o���鎞�Ɏg�p
//struct Obstacle_data {
//	double x;	//�_��x���W
//	double y;	//�_��y���W
//	double bumpX;
//	double bumpY;
//	double height;
//	double distance;
//	double depth;
//	double grad;	//�����̌X��
//	double intr;	//�ؕ�
//	double bump_p;
//	int size;
//	bool flag;
//};
//
////�������o���Ɏg�p(Split and Merge�@�̏����̒�)
//struct Line_data{
//	bool flag;
//	int data_num;
//	double x;
//	double y;
//	double grad;
//	double intr;
//};

/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
//���C�����[�v�ŌĂяo���֐�
void Calculate_URG(urg_t *urg, long *length_data, int *length_data_size, int *urgTime, struct URG_data *urgData, struct Obstacle_data *obstacleData);

//URG�̏����ݒ�C�v���C�I������
void Init_URG(urg_t *urg, const long connect_baudrate, const char connect_device[]);	//���������ɌĂяo��
void Measure_URG(urg_t *urg, long *length_data, int *length_data_size, int *time);	//���[�v���ŌĂяo��
void KeepData(struct URG_data *beforeData, const int length_data_size, struct URG_data *afterData);
void EndMeasure(urg_t *urg);  //�I�����ɌĂяo��

//���f�[�^�̏���
void ChangeZeroPoint(long *length_data, const int length_data_size);
void ChangeCoordinate(urg_t *urg, long *length_data, const int length_data_size, struct URG_data *urgData);
void DeleateOfNotMeasureData(const int length_data_size, struct URG_data *urgData);
void FromSensorToGround(const int flag, urg_t *urg, const int length_data_size, struct URG_data *urgData);
void Smoothing_Data(const int filterMode, const int length_data_size, struct URG_data *urgData);
void ChangeData(struct URG_data *urgData, urg_t *urg, const int length_data_size, struct Obstacle_data *obstacleData); //simi urg_t *urg,

//Split and Merge�@
void DetectLine_SAM(struct Obstacle_data *obstacleData, int data_size); //Split and Merge�@���g�p����ꍇ�͂��̊֐����Ăяo��
void UpdataValue(struct Line_data *line, struct Line_data *new_line, const int data_num);
void DetectObject(struct Obstacle_data *obstacleData, int data_size, int *FINISH_POINT);
void InitDetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, int *point_num, const int START_POINT,const int FINISH_POINT);
void DetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, struct Line_data *new_line, int *point_num, const int data_num);
void FittingLine(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num);
void CalculateIntersection(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num);
void BumpRecognition(struct Obstacle_data *obstacleData, int point_num);