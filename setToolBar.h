/*****************************************************************************************************************************
******************************************************************************************************************************
�c�[���o�[�̏����ݒ�̂��߂̃w�b�_�t�@�C��


******************************************************************************************************************************
*****************************************************************************************************************************/


#pragma once

#include <stdlib.h>
#include <Windows.h>
#include <AntTweakBar.h>
#include "DrawParam.h"
#include "struct.h"

/************************************************************
					�}�N���̒�`
************************************************************/
#define DRAW_NUM (4)
#define DRAW_GRAPH_NUM (6)
#define CHANGE_GRAPH_NUM (6)

/************************************************************
					�\����
************************************************************/



/************************************************************
					�v���g�^�C�v�̐錾
************************************************************/
//�c�[���o�[�̐ݒ�
void CreateToolBars(tweakHandle *hBar, tweakParam *barParam);
void SetToolBar(tweakHandle *hBar, tweakParam *barParam);
void SetGraphBar_Param(tweakHandle *hBar, tweakParam *barParam, char *title_up, char *title_down);
void SetWorldBar_Param(TwBar *bar, worldParam *param);
void SetOperationBar_Param(TwBar *bar, tweakParam *param);
void SetValueBar_Param(TwBar *bar, valueParam *param);

//�c�[���o�[�̏����l
void InitParam_WorldBar(worldParam *param);

//�`�惂�[�h��ύX���ɍ��W�n��������
void ChangeDrawMode(DrawMode mode);

//�c�[���o�[�p�̃R�[���o�b�N�֐�
void TW_CALL SetAutoRotateCB(const void *value, void *clientData);
void TW_CALL GetAutoRotateCB(void *value, void *clientData);

void TW_CALL SetDrawModeCB(const void *value, void *clientData);
void TW_CALL GetDrawModeCB(void *value, void *clientData);

void TW_CALL SetChangeGraphUpCB(const void *value, void *clientData);
void TW_CALL SetChangeGraphDownCB(const void *value, void *clientData);
void TW_CALL GetChangeGraphCB(void *value, void *clientData);

void TW_CALL SetDrawGraphCB(const void *value, void *clientData);
void TW_CALL GetDrawGraphCB(void *value, void *clientData);

void TW_CALL SetRecordDataCB(const void *value, void *clientData);
void TW_CALL GetRecordDataCB(void *value, void *clientData);

void TW_CALL ResetButtonCB(void *clientData);
void TW_CALL QuitButtonCB(void *clientData);





/***************************************************************************************************************************************************************************************/
//���W�n�̉�]�Ȃǁi�T���v���R�[�h�ɂ��������̂����̂܂܎g�p�j
void SetQuaternionFromAxisAngle(const float *axis, float angle, float *quat);
void ConvertQuaternionToMatrix(const float *quat, float *mat);
void MultiplyQuaternions(const float *q1, const float *q2, float *qout);
int GetTimeMs();
void Terminate(void);
