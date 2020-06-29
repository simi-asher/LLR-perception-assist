/*****************************************************************************************************************************
******************************************************************************************************************************
ツールバーの初期設定のためのヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/


#pragma once

#include <stdlib.h>
#include <Windows.h>
#include <AntTweakBar.h>
#include "DrawParam.h"
#include "struct.h"

/************************************************************
					マクロの定義
************************************************************/
#define DRAW_NUM (4)
#define DRAW_GRAPH_NUM (6)
#define CHANGE_GRAPH_NUM (6)

/************************************************************
					構造体
************************************************************/



/************************************************************
					プロトタイプの宣言
************************************************************/
//ツールバーの設定
void CreateToolBars(tweakHandle *hBar, tweakParam *barParam);
void SetToolBar(tweakHandle *hBar, tweakParam *barParam);
void SetGraphBar_Param(tweakHandle *hBar, tweakParam *barParam, char *title_up, char *title_down);
void SetWorldBar_Param(TwBar *bar, worldParam *param);
void SetOperationBar_Param(TwBar *bar, tweakParam *param);
void SetValueBar_Param(TwBar *bar, valueParam *param);

//ツールバーの初期値
void InitParam_WorldBar(worldParam *param);

//描画モードを変更時に座標系を初期化
void ChangeDrawMode(DrawMode mode);

//ツールバー用のコールバック関数
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
//座標系の回転など（サンプルコードにあったものをそのまま使用）
void SetQuaternionFromAxisAngle(const float *axis, float angle, float *quat);
void ConvertQuaternionToMatrix(const float *quat, float *mat);
void MultiplyQuaternions(const float *q1, const float *q2, float *qout);
int GetTimeMs();
void Terminate(void);
