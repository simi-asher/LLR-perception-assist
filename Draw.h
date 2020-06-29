/*****************************************************************************************************************************
******************************************************************************************************************************
ツールバーの初期設定のためのヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/


#pragma once

#include <stdlib.h>
#include <Windows.h>
#include "calculateDraw.h"
#include "DrawParam.h"


/************************************************************
					マクロの定義
************************************************************/
#define PER (20) //3次元の地面のマスの数

#define TEXTURE_NUM (1)

#define COORDINATE2D_RIGHT  ( 0.9)
#define COORDINATE2D_LEFT   (-0.9)
#define COORDINATE2D_TOP    ( 0.9)
#define COORDINATE2D_BOTTOM (-0.9)

#define COORDINATE2D_MAX    (50)
#define COORDINATE2D_STEP   ( (COORDINATE2D_RIGHT-COORDINATE2D_LEFT)/COORDINATE2D_MAX )

/************************************************************
					構造体
************************************************************/



/************************************************************
					プロトタイプの宣言
************************************************************/
void Draw_Joint3D(const int t, data_struct *data, tweakParam  *barParam, GLuint texId[TEXTURE_NUM]);
void Draw_Joint2D(const int t, int DSP_WIDTH, int DSP_HEIGHT, data_struct *data, tweakParam  *barParam, GLuint texId[TEXTURE_NUM]);
void Draw_URG(const int t, int DSP_WIDTH, int DSP_HEIGHT, data_struct *data, GLuint texId[TEXTURE_NUM]);
void Draw_Graph(const int t, int DSP_WIDTH, int DSP_HEIGHT, data_struct *data);

void ModeURG(struct drawURG_struct *drawData);
void ModeObstacle(struct drawURG_struct *drawData, int mode);

void DrawBump(double bump[7], GLuint texId);
void DrawVWall(double bump[7], GLuint texId);
void DrawJoint(int t, draw_joint joint);
void CreateGround3D();
void CreateGround2D();

void DrawForceVector2D(draw_joint joint, double force[3]);
void CreateCoordinate2D(double *data, int num, int mode, double min, double max);
void DrawDataUp2D(const int t, GLdouble *data, int num, double min, double max);
void DrawDataDown2D(const int t, GLdouble *data, int num, double min, double max);
void CreateCoordinateTrajectory2D(double data[2], int mode, double min, double max);
void DrawDataTrajectory2D(const int t, GLdouble data[2], double min, double max);
void CreateCoordinate_URG();
void CreateCoordinate(double data[EMG_CH][RULE], int num, int mode, double min, double max);
void DrawData2D(GLdouble data[EMG_CH][RULE], int num, double min, double max);

int DrawBackground_URG();
int DrawProcessingData_URG();
void Circle(double r, int x, int y);
void DrawString(std::string str, int w, int h, double x0, double y0);


#define CenterX 0.85
#define CenterY 0.85
#define MapResolution 2
#define perCM   CenterY/MapResolution
#define StandardDistance (1.10)
#define CircleResolution 50