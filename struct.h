/*****************************************************************************************************************************
******************************************************************************************************************************
構造体をまとめたヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#pragma once

#include <stdlib.h>
#include <Windows.h>
#include <gl/glew.h>
#include <gl/wglew.h>
#include <gl/glut.h>
#include <AntTweakBar.h>
#include "Parameter.h"

#define LEN_BODY 0.3				// length between hip joint and COG of body part( 現状長さ適当 )

#define LEN_H_K 0.40 //0.38 //simi new part				// Link length [m] : from hip joint to knee joint
#define LEN_K_A 0.40 //0.355 //0.38				// Link length [m] : from knee joint to ankle joint
#define LEN_A_T 0.25//0.25			// Link length [m] : from ankle joint to toe
#define LEN_A_S 0.10 //0.10				//Link length [m] : from ankle joint to sole
#define LEN_S_H 0.07				//Link length [m] : from sole to heel
#define LEN_S_T 0.22				//Link length [m] : from sole to toe //simi prev 0.20
#define LEN_A_P 0.20                // 反力を加える点の足首からの距離[m] Distance from the ankle of the point where the reaction force is applied
#define LEN_K_U  0.17 //0.21				// Link length [m] : from knee joint and URG //simi previously 0.15 //imp
#define LEN_LR_Hip  0.42			//Distance betwwen left and right Hip joint //simi MPU

#define LEN_HG (LEN_H_K/2)			//重心の位置
#define LEN_KG (LEN_K_A/2)
#define LEN_AG (LEN_A_T/2)
#define NN_INPUTS (36) //simi neural network

#define NN_OUTPUTS (1) //simi neural network

#define LINK_NUM (3)				//リンク数（股-膝，膝-足首，足首-つま先）

#define EMG_CH      (8)		//EMGのチャンネル数
#define NUM_RMS	(int)(0.1*SAMPLING_FREQ)	//2000[Hz]のとき200

#define RULE 32								// number of rules (4×4×2)現在てきとう
/************************************************************
					列挙型
************************************************************/
enum{
	Stand=0, SwingR, SwingL,
	HipBase=0, AnkleBase,
};

enum num_def{
	R=0, L=1,						//R:right L:left
	X=0, Y=1, Z=2,
	OFF=0, ON=1,
	H=0, K=1, A=2, U=3,				//H:hip K:knee: A:ankle U:URG
	POS=0, VEL=1, ACC=2,			//位置，速度，加速度
};

enum{
	FOOT_MOTION=0, GAIT_MOTION, UPSTAIR_MOTION,
	CONST_CTRL=0, P_CTRL, PD_CTRL,
};

enum{
	URG_OFF=0,URG_ON,
	NonFilter=0, AveragingFilter, GaussianFilter, DelayFilter,
};

typedef enum{ MODE_3D=0, MODE_2D, MODE_URG, MODE_GRAPH}DrawMode;
typedef enum{ NULL_DRAW=0, ANGLE_DRAW=1, FOOT_DRAW, TORQ_OUT_DRAW, TORQ_POW_DRAW, TORQ_PER_DRAW}DrawGraph;
typedef enum{ NULL_GRAPH=0, ANGLE_GRAPH=1, FOOT_GRAPH, TORQ_OUT_GRAPH, TORQ_POW_GRAPH, TORQ_PER_GRAPH}ChangeGraph;
typedef enum{ DISTANCE=0,HEIGHT = 1,DEPTH=2,VIR_START_X=3, VIR_START_Y=4, VIR_END_X=5, VIR_END_Y=6}BumpMode;

typedef enum { PO_B = 0, PO_S, ZERO, NE_S } fuzzy_set;


/************************************************************
					構造体
************************************************************/





/**************************  姿勢計算関係  *************************************/


struct motion_data{
	double pos[2];						// current position
	double vel[2];						// current velocity
	double acc[2];						// current accelaretion

	double prePos[2];						// current position
	double preVel[2];						// current velocity
	double preAcc[2];						// current accelaretion
};

struct leg_state{
	int state;
	int support;
	int swing;
};

//身体各部の座標(R,L X,Y)
struct body_posi{
	double Hip[2][2];
	double Knee[2][2];
	double Ankle[2][2];
	double Sole[2][2];
	double Toe[2][2];
	double Heel[2][2];
	double Urg[2][2];
	
	//身体各部位の重心の状態(R,L)
	struct motion_data UppThigh[2];		//上腿の位置,速度,加速度
	struct motion_data LowThigh[2];		//下腿の位置,速度,加速度
	struct motion_data Foot[2];			//足の位置,速度,加速度

	struct motion_data Body;				//胴体の重心座標(X,Y)

	double CoG[2];							//Center of Gravity (X,Y)
	double XZmp;							//ZMP(X方向のみ、Y=0:床面上）
};

//
struct jacob_data{
	//double trans[3][2];
	//double inv_trans[2][3];
	double matrix[6][6];		//ヤコビ行列 //simi new prev [3][3] for all below
	double diff[6][6];			//ヤコビ行列の微分
	double trans[6][6];			//ヤコビ行列の転置
	double inv[6][6];			//ヤコビ行列の逆行列
	double inv_trans[6][6];		//ヤコビ行列の逆行列の転置
	double pse[6][6];			//ヤコビ行列の擬似逆行列
	double pse_trans[6][6];		//ヤコビ行列の擬似逆行列の転置
	double null_space[6][6];	//ヤコビ行列の冗長性利用のための行列
};

struct jacob_body {
	double matrix[3][3];		//ヤコビ行列 
	double diff[3][3];			//ヤコビ行列の微分
	double trans[3][3];			//ヤコビ行列の転置
};

struct jacob_hip {
	double matrix[2][2];		//ヤコビ行列 
	double diff[2][2];			//ヤコビ行列の微分
	double trans[2][2];			//ヤコビ行列の転置
};
/**************************  動力学関係  *************************************/
struct transfer_data{
	double T[LINK_NUM][4][4];
	double T0[LINK_NUM][4][4];
	double T_diff[LINK_NUM][LINK_NUM][4][4];
	double T_diff_trans[LINK_NUM][LINK_NUM][4][4];
	double T_diff2[LINK_NUM][LINK_NUM][LINK_NUM][4][4];
	double H[LINK_NUM][4][4];
};

struct dynamic_data{
	double mass[LINK_NUM][LINK_NUM];
	double h[LINK_NUM];
	double gra[LINK_NUM];
};


/**************************  トルク関係  *************************************/
struct torq_data{
	double pow[2][3];						// torque by power-assist
	double per[2][3];				        // torque by perception-assist
	double gra[2][3];						// torque by gravity compensation
	double friction_for[2][3];				// torque by friction compensation (+)
	double friction_rev[2][3];				// torque by friction compensation (-)
	double vir[2][3];						// torque by virtual wall
	double fin[2][3];						// final torque
	double sw[2][3];						// モータの回転方向
	double out[2][3];						// モータ用に電圧に変換
	double per_comp;						//compensation at hip joint based on ZMP //simi
};



/**************************  パワーアシスト関係  *************************************/
//EMG
struct EMG_data{
	double raw[2][EMG_CH];
	double rms[2][EMG_CH];
};

typedef struct{
	double dev[3][4];				// Deviation value for EMG
	double cen[3][4];				// Center value for EMG
	double r_d[3][EMG_CH][RULE];	// Weight of between Rule and Defuzzifier layer
	double ori[3][EMG_CH];			// initial weight matrix
}_weight;



/**************************  認知アシスト関係  *************************************/
struct VirWall_data{
	double grad;
	double intr;
	double start_x;
	double start_y;
	double end_x;
	double end_y;
	double x_dist;
	double error;
	double force[2];
};

//認知アシストに必要なデータを格納
struct Per_data{
	double distance;
	double height;
	double depth;
	int    motion;
	bool   flag;
	bool   perFlag;

	struct VirWall_data virWall;
};

//urgの座標データを格納
struct URG_data {
	double rawX;
	double rawY;
	double x;
	double y;
};

//urgのデータを元に物体を検出する時に使用
struct Obstacle_data {
	double x;	//点のx座標
	double y;	//点のy座標
	double bumpX;
	double bumpY;
	double height;
	double distance;
	double depth;
	double grad;	//直線の傾き
	double intr;	//切片
	double bump_p;
	int size;
	bool flag;
};

//直線検出時に使用(Split and Merge法の処理の中)
struct Line_data{
	bool flag;
	int data_num;
	double x;
	double y;
	double grad;
	double intr;
};



/**************************  データ関係  *************************************/
//
struct Sensor_data{
	double enc_data[2][3];
	double force_data[2][3][2];
	EMG_data emg;

};

struct Record_data{ 
	double enc[2][3];
	double force[2][3];
	double torq[2][3];
	double torq_pow[2][3];
	double torq_per[2][3];
	double bump[7]; //simi virtual wall
	body_posi jointPos;
	EMG_data emg;
	float roll; //simi MPU
	float pitch1; //simi MPU
	float pitch2; //simi MPU
	float yaw1;
	float yaw2;
	double inputs[NN_INPUTS]; //simi neural network
	double outputs[NN_OUTPUTS]; //simi neural network
	bool per_flag; //flag for seeing if virtual wall is breached and PA is on
	double torq_per_comp;//perception assist torque compensation
};


/**************************  ツールバー関係  *************************************/
//絶対座標系の操作に関するツールバー
typedef struct{
	float Zoom;				// Shapes scale
	float Rotation[4];		// Shape orientation (stored as a quaternion)
	int AutoRotate;			// Auto rotate
	int RotateTime;
	float RotateStart[4];
	float MatAmbient[4];	// Shapes material
	float MatDiffuse[4];
	float LightMultiplier;	// Light parameter
	float LightDirection[3];

	float Color[3][4];

}worldParam;

//描画モードの変更に関するツールバー
typedef struct{
	DrawMode    drawMode;
	DrawGraph   drawGraph;
	ChangeGraph changeGraph[2];
	int         angData[2][3];

}operationParam;

//値の表示に関するツールバー
typedef struct{
	bool   recordFlag;
	double Foot[2];
	double ForceSensor[2][3];
	double ZmpX;
	int    time;
	double Freq;
	double DrawFreq;
	double Bump[6];
	double perForce[3];
	torq_data Torq;

}valueParam;

//ツールバーの設定に関する構造体をまとめた構造体
typedef struct{
	worldParam	   wldParam;
	operationParam opeParam;
	valueParam	   valParam;
}tweakParam;

//各ツールバーのハンドルをまとめた構造体
typedef struct{
	TwBar *worldBar;
	TwBar *valueBar;
	TwBar *operationBar;
	TwBar *graphUpBar;
	TwBar *graphDownBar;
}tweakHandle;

/**************************  描画関係  *************************************/
//描画用の関節のための構造体
typedef struct{
	double Hip[2][3];
	double Knee[2][3];
	double Ankle[2][3];
	double Toe[2][3];
	double Heel[2][3];
	double Body[3];
}draw_joint;

struct drawURG_struct{
	int    dataSize;
	int    dataBumpSize;
	double length[2];
	double position[2];
	double smooth[2];
};


//読み込んだデータを格納するための構造体
typedef struct{
	valueParam *element;
	draw_joint *joint;
	draw_joint drawJoint;
	double     bump[7];//last 3 for vwall, prev 6
	int        length;
	int        time;
	double     freq;
	torq_data  torq;
	drawURG_struct *urg;
	double     weight[EMG_CH][RULE];
	double	   outputs[NN_OUTPUTS]; //simi neural network
	bool perflag; //if perception assist is on/virtual wall breached
}data_struct;

