/*****************************************************************************************************************************
******************************************************************************************************************************
トルク計算に必要な関数をまとめたヘッダファイル



******************************************************************************************************************************
*****************************************************************************************************************************/
#pragma once

#include <stdio.h>
#include "Calculate_Position.h"
#include "Parameter.h"
#include <iostream>
/************************************************************
					マクロの定義
************************************************************/

//モータ出力電圧の上限
#define VOLT_LIMIT 5.0						// output voltage limiter

// トルク定数[Nm/V]	//
// トルク定数は30.2[mNm/A](カタログから)，[mNm/V]に変換するためにはモータドライバ(typeS)のマニュアルから±0.5[V/A] (5[V]/10[A]) //
// よってトルク定数は30.2/0.5 = 60.4[mNm/V] = 0.0604[Nm/V] //	(Max = 9.0600*5.0 = 45.300[Nm])
#define TORQUE_CONSTANT_HIP   (9.0600)			// 0.0604*ギア比(150)	(Max = 9.0600*5.0 = 45.300[Nm])
#define TORQUE_CONSTANT_KNEE  (6.8252)			// 0.0604*ギア比(113)	(Max = 6.8252*5.0 = 34.126[Nm])
#define TORQUE_CONSTANT_ANKLE (4.8924)		    // 0.0604*ギア比(81)		(Max = 4.8924*5.0 = 24.462[Nm])←モータが股関節だけ違うので本当は違う


//　モータの摩擦保障(正転)　
//モータ摩擦の電圧*1[V]あたりのトルク[Nm]
#define FRICTION_FORWARD_RH (0.67*TORQUE_CONSTANT_HIP)		//3.2779[Nm]
#define FRICTION_FORWARD_RK (0.69*TORQUE_CONSTANT_KNEE)		//4.7094[Nm]
#define FRICTION_FORWARD_RA (0.69*TORQUE_CONSTANT_ANKLE)	//6.2514[Nm]
#define FRICTION_FORWARD_LH (0.70*TORQUE_CONSTANT_HIP)		//3.4247[Nm]
#define FRICTION_FORWARD_LK (0.69*TORQUE_CONSTANT_KNEE)		//4.7094[Nm]
#define FRICTION_FORWARD_LA (0.70*TORQUE_CONSTANT_ANKLE)	//6.3420[Nm]
//　モータの摩擦保障(逆転)
#define FRICTION_REVERSE_RH (0.67*TORQUE_CONSTANT_HIP)		//3.2779[Nm]
#define FRICTION_REVERSE_RK (0.68*TORQUE_CONSTANT_KNEE)		//4.6411[Nm]
#define FRICTION_REVERSE_RA (0.68*TORQUE_CONSTANT_ANKLE)	//6.1608[Nm]
#define FRICTION_REVERSE_LH (0.71*TORQUE_CONSTANT_HIP)		//3.4736[Nm]
#define FRICTION_REVERSE_LK (0.69*TORQUE_CONSTANT_KNEE)		//4.7094[Nm]
#define FRICTION_REVERSE_LA (0.67*TORQUE_CONSTANT_ANKLE)	//6.0702[Nm]




/************************************************************
					構造体
************************************************************/
//struct torq_data{
//	double power[2][3];						// torque by power-assist
//	double perception[2][3];				// torque by perception-assist
//	double gravity[2][3];					// torque by gravity compensation
//	double friction_for[2][3];				// torque by friction compensation (+)
//	double friction_rev[2][3];				// torque by friction compensation (-)
//	double vir[2][3];						// torque by virtual wall
//	double fin[2][3];						// final torque
//	double sw[2][3];						// モータの回転方向
//	double out[2][3];						// モータ用に電圧に変換
//};




/************************************************************
					プロトタイプの宣言
************************************************************/
void Initialize_Torq(struct torq_data *torq);
void Calculate_Torq(struct torq_data *torq);
void Reset_torque(struct torq_data *torq);
void Friction_Compensation(const int flag, struct torq_data *torq);
void Calculate_Final_Torque(struct torq_data *torq);
void Calculate_Output_Torque(struct torq_data *torq);

void ConvertTorqueToForce(struct leg_state *Leg, struct jacob_data *jacobData, double torq[2][3], double force[3]);
void ConvertForceToTorque(struct leg_state *Leg, struct jacob_data *jacobData, double force[2], double torq[2][3]);//force prev 3
void ConvertForceToTorque(struct leg_state* Leg, struct jacob_body* jacobData, double force[2], double torq[2][3]);
void ConvertForceToTorque(struct leg_state* Leg, struct jacob_hip* jh, struct jacob_body* jb, double force[2], double torq[2][3]);//force prev 3
//void Convert_Torque_to_Force(struct leg_state *Leg, double jacob_ti[3][3], double torq[2][3], double force[2]); 
//void Convert_Torque_to_Force(struct leg_state *Leg, double jacob_ti[2][3], double torq[2][3], double force[2]);
//void Convert_force_to_torque(struct leg_state *Leg, double jacob_t[3][3], double force[2], double torq[2][3]);
//void Convert_force_to_torque(struct leg_state *Leg, double jacob_t[3][2], double force[2], double torq[2][3]);
void Calculate_Perception_Compensation(struct leg_state* Leg, struct torq_data* torq, struct body_posi* body_posi, double enc_data[2][3]);