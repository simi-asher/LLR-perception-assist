/*****************************************************************************************************************************
******************************************************************************************************************************
数値計算に必要な関数をまとめたソースファイル



******************************************************************************************************************************
*****************************************************************************************************************************/

#include "calculate.h"
//#include "myFilter.cpp"


/****************************		速度・加速度，ZMP関係		*****************************************************************************************************/
float time_sum;
float sum_time;

/********************************************************
関数名：Calculate_Joint_VelAcc
説明　：位置データから速度，加速度の計算
引数　：const int count					時間
		double pos[2]					位置データ
		struct motion_data *motion		位置，速度，加速度データ
		
出力  ：struct motion_data *motion	
********************************************************/
void Calculate_Joint_VelAcc(const int count, double q[3][2][3]){//only for joints, not uppThigh,lowThigh and foot
	
	static double pre[3][2][3];
	static float buf_enc[3][2][3][2][FILTER_LEN];
	time_sum += SAMPLING_TIME;
	//現在（2016/10/24）データの平滑化を行っていないため必要に応じて実装してください!! Data smoothing not done
	//simi added low pass filter
	//if ((count % (SAMPLING_FREQ / 2000)) == 1) {	//2000Hz
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			q[VEL][i][j] = (q[POS][i][j] - pre[POS][i][j]) / time_sum; //prev SAMPLING_TIME
			q[VEL][i][j] = LowPassFilter(2000.0f, 2.0f, q[VEL][i][j], buf_enc[VEL][i][j]);
			q[ACC][i][j] = (q[VEL][i][j] - pre[VEL][i][j]) / time_sum;
			//q[ACC][i][j] = LowPassFilter(2000.0f, 2.0f, q[ACC][i][j], buf_enc[ACC][i][j]);

			pre[POS][i][j] = q[POS][i][j];
			pre[VEL][i][j] = q[VEL][i][j];
			pre[ACC][i][j] = q[ACC][i][j];
				
		}
		//simi commented
		/*q[VEL][i][H] = ( q[POS][i][H] - pre[POS][i][H] )/SAMPLING_TIME;
		q[VEL][i][K] = ( q[POS][i][K] - pre[POS][i][K] )/SAMPLING_TIME;
		q[VEL][i][A] = ( q[POS][i][A] - pre[POS][i][A] )/SAMPLING_TIME;

		q[ACC][i][H] = ( q[VEL][i][H] - pre[VEL][i][H] )/SAMPLING_TIME;
		q[ACC][i][K] = ( q[VEL][i][K] - pre[VEL][i][K] )/SAMPLING_TIME;
		q[ACC][i][A] = ( q[VEL][i][A] - pre[VEL][i][A] )/SAMPLING_TIME;*/
	}
	sum_time = time_sum;
	time_sum = 0.0f;
	/*}
	else {
		for (int i = 0; i < 2; i++) {
			for (int j = 0; j < 3; j++) {
				q[POS][i][j] = pre[POS][i][j];
				q[VEL][i][j] = pre[VEL][i][j];
				q[ACC][i][j] = pre[ACC][i][j];
			}
		}
	}*/
	//データの更新
	/*for(int i=0;i<2;i++){
		pre[POS][i][H] = q[POS][i][H];
		pre[POS][i][K] = q[POS][i][K];
		pre[POS][i][A] = q[POS][i][A];

		pre[VEL][i][H] = q[VEL][i][H];
		pre[VEL][i][K] = q[VEL][i][K];
		pre[VEL][i][A] = q[VEL][i][A];
	}*/
}


/********************************************************
関数名：Calculate_VelocityAcceleration
説明　：位置データから速度，加速度の計算
引数　：const int count					時間
		double pos[2]					位置データ
		struct motion_data *motion		位置，速度，加速度データ
		
出力  ：struct motion_data *motion	
********************************************************/
void Calculate_VelocityAcceleration(const int count, double pos[2], struct motion_data *motion, float buf[2][2][3][FILTER_LEN]){
	//buf: Body, RUpp,LUpp,...: pos, vel, acc:X, Y

	//現在（2016/10/24）データの平滑化を行っていないため必要に応じて実装してください!! As of (2016/10/24) data smoothing is not performed, please implement as needed !!

	//データの初期化
	if(count == 0){
		for(int i=0;i<2;i++){
			motion->prePos[i] = motion->pos[i];//simi to stop init vel, acc jump
			motion->preVel[i] = motion->vel[i];
			motion->preAcc[i] = motion->acc[i];
		}
	}

	else {

	//速度，加速度の計算
	//if (count % (SAMPLING_FREQ / 2000) == 1) {
		motion->pos[X] = pos[X];
		motion->pos[Y] = pos[Y];
		//motion->pos[X] = LowPassFilter(2000.0f, 10.0f, motion->pos[X], buf[POS][X]);
		//motion->pos[Y] = LowPassFilter(2000.0f, 10.0f, motion->pos[Y], buf[POS][Y]);

		motion->vel[X] = (motion->pos[X] - motion->prePos[X]) / sum_time;
		motion->vel[X]= LowPassFilter(2000.0f, 2.0f, motion->vel[X], buf[VEL][X]);
		motion->vel[Y] = (motion->pos[Y] - motion->prePos[Y]) / sum_time;
		motion->vel[Y] = LowPassFilter(2000.0f, 2.0f, motion->vel[Y], buf[VEL][Y]);

		motion->acc[X] = (motion->vel[X] - motion->preVel[X]) / sum_time;
		motion->acc[X]= LowPassFilter(2000.0f, 2.0f, motion->acc[X], buf[ACC][X]);
		motion->acc[Y] = (motion->vel[Y] - motion->preVel[Y]) / sum_time;
		motion->acc[Y] = LowPassFilter(2000.0f, 2.0f, motion->acc[Y], buf[ACC][Y]);

		//データの更新
		for (int i = 0; i < 2; i++) {
			motion->prePos[i] = motion->pos[i];
			motion->preVel[i] = motion->vel[i];
			motion->preAcc[i] = motion->acc[i];
		}
	}
	/*else if(count!=0){
		for (int i = 0; i < 2; i++) {
			motion->pos[i] = motion->prePos[i];
			motion->vel[i] = motion->preVel[i];
			motion->acc[i] = motion->preAcc[i];
		}
	}*/
}


/********************************************************
関数名：Calculate_ZMP
説明　：位置データから速度，加速度の計算
引数　：const int count					時間
		double pos[2]					位置データ
		struct motion_data *motion		位置，速度，加速度データ
		
出力  ：struct motion_data *motion	
********************************************************/
double Calculation_ZMP( struct body_posi *body_posi){

	double a=0;
	double preZMP = (*body_posi).XZmp;
	a=       ( SUM_FOOT*( ((*body_posi).Foot[R].acc[Y]+GRAVITY)*(*body_posi).Foot[R].pos[X]				- (*body_posi).Foot[R].acc[X]*(*body_posi).Foot[R].pos[Y] 
						+((*body_posi).Foot[L].acc[Y]+GRAVITY)*(*body_posi).Foot[L].pos[X]				- (*body_posi).Foot[L].acc[X]*(*body_posi).Foot[L].pos[Y] )
			  +SUM_LOWER*( ((*body_posi).LowThigh[R].acc[Y]+GRAVITY)*(*body_posi).LowThigh[R].pos[X]	- (*body_posi).LowThigh[R].acc[X]*(*body_posi).LowThigh[R].pos[Y] 
						+((*body_posi).LowThigh[L].acc[Y]+GRAVITY)*(*body_posi).LowThigh[L].pos[X]		- (*body_posi).LowThigh[L].acc[X]*(*body_posi).LowThigh[L].pos[Y] )
		      +SUM_THIGH*( ((*body_posi).UppThigh[R].acc[Y]+GRAVITY)*(*body_posi).UppThigh[R].pos[X]	- (*body_posi).UppThigh[R].acc[X]*(*body_posi).UppThigh[R].pos[Y] 
						+((*body_posi).UppThigh[L].acc[Y]+GRAVITY)*(*body_posi).UppThigh[L].pos[X]		- (*body_posi).UppThigh[L].acc[X]*(*body_posi).UppThigh[L].pos[Y] )
			  +MASS_BODY*( ((*body_posi).Body.acc[Y]+GRAVITY)*(*body_posi).Body.pos[X]					- (*body_posi).Body.acc[X]*(*body_posi).Body.pos[Y] ) 	
			 )/(
			  +SUM_FOOT*( (*body_posi).Foot[R].acc[Y]     +GRAVITY	+ (*body_posi).Foot[L].acc[Y]    +GRAVITY )
			  +SUM_LOWER*( (*body_posi).LowThigh[R].acc[Y]+GRAVITY	+ (*body_posi).LowThigh[L].acc[Y]+GRAVITY )
			  +SUM_THIGH*( (*body_posi).UppThigh[R].acc[Y]+GRAVITY	+ (*body_posi).UppThigh[L].acc[Y]+GRAVITY )
			  +MASS_BODY *( (*body_posi).Body.acc[Y]      +GRAVITY )
			  );


	//simi to limit ZMP. determine actual limits
	/*if (a > 300)
		a = 299.0;
	else if (a < -300)
		a = -299.0;*/
	if (abs(a) > 100)
		a = preZMP;
	return ( a );		  
}



//	**************	CoGの計算	**************************//
//
//	*****************************************************//
int Calculation_Center_of_Gravity( struct body_posi *body_posi ){

	//上腿、下腿、足のそれぞれの重心の計算(R/L, X,Y)
	(*body_posi).UppThigh[R].pos[X]=( (*body_posi).Hip[R][X]+(*body_posi).Knee[R][X] )/2;			(*body_posi).UppThigh[R].pos[Y] = ( (*body_posi).Hip[R][Y]+(*body_posi).Knee[R][Y] ) /2;
	(*body_posi).LowThigh[R].pos[X]=( (*body_posi).Knee[R][X]+(*body_posi).Ankle[R][X] )/2;		(*body_posi).LowThigh[R].pos[Y] = ( (*body_posi).Knee[R][Y]+(*body_posi).Ankle[R][Y] ) /2;
	(*body_posi).Foot[R].pos[X]=( (*body_posi).Ankle[R][X]+(*body_posi).Toe[R][X] )/2;			(*body_posi).Foot[R].pos[Y] = ( (*body_posi).Ankle[R][Y]+(*body_posi).Toe[R][Y] ) /2;

	(*body_posi).UppThigh[L].pos[X]=( (*body_posi).Hip[L][X]+(*body_posi).Knee[L][X] )/2;			(*body_posi).UppThigh[L].pos[Y] = ( (*body_posi).Hip[L][Y]+(*body_posi).Knee[L][Y] ) /2;
	(*body_posi).LowThigh[L].pos[X]=( (*body_posi).Knee[L][X]+(*body_posi).Ankle[L][X] )/2;		(*body_posi).LowThigh[L].pos[Y] = ( (*body_posi).Knee[L][Y]+(*body_posi).Ankle[L][Y] ) /2;
	(*body_posi).Foot[L].pos[X]=( (*body_posi).Ankle[L][X]+(*body_posi).Toe[L][X] )/2;			(*body_posi).Foot[L].pos[Y] = ( (*body_posi).Ankle[L][Y]+(*body_posi).Toe[L][Y] ) /2;

	(*body_posi).CoG[X]=0;
	(*body_posi).CoG[Y]=0;

	(*body_posi).CoG[X] = ( (*body_posi).Body.pos[X]*(SUM_BODY)
							+(*body_posi).UppThigh[R].pos[X]*(SUM_THIGH) + (*body_posi).LowThigh[R].pos[X]*(SUM_LOWER) + (*body_posi).Foot[R].pos[X]*(SUM_FOOT)
							+(*body_posi).UppThigh[L].pos[X]*(SUM_THIGH) + (*body_posi).LowThigh[L].pos[X]*(SUM_LOWER) + (*body_posi).Foot[L].pos[X]*(SUM_FOOT) )/ MASS_ALL;
	(*body_posi).CoG[Y] = ( (*body_posi).Body.pos[Y]*(SUM_BODY)
							+(*body_posi).UppThigh[R].pos[Y]*(SUM_THIGH) + (*body_posi).LowThigh[R].pos[Y]*(SUM_LOWER) + (*body_posi).Foot[R].pos[Y]*(SUM_FOOT)
							+(*body_posi).UppThigh[L].pos[Y]*(SUM_THIGH) + (*body_posi).LowThigh[L].pos[Y]*(SUM_LOWER) + (*body_posi).Foot[L].pos[Y]*(SUM_FOOT) )/ MASS_ALL;
	
	
	//デバッグ用
	//printf("②CoG[X] =%lf\t, %lf\t\n ", (*body_posi).CoG[X],(*body_posi).CoG[Y] );

	
	return 0;
}


//	**************	ZMPとCoGの計算	**************************//
//
//	*****************************************************//
void Calculate_ZMP_COG(int t,struct body_posi *body_posi){
		
	(*body_posi).XZmp=0;
	// calculate center of gravity of each body and whole body //
	Calculation_Center_of_Gravity(body_posi);

	// calculate velocity and acceleration //
	
	//static double pre[3][7][2]; //pos, vel, acc: Body, RUpp,LUpp,...,X, Y
	static float buf[7][2][2][3][FILTER_LEN]; //Body, RUpp,RLow, RFoot,LUpp,...: pos, vel, acc:X, Y.[2][3]

	Calculate_VelocityAcceleration(t, body_posi->Body.pos, &body_posi->Body, buf[0]); //simi changed since buf is static
	for (int i = 0; i < 2; i++) {
		Calculate_VelocityAcceleration(t, body_posi->UppThigh[i].pos, &body_posi->UppThigh[i], buf[1 + i * 3]);
		Calculate_VelocityAcceleration(t, body_posi->LowThigh[i].pos, &body_posi->LowThigh[i], buf[2 + i * 3]);
		Calculate_VelocityAcceleration(t, body_posi->Foot[i].pos, &body_posi->Foot[i], buf[3 + i * 3]);
	}

	// calculate ZMP //
	(*body_posi).XZmp = Calculation_ZMP(body_posi);
	
		
}


/********************************************************
関数名：Calculate_JacobMatrix
説明　：股関節基準のヤコビ行列の計算
		メインループ内で使用
引数　：double enc_data[2][3]		    3×3の単位行列
		struct leg_state *Leg			遊脚，立脚の判定
		struct jacob_data *jacobData    ヤコビ行列関係の構造体
		
出力　：struct jacob_data *jacobData
********************************************************/
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state *Leg, struct jacob_data *jacobData){

	//地面（x-y座標系）に対しての角度なので注意 Note that the angle is with respect to the ground (x-y coordinate system)
	double q[2][3];
	double dq[2][3];
	double body_ang = 0;
	
	/*[(HK * cos(t1 + t2)) / 2 - HK / 2 + KA * cos(t1 - t2 + t3 - t4 + t5) - KA * cos(t1) - (HK * cos(t1 - t2)) / 2 - (HK * cos(2 * t1)) / 2 + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), (HK * cos(t1 + t2)) / 2 - HK / 2 + KA * cos(t1 - t2 + t3 - t4 + t5) - (HK * cos(t1 - t2)) / 2 - (HK * cos(2 * t1)) / 2 + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), KA * cos(t1 - t2 + t3 - t4 + t5) + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), KA * cos(t1 - t2 + t3 - t4 + t5) + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), KA * cos(t1 - t2 + t3 - t4 + t5) + AT * sin(t1 - t2 + t3 - t4 + t5 + t6), AT * sin(t1 - t2 + t3 - t4 + t5 + t6)]
	[KA * sin(t1) - (HK * sin(t1 + t2)) / 2 - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4) + (HK * sin(t1 - t2)) / 2 + AT * cos(t1 - t2 + t3 - t4 + t5 + t6) + (HK * sin(2 * t1)) / 2, (HK * sin(t1 - t2)) / 2 - (HK * sin(t1 + t2)) / 2 - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4) + AT * cos(t1 - t2 + t3 - t4 + t5 + t6) + (HK * sin(2 * t1)) / 2, AT * cos(t1 - t2 + t3 - t4 + t5 + t6) - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4), AT * cos(t1 - t2 + t3 - t4 + t5 + t6) - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4), AT * cos(t1 - t2 + t3 - t4 + t5 + t6) - KA * sin(t1 - t2 + t3 - t4 + t5), AT * cos(t1 - t2 + t3 - t4 + t5 + t6)]
	[0, 0, 0, 0, 0, 0]
	[0, 0, 0, 0, 0, 0]
	[0, 0, 0, 0, 0, 0]
	[1, 1, 1, 1, 1, 1]*/
	

	//遊脚の地面基準の関節角度 Ground angle of swing leg
	//Calculation_Swing_Angle(angData[POS], Leg, q, body_ang); //simi commented
	for (int j=0;j<3;j++){
		q[R][j] = angData[POS][R][j];//simi added to make R H origin, indep of left leg
		dq[R][j] = angData[VEL][R][j];
		q[L][j] = angData[POS][L][j];
		dq[L][j] = angData[VEL][L][j];
	}
		
	printf("H %lf, K %lf, A %lf\n", q[R][H], q[R][K], q[R][A]); //print cur
	//ヤコビの計算(2×3)　地面（x-y平面)での腰基準ヤコビアン(行列計算の簡単化のため3×3)
	//Jacobi calculation (2 × 5) 腰 ankle reference Jacobian on the ground (x-y plane) (3 × 5 to simplify matrix calculation)
	jacobData->matrix[0][0] = (LEN_H_K * cos(q[L][A] + q[L][K])) / 2 - LEN_H_K / 2 + LEN_K_A * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K]) - LEN_K_A * cos(q[L][A]) - (LEN_H_K * cos(q[L][A] - q[L][K])) / 2 - (LEN_H_K * cos(2 * q[L][A])) / 2 + LEN_A_T * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]) + LEN_H_K * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H]);
	jacobData->matrix[0][1] = (LEN_H_K * cos(q[L][A] + q[L][K])) / 2 - LEN_H_K / 2 + LEN_K_A * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K])							 - (LEN_H_K * cos(q[L][A] - q[L][K])) / 2 - (LEN_H_K * cos(2 * q[L][A])) / 2 + LEN_A_T * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]) + LEN_H_K * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H]);
	jacobData->matrix[0][2] =													   + LEN_K_A * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K]) 																									 + LEN_A_T * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]) + LEN_H_K * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H]);
	jacobData->matrix[0][3] =													   + LEN_K_A * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K])																										 + LEN_A_T * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]) + LEN_H_K * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H]);
	jacobData->matrix[0][4] =													   + LEN_K_A * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K])																										 + LEN_A_T * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]);
	jacobData->matrix[0][5] =																																																							 + LEN_A_T * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]);

	jacobData->matrix[1][0] = LEN_K_A * sin(q[L][A]) - (LEN_H_K * sin(q[L][A] + q[L][K])) / 2 - LEN_K_A * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K]) - LEN_H_K * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H]) + (LEN_H_K * sin(q[L][A] - q[L][K])) / 2 + LEN_A_T * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]) + (LEN_H_K * sin(2 * q[L][A])) / 2;
	jacobData->matrix[1][1] =						 - (LEN_H_K * sin(q[L][A] + q[L][K])) / 2 - LEN_K_A * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K]) - LEN_H_K * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H]) + (LEN_H_K * sin(q[L][A] - q[L][K])) / 2 + LEN_A_T * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]) + (LEN_H_K * sin(2 * q[L][A])) / 2;
	jacobData->matrix[1][2] =																  - LEN_K_A * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K]) - LEN_H_K * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H])										   + LEN_A_T * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]);
	jacobData->matrix[1][3] =																  - LEN_K_A * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K]) - LEN_H_K * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H])										   + LEN_A_T * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]);																																								
	jacobData->matrix[1][4] =																  - LEN_K_A * sin(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K])																								   + LEN_A_T * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]);
	jacobData->matrix[1][5] =																																																								   + LEN_A_T * cos(q[L][A] - q[L][K] + q[L][H] - q[R][H] + q[R][K] + q[R][A]);
	
	jacobData->matrix[2][0] = 0.0;
	jacobData->matrix[2][1] = 0.0;
	jacobData->matrix[2][2] = 0.0;
	jacobData->matrix[2][3] = 0.0;
	jacobData->matrix[2][4] = 0.0;
	jacobData->matrix[2][5] = 0.0;

	jacobData->matrix[3][0] = 0.0;
	jacobData->matrix[3][1] = 0.0;
	jacobData->matrix[3][2] = 0.0;
	jacobData->matrix[3][3] = 0.0;
	jacobData->matrix[3][4] = 0.0;
	jacobData->matrix[3][5] = 0.0;

	jacobData->matrix[4][0] = 0.0;
	jacobData->matrix[4][1] = 0.0;
	jacobData->matrix[4][2] = 0.0;
	jacobData->matrix[4][3] = 0.0;
	jacobData->matrix[4][4] = 0.0;
	jacobData->matrix[4][5] = 0.0;

	jacobData->matrix[5][0] = 1.0;
	jacobData->matrix[5][1] = 1.0;
	jacobData->matrix[5][2] = 1.0;
	jacobData->matrix[5][3] = 1.0;
	jacobData->matrix[5][4] = 1.0;
	jacobData->matrix[5][5] = 1.0;
	
	////ヤコビの微分の計算(2×3)　地面（x-y平面)での腰基準ヤコビアン(行列計算の簡単化のため3×5)
	//jacobData->diff[0][0] =  -LEN_K_A*(dq[L][A])*sin( q[L][A]) - LEN_H_K*( dq[L][A] + dq[L][K])*sin( q[L][A] + q[L][K]) - LEN_H_K*( dq[L][A] + dq[L][K] + dq[R][H])*sin( q[L][A] + q[L][K] + q[R][H]) - LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) - LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[0][1] =                                	   - LEN_H_K*( dq[L][A] + dq[L][K])*sin( q[L][A] + q[L][K]) - LEN_H_K*( dq[L][A] + dq[L][K] + dq[R][H])*sin( q[L][A] + q[L][K] + q[R][H]) - LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) - LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[0][2] =													                               				- LEN_H_K*( dq[L][A] + dq[L][K] + dq[R][H])*sin( q[L][A] + q[L][K] + q[R][H]) - LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) - LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[0][3] =																																											  - LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) - LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[0][4] =																																																																				   - LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*sin( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[1][0] =  LEN_K_A*(dq[L][A])*cos( q[L][A]) + LEN_H_K*( dq[L][A] + dq[L][K])*cos( q[L][A] + q[L][K]) + LEN_H_K*( dq[L][A] + dq[L][K] + dq[R][H])*cos( q[L][A] + q[L][K] + q[R][H]) + LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) + LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[1][1] =                                	  + LEN_H_K*( dq[L][A] + dq[L][K])*cos( q[L][A] + q[L][K]) + LEN_H_K*( dq[L][A] + dq[L][K] + dq[R][H])*cos( q[L][A] + q[L][K] + q[R][H]) + LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) + LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[1][2] =													                               			   + LEN_H_K*( dq[L][A] + dq[L][K] + dq[R][H])*cos( q[L][A] + q[L][K] + q[R][H]) + LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) + LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[1][3] =																																											 + LEN_K_A*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] ) + LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[1][4] =																																																																				  + LEN_A_T*( dq[L][A] + dq[L][K] + dq[R][H] + dq[R][K] + dq[R][A] )*cos( q[L][A] + q[L][K] + q[R][H] + q[R][K] + q[R][A] );
	//jacobData->diff[2][0] = 0.0;
	//jacobData->diff[2][1] = 0.0;
	//jacobData->diff[2][2] = 0.0;
	//jacobData->diff[2][3] = 0.0;
	//jacobData->diff[2][4] = 0.0;
	//jacobData->diff[3][0] = 0.0;
	//jacobData->diff[3][1] = 0.0;
	//jacobData->diff[3][2] = 0.0;
	//jacobData->diff[3][3] = 0.0;
	//jacobData->diff[3][4] = 0.0;
	//jacobData->diff[4][0] = 0.0;
	//jacobData->diff[4][1] = 0.0;
	//jacobData->diff[4][2] = 0.0;
	//jacobData->diff[4][3] = 0.0;
	//jacobData->diff[4][4] = 0.0;
	//	
	// //ヤコビの微分の計算(2×3)　地面（x-y平面)での腰基準ヤコビアン(行列計算の簡単化のため3×3)
	// jacobData->diff[0][0] =  -LEN_H_K*( dq[H] )*sin( q[H] ) - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	// jacobData->diff[0][1] =                                 - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );       
	// jacobData->diff[0][2] =													                               - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	// jacobData->diff[1][0] =   LEN_H_K*( dq[H] )*cos( q[H] ) + LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );                                     
	// jacobData->diff[1][1] =                                   LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] ); 
	// jacobData->diff[1][2] =                                                                                  LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );
	// jacobData->diff[2][0] = 0.0;
	// jacobData->diff[2][1] = 0.0;
	// jacobData->diff[2][2] = 0.0;

	//ヤコビ行列の転置の計算
	//Matrix33_Trans(jacobData->matrix, jacobData->trans);
	Matrix66_Trans(jacobData->matrix, jacobData->trans);
	
	//ヤコビ行列の擬似逆行列の計算
	//Matrix33_PseudoInverse(jacobData->matrix, jacobData->pse);

	//擬似逆行列の転置の計算
	//Matrix33_Trans(jacobData->pse, jacobData->pse_trans);

	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,jacobData->pse[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}

/********************************************************
関数名：Calculate_JacobMatrix
説明　：股関節基準のヤコビ行列の計算
		メインループ内で使用
引数　：double enc_data[2][3]		    3×3の単位行列
		struct leg_state *Leg			遊脚，立脚の判定
		struct jacob_data *jacobData    ヤコビ行列関係の構造体
		struct jacob_data *jacobData    ヤコビ行列関係の構造体
		
出力　：struct jacob_data *jacobData
********************************************************/
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state *Leg, struct jacob_hip *jh,struct jacob_body *jb){//sit ankle base- SUP leg

	//地面（x-y座標系）に対しての角度なので注意 Note that the angle is with respect to the ground (x-y coordinate system)
	double q[3];
	double dq[3];
	double body_ang = 0;
	////for body only (sitting)
	//Jb = np.array([[+LEN_BODY * math.cos(q[H] + q[A] + q[K]) + LEN_H_K * math.cos(-q[A] - q[K]) + LEN_K_A * math.cos(q[A])
	//	, +LEN_BODY * math.cos(q[H] + q[A] + q[K]) + LEN_H_K * math.cos(-q[A] - q[K])
	//	, +LEN_BODY * math.cos(q[A] + q[K] + q[H])],
	//	[+LEN_BODY * math.sin(q[A] + q[K] + q[H]) - LEN_H_K * math.sin(-q[A] - q[K]) + LEN_K_A * math.sin(q[A])
	//	, +LEN_BODY * math.sin(q[A] + q[K] + q[H]) - LEN_H_K * math.sin(-q[A] - q[K])
	//	, +LEN_BODY * math.sin(q[A] + q[K] + q[H])],
	//	[0., 0., 0.]] )
	//Jh = np.array([[+LEN_H_K * math.cos(-q[A] - q[K]) + LEN_K_A * math.cos(q[A])
	//		, LEN_H_K * math.cos(-q[A] - q[K])],
	//		[-LEN_H_K * math.sin(-q[A] - q[K]) + LEN_K_A * math.sin(q[A])
	//		, -LEN_H_K * math.sin(-q[A] - q[K])],
	//		[0., 0.]] )
	//遊脚の地面基準の関節角度 Ground angle of swing leg
	//Calculation_Swing_Angle(angData[POS], Leg, q, body_ang); //simi commented
	for (int j=0;j<3;j++)
		q[j] = angData[POS][L][j];//simi added to make R H origin, indep of left leg. NOTE: SUP(L) angles may be opp of SW angles, in which case make these neg
	for(int i=0;i<3;i++) dq[i] = angData[VEL][L][i];
	printf("H %lf, K %lf, A %lf\n", q[H], q[K], q[A]); //print cur
	//ヤコビの計算(2×3)　地面（x-y平面)での腰基準ヤコビアン(行列計算の簡単化のため3×3)
	//Jacobi calculation (2 × 3) 腰 waist reference Jacobian on the ground (x-y plane) (3 × 3 to simplify matrix calculation)
	jb->matrix[0][0] = LEN_BODY * cos(q[A] + q[K] + q[H]) + LEN_H_K * cos(-q[A] - q[K]) + LEN_K_A * cos(q[A]);
	jb->matrix[0][1] = LEN_BODY * cos(q[A] + q[K] + q[H]) + LEN_H_K * cos(-q[A] - q[K]);
	jb->matrix[0][2] = LEN_BODY * cos(q[A] + q[K] + q[H]);
	jb->matrix[1][0] =  LEN_BODY * sin(q[A] + q[K] + q[H]) - LEN_H_K * sin(-q[A] - q[K]) + LEN_K_A * sin(q[A]);
	jb->matrix[1][1] =  LEN_BODY * sin(q[A] + q[K] + q[H]) - LEN_H_K * sin(-q[A] - q[K]);
	jb->matrix[1][2] =  LEN_BODY * sin(q[A] + q[K] + q[H]);
	jb->matrix[2][0] = 0;
	jb->matrix[2][1] = 0;
	jb->matrix[1][2] = 0;

	jh->matrix[0][0] = + LEN_H_K * cos(-q[A] - q[K]) + LEN_K_A * cos(q[A]);
	jh->matrix[0][1] = + LEN_H_K * cos(-q[A] - q[K]);
	jh->matrix[1][0] = - LEN_H_K * sin(-q[A] - q[K]) + LEN_K_A * sin(q[A]);
	jh->matrix[1][1] = - LEN_H_K * sin(-q[A] - q[K]);
	jh->matrix[2][0] = 0;
	jh->matrix[2][1] = 0;
	
	////ヤコビの微分の計算(2×3)　地面（x-y平面)での腰基準ヤコビアン(行列計算の簡単化のため3×3)
	//Jb->diff[0][0] =  -LEN_H_K*( dq[H] )*sin( q[H] ) - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	//Jb->diff[0][1] =                                 - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );       
	//Jb->diff[0][2] =													                               - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	//Jb->diff[1][0] =   LEN_H_K*( dq[H] )*cos( q[H] ) + LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );                                     
	//Jb->diff[1][1] =                                   LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] ); 
	//Jb->diff[1][2] =                                                                                  LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );
	//Jb->diff[2][0] = 0.0;
	//Jb->diff[2][1] = 0.0;
	//Jb->diff[2][2] = 0.0;

	//ヤコビ行列の転置の計算
	Matrix33_Trans(jb->matrix, jb->trans);
	Matrix22_Trans(jh->matrix, jh->trans);
	////ヤコビ行列の擬似逆行列の計算
	//Matrix33_PseudoInverse(jacobData->matrix, jacobData->pse);

	////擬似逆行列の転置の計算
	//Matrix33_Trans(jacobData->pse, jacobData->pse_trans);

	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,jacobData->pse[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}

void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state* Leg, struct jacob_body* jacob_data) { //asc/desc hip base-SW leg
	/*np.array([[+LEN_H_K * math.cos(q[H]) + LEN_K_A * math.cos(q[H] + q[K]) + LEN_A_T * math.sin(q[H] + q[K] + q[A]) #prev - LEN_H_K
		, +LEN_K_A * math.cos(q[H] + q[K]) + LEN_A_T * math.sin(q[H] + q[K] + q[A])
		, LEN_A_T * math.sin(q[H] + q[K] + q[A])],
		[LEN_H_K * math.sin(q[H]) + LEN_K_A * math.sin(q[H] + q[K]) + LEN_A_T * math.cos(q[H] + q[K] + q[A])
		, LEN_K_A * math.sin(q[H] + q[K]) + LEN_A_T * math.cos(q[H] + q[K] + q[A])
		, LEN_A_T * math.cos(q[H] + q[K] + q[A])],
		[0., 0., 0.]] )*/
	double q[3];
	double dq[3];
	double body_ang = 0;

	for (int j = 0;j < 3;j++)
		q[j] = angData[POS][R][j];//simi added to make R H origin, indep of left leg
	for (int i = 0;i < 3;i++) dq[i] = angData[VEL][R][i];
	//printf("H %lf, K %lf, A %lf\n", q[H], q[K], q[A]); //print cur
	//ヤコビの計算(2×3)　地面（x-y平面)での腰基準ヤコビアン(行列計算の簡単化のため3×3)
	//Jacobi calculation (2 × 3) 腰 waist reference Jacobian on the ground (x-y plane) (3 × 3 to simplify matrix calculation)
	jacob_data->matrix[0][0] = LEN_H_K * cos(q[H]) + LEN_K_A * cos(q[H] + q[K]) + LEN_A_T * sin(q[H] + q[K] + q[A]);
	jacob_data->matrix[0][1] = LEN_K_A * cos(q[H] + q[K]) + LEN_A_T * sin(q[H] + q[K] + q[A]);
	jacob_data->matrix[0][2] = LEN_A_T * sin(q[H] + q[K] + q[A]);
	jacob_data->matrix[1][0] = LEN_H_K * sin(q[H]) + LEN_K_A * sin(q[H] + q[K]) + LEN_A_T * cos(q[H] + q[K] + q[A]);
	jacob_data->matrix[1][1] = LEN_K_A * sin(q[H] + q[K]) + LEN_A_T * cos(q[H] + q[K] + q[A]);
	jacob_data->matrix[1][2] = LEN_A_T * cos(q[H] + q[K] + q[A]);
	jacob_data->matrix[2][0] = 0;
	jacob_data->matrix[2][1] = 0;
	jacob_data->matrix[1][2] = 0;

	//ヤコビ行列の転置の計算
	Matrix33_Trans(jacob_data->matrix, jacob_data->trans);

}


/****************************		インピーダンス制御関係		*****************************************************************************************************/

//int Impedance_Control_Acc(double foot[3][2][3], double target[3][2][3], double force[3]){
//
//
//	//インピーダンス制御
//	//足先加速度から
//	force[X] = 0.0;
//	force[Y] = ( MASS_ALL-MASS_d )*foot[ACC][R][Y]
//				 - VIS_Y*( foot[VEL][R][Y] - target[VEL][R][Y] )
//				 - SPR_Y*( foot[POS][R][Y] - target[POS][R][Y] );
//	force[Z] = 0.0;
//
//	return 0;
//}
//
//
//int Impedance_Control_Force(double foot[3][2][3], double target[3][2][3], double Force_d[3], double force[3]){
//
//	double mass = MASS_ALL/MASS_d;
//
//	//インピーダンス制御
//	//足先力から
//	force[X] = 0.0;
//	force[Y] = ( mass - 1 )*Force_d[Y]
//				 - mass*VIS_Y*( foot[VEL][R][Y] - target[VEL][R][Y] )
//				 - mass*SPR_Y*( foot[POS][R][Y] - target[POS][R][Y] );
//
//	force[Z] = 0.0;
//
//	return 0;
//}


/********************************************************
関数名：ConvertForceToAcc
説明　：つま先の力からつま先の加速度に変換
引数　：double force[3]		//つま先に加わる力
		double acc[3]		//つま先に生じる加速度
	　　
出力　：double acc[3]
********************************************************/
void ConvertForceToAcc(double force[3], double acc[3]){

	//MASS_FOOTを質量行列に変更する必要あり?
	acc[X] = force[X]/MASS_FOOT;
	acc[Y] = force[Y]/MASS_FOOT;
	acc[Z] = force[Z]/MASS_FOOT;
}





/****************************		冗長性利用関係		*****************************************************************************************************/


/********************************************************
関数名：Calculate_UsingRedundancy2
説明　：トルクに関する冗長性利用を計算する関数
		メインループ内で使用
引数　：double jointVal[2][3]			エンコーダの値を元にした位置，速度
		double targetAcc[3]				認知アシストにより与えられる加速度
		struct jacob_data jacobData		ヤコビ行列関係の構造体
		double redundancyAcc[3]			冗長性利用プログラムにより得られた関節角加速度
		
出力　：double redundancyAcc[3]
********************************************************/
void Calculate_UsingRedundancy2(double jointVal[3][2][3], double force[3], double error[3][3], struct jacob_data *J, struct dynamic_data *D, double torq[3], double torq2[3]){

	double I[3][3];
	double nullSpaceVector[3];
	double K_f[3][3] = {
							{ 1.0, 0.0, 0.0 },
							{ 0.0, 1.5, 0.0 },
							{ 0.0, 0.0, 1.0 }
						};
	double D_d[3][3] = {
							{ 1.0, 0.0, 0.0 },
							{ 0.0, 1.0, 0.0 },
							{ 0.0, 0.0, 0.0 }
						};
	double K_d[3][3] = {
							{ 1.0, 0.0, 0.0 },
							{ 0.0, -290.0, 0.0 },
							{ 0.0, 0.0, 0.0 }
						};


	//単位行列の計算
	Matrix_Unit(I);
	double F_tmp[3][3]={0.0};
	double F[3]={0.0}, X_d[3]={0.0}, X[3]={0.0};
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			F_tmp[i][j] = K_f[i][j] - I[i][j];
		
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			F[i]   += F_tmp[i][j]*force[j];
			X_d[i] += D_d[i][j]  *error[VEL][j];
			X[i]   += K_d[i][j]  *error[POS][j];
		}
	}

	
	//ゼロ空間ベクトルの計算( (I-J#J)k )
	Calculate_NullSpace(jointVal, J, nullSpaceVector);
	
	double M[3][3]={0}, h[3]={0};
	double M_tmp[3][3];
	//Matrix33_Product(J->pse_trans, D->mass, M_tmp);
	//Matrix33_Product(M_tmp, J->pse, M);


	double V_tmp1[3]={0}, V_tmp2[3]={0}, V_tmp3[3]={0};		//一時保存のためのベクトル
	//冗長性を利用した関節角加速度の計算
	//( dJdq )
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			V_tmp1[i] += J->diff[i][j]*jointVal[VEL][R][j];

	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			V_tmp2[i] += M[i][j]*V_tmp1[j];
			V_tmp3[i] += J->pse_trans[i][j]*D->h[j];
		}
		h[i] = V_tmp2[i] + V_tmp3[i];
	}


	double torq_tmp[3]={0.0};

	for(int i=0;i<3;i++)
		torq_tmp[i] = ( h[i] - X_d[i] - X[i]);// +  F[i] ;//- nullSpaceVector[i];

	for(int i=0;i<3;i++){
		torq[i] = 0;
		for(int j=0;j<3;j++){
			torq[i] += J->trans[i][j]*torq_tmp[j];
		}
	}

	for(int i=0;i<3;i++)	torq2[i] = torq[i] + nullSpaceVector[i];
		

	//デバック
	//printf("%lf %lf %lf\n",nullSpaceVector[0], nullSpaceVector[1], nullSpaceVector[2]);
	//printf("%lf %lf %lf\n",redundancyAcc[0], redundancyAcc[1], redundancyAcc[2]);
	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("%lf\t",jacobData->diff[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}


/********************************************************
関数名：Calculate_NullSpace
説明　：冗長性利用におけるゼロ空間ベクトルの計算( (I-J#J)k )
引数　：double jointVal[3][2][3]	関節の角度，角速度，角加速度
		double null_space[3][3]		ヤコビ行列関係の構造体
		double nullSpaceVector[3]	ゼロ空間ベクトル
		
出力　：double nullSpaceVector[3]
********************************************************/
void Calculate_NullSpace(double jointVal[3][2][3], struct jacob_data *jacobData, double nullSpaceVector[3]){

	double eta[3];		//ゼロ空間ベクトルの任意係数η
	
	//ゼロ空間の計算( (I-J#J) )
	NullSpace(jacobData);
	
	//ゼロ空間ベクトルのための評価関数の計算( k )
	EvaluationFunc_NullSpace(jointVal[POS][R], eta);

	//ゼロ空間ベクトルの計算( (I-J#J)k )
	//Calculate_NullSpaceVector(jacobData->null_space, eta, nullSpaceVector);


	//デバック
	//printf("%lf %lf %lf\n",eta[0], eta[1], eta[2]);
	//printf("%lf %lf %lf\n",redundancyAcc[0], redundancyAcc[1], redundancyAcc[2]);
	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("%lf\t",jacobData->diff[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/

}



/********************************************************
関数名：Calculate_UsingRedundancy
説明　：関節角加速度に関する冗長性利用を計算する関数
		メインループ内で使用
引数　：double jointVal[2][3]			エンコーダの値を元にした位置，速度
		double targetAcc[3]				認知アシストにより与えられる加速度
		struct jacob_data jacobData		ヤコビ行列関係の構造体
		double redundancyAcc[3]			冗長性利用プログラムにより得られた関節角加速度
		
出力　：double redundancyAcc[3]
********************************************************/
void Calculate_UsingRedundancy(double jointVal[3][2][3], double targetAcc[3], struct jacob_data *jacobData, double redundancyAcc[3]){

	double eta[3];
	double nullSpaceVector[3];
	
	//ゼロ空間の計算( (I-J#J) )
	NullSpace(jacobData);
	
	//ゼロ空間ベクトルのための評価関数の計算( k )
	EvaluationFunc_NullSpace(jointVal[POS][R], eta);

	//ゼロ空間ベクトルの計算( (I-J#J)k )
	//	Calculate_NullSpaceVector(jacobData->null_space, eta, nullSpaceVector);

	//冗長性利用後の各関節の角加速度の計算(  d2q = J#(d2r-dJdq) + (I-J#J)k  )
	Calculate_JointAcc_UsingRedundancy(targetAcc, jointVal[VEL][R], jacobData, nullSpaceVector, redundancyAcc);

	//デバック
	//printf("%lf %lf %lf\n",eta[0], eta[1], eta[2]);
	//printf("%lf %lf %lf\n",redundancyAcc[0], redundancyAcc[1], redundancyAcc[2]);
	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("%lf\t",jacobData->diff[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}


/********************************************************
関数名：NullSpace
説明　：冗長性利用におけるゼロ空間の計算(この行列がゼロ行列でなければ冗長性が利用できる)
引数　：struct jacob_data jacobData		ヤコビ行列関係の構造体
		
出力　：struct jacob_data jacobData
********************************************************/
void NullSpace(struct jacob_data *jacobData){

	double I[3][3];		//単位行列
	double M[3][3];		//一時保存のための行列
	double N[3][3];

	//単位行列の計算
	Matrix_Unit(I);

	//(J#)(J)の計算
	//Matrix33_Product(jacobData->pse, jacobData->matrix, M);
	
	//I-J#Jの計算
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			jacobData->null_space[i][j] = I[i][j]-M[i][j];

	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,jacobData->null_space[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/

	
}


///********************************************************
//関数名：NullSpace
//説明　：冗長性利用におけるゼロ空間のための評価関数の計算
//引数　：struct jacob_data jacobData		ヤコビ行列関係の構造体
//		
//出力　：struct jacob_data jacobData
//********************************************************/
//void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]){
//
//	double enc_min[3] = { 50, -60, -12.5 };							//各関節の関数の最小値
//	for(int i=0;i<3;i++)	enc_min[i] = enc_min[i]*PI/180;		//deg→rad
//	
//	//重み　ここを変更することで各関節トルクのウエイト値が変更されエネルギー効率を考慮することになる
//	double Coeff[3][3] = { 
//							{100.0, 0.0, 0.0},
//							{0.0, 0.0, 0.0},
//							{0.0, 0.0, 100.0}  };
//	
//	//各関節と評価関数の関係
//	double C[3] = {   (1.0/10)*pow( enc_data[H]-enc_min[H], 10.0),
//					  (1.0/25)*pow(-enc_data[K]+enc_min[K], 10.0),		//膝だけエンコーダの向きが逆なので注意
//					(1.0/0.01)*pow( enc_data[A]-enc_min[A], 10.0) };
//	//Cを各関節角度で偏微分した値
//	double C_diff[3] = { (10.0/10)*pow( enc_data[H]-enc_min[H], 9.0),
//						 -(10.0/25)*pow(-enc_data[K]+enc_min[K], 9.0),		//膝だけエンコーダの向きが逆なので注意
//						 (10.0/0.01)*pow( enc_data[A]-enc_min[A], 9.0) };
//
//	//評価関数
//	double E = ( C[H] + C[K] + C[A] );
//
//	//評価関数を各関節角度で偏微分した値
//	double E_diff[3];
//	for(int i=0;i<3;i++)	E_diff[i] = C_diff[i];
//
//	for(int i=0;i<3;i++){
//		eta[i] = 0.0;
//		for(int j=0;j<3;j++){
//			eta[i] += -Coeff[i][j]*E_diff[j];
//		}
//	}
//
//
//	//デバック用
//	//printf("%lf %lf %lf\n",C_diff[0], C_diff[1], C_diff[2]);
//	printf("%lf %lf %lf\n",eta[0], eta[1], eta[2]);
//}



/********************************************************
関数名：NullSpace
説明　：冗長性利用におけるゼロ空間のための評価関数の計算
引数　：struct jacob_data jacobData		ヤコビ行列関係の構造体
		
出力　：struct jacob_data jacobData
********************************************************/
void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]){

	double enc_min[3] = { 20, -50, -12.5 };							//各関節の関数の最小値
	for(int i=0;i<3;i++)	enc_min[i] = enc_min[i]*PI/180;		//deg→rad
	
	//重み　ここを変更することで各関節トルクのウエイト値が変更されエネルギー効率を考慮することになる
	double Coeff[3][3] = { 
							{1.0, 0.0, 0.0},
							{0.0, 1.0, 0.0},
							{0.0, 0.0, 10.0}  };
	
	//各関節と評価関数の関係
	double C[3] = {   (1.0/2)*pow( enc_data[H]-enc_min[H], 10.0),
					  (1.0/150)*pow(-enc_data[K]+enc_min[K], 10.0),		//膝だけエンコーダの向きが逆なので注意
					(1.0/0.1)*pow( enc_data[A]-enc_min[A], 10.0) + (1.0/2.0)*pow( enc_data[A]-15*PI/180, 2.0 ) };
	//Cを各関節角度で偏微分した値
	double C_diff[3] = { (10.0/2)*pow( enc_data[H]-enc_min[H], 9.0),
						 -(10.0/150)*pow(-enc_data[K]+enc_min[K], 9.0),		//膝だけエンコーダの向きが逆なので注意
						 (10.0/0.1)*pow( enc_data[A]-enc_min[A], 9.0) + (2.0/2.0)*pow( enc_data[A]-15*PI/180, 1.0 ) };

	//評価関数
	double E = ( 1 / ( C[H] + C[K] + C[A] + 1 ) );

	//評価関数を各関節角度で偏微分した値
	double E_diff[3];
	for(int i=0;i<3;i++)	E_diff[i] = -C_diff[i]/(E*E);

	for(int i=0;i<3;i++){
		eta[i] = 0.0;
		for(int j=0;j<3;j++){
			eta[i] += Coeff[i][j]*E_diff[j];
		}
	}

	//for(int i=0;i<3;i++)	eta[i] = 0.0;


	//デバック用
	//printf("%lf %lf %lf\n",C_diff[H], C_diff[K], C_diff[A]);
	//printf("%lf %lf %lf %lf\n",eta[0], eta[1], eta[2], E);
}


///********************************************************
//関数名：NullSpace
//説明　：冗長性利用におけるゼロ空間のための評価関数の計算
//引数　：struct jacob_data jacobData		ヤコビ行列関係の構造体
//		
//出力　：struct jacob_data jacobData
//********************************************************/
//void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]){
//
//	double enc[3];
//	for(int i=0;i<3;i++)	enc[i] = enc_data[i]*180/PI;
//
//	//重み　ここを変更することで各関節トルクのウエイト値が変更されエネルギー効率を考慮することになる
//	double Coeff[3] = { (5.0*pow(10.0,4.0)), (1.0*pow(10.0,5.0)), (2.0*pow(10.0,6.0)) };
//	
//	//各関節と評価関数の関係
//	double C[3] = { pow(10.0, -6.0)*pow( enc[H]-55.0, 4.0),
//					pow(10.0, -6.0)*pow(-enc[K]+65.0, 4.0),		//膝だけエンコーダの向きが逆なので注意
//					pow(10.0, -5.0)*pow( enc[A]+15.0, 4.0) };
//	//Cを各関節角度で偏微分した値
//	double C_diff[3] = { -4.0*pow(10.0, -6.0)*pow( enc[H]-55.0, 3.0),
//						  4.0*pow(10.0, -6.0)*pow(-enc[K]+65.0, 3.0),		//膝だけエンコーダの向きが逆なので注意
//						 -4.0*pow(10.0, -5.0)*pow( enc[A]+15.0, 3.0) };
//
//	//評価関数
//	double E = 1/( C[H] + C[K] + C[A] + 1 );
//
//	//評価関数を各関節角度で偏微分した値
//	double E_diff[3];
//	for(int i=0;i<3;i++)	E_diff[i] = -1.0*C_diff[i]*pow(E, 2.0);
//
//	for(int i=0;i<3;i++)	eta[i] = Coeff[i]*E_diff[i];
//
//}


/********************************************************
関数名：NullSpace
説明　：冗長性利用におけるゼロ空間ベクトルの計算
引数　：double null_space[3][3]		ヤコビ行列関係の構造体
		double eta[3]				評価関数
		double vector[3]			ゼロ空間ベクトル
		
出力　：struct jacob_data jacobData
********************************************************/
void Calculate_NullSpaceVector(double null_space[3][3], double eta[3], double vector[3]){

	for(int i=0;i<3;i++){
		vector[i] = 0;
		for(int j=0;j<3;j++){
			vector[i] += null_space[i][j]*eta[j];
		}
	}

	double vector2=0.0;
	for(int i=0;i<3;i++)
		vector2 += vector[i]*eta[i];
		
	//printf("%lf\n",vector2);

	//デバック用
	//printf("%lf %lf %lf\n",eta[H], eta[K], eta[A]);
	//printf("%lf %lf %lf\n",vector[H], vector[K], vector[A]);
	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,null_space[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}


/********************************************************
関数名：Calculate_JointAcc_UsingRedundancy
説明　：冗長性を利用してサブタスクを与えた各関節の角加速度を計算する関数
引数　：double targetAcc[3]				認知アシストにより与えられる加速度
		double jointVel[3]				現在の関節速度
		struct jacob_data jacobData		ヤコビ行列関係の構造体
		double nullSpaceVector[3]		ゼロ空間ベクトル
		double jointVal[3][3]			冗長性利用プログラムにより得られた関節角速度と角加速度
		
出力　：double jointVal[3][3]
********************************************************/
void Calculate_JointAcc_UsingRedundancy(double targetAcc[3], double jointVel[3], struct jacob_data *jacobData, double nullSpaceVector[3], double jointAcc[3]){

	double V_tmp1[3]={0}, V_tmp2[3]={0};		//一時保存のためのベクトル
	

	//冗長性を利用した関節角加速度の計算
	//d2q = J#(d2r-dJdq) + (I-J#J)k
	for(int i=0;i<3;i++){
		jointAcc[i] = 0;
		for(int j=0;j<3;j++){
			//dJdq
			for(int k=0;k<3;k++){
				V_tmp1[j] += jacobData->diff[j][k]*jointVel[k];
			}
			//J#(d2r-dJdq)
			V_tmp2[i] += jacobData->pse[i][j]*(targetAcc[j] - V_tmp1[j]);
		}
		//d2q = J#(d2r-dJdq) + (I-J#J)k
		jointAcc[i] = V_tmp2[i] + nullSpaceVector[i];
	}


	//デバック用
	//printf("%lf %lf %lf\n",jointAcc[H], jointAcc[K], jointAcc[A]);
	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,jacobData->pse[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}


/********************************************************
関数名：Matrix_Unit
説明　：3×3の単位行列の計算
引数　：double I[3][3]		3×3の単位行列
		
出力　：double I[3][3]
********************************************************/
void EnergyMinimization(double enc_data[2][3], struct leg_state *Leg, struct jacob_data *jacobData, double torq[3], double force[3]){
	
	//地面（x-y座標系）に対しての角度なので注意
	double theta[3];
	double eta[3];
	double force_tmp[3];
	double force_null[3] = {0,0,0};
	double force_image[3] = {0,0,0};
	double force_image_tmp[3] = {0,0,0};
	double body_ang = 0;
	Calculation_Swing_Angle(enc_data, Leg, theta, body_ang);

	//ゼロ空間ベクトルのための評価関数の計算
	NullSpace_TargetFunc(theta, eta);

	//ゼロ空間ベクトルの計算
	NullSpaceVector(force_tmp, *jacobData, eta);

	/*for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			force_image[i] += jacobData->pse_trans[i][j] * torq[j];

	for(int i=0;i<3;i++)	force[i] = force_image[i] + force_null[i];*/

	//
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			force_image_tmp[i] += jacobData->pse_trans[i][j] * force[j];
			force_image[i] += jacobData->trans[i][j] * force[j];
			force_null[i]  += jacobData->trans[i][j] * force_tmp[j];
		}
	}

	for(int i=0;i<3;i++)	force[i] = force_image_tmp[i] + force_tmp[i];
	//force[X] = -force[X];
	for(int i=0;i<3;i++)	torq[i] = force_image[i] + force_null[i];

	/*for(int i=0;i<3;i++)	printf("%lf\t",force[i]);
	printf("%lf\n",sqrt(pow(force[X],2)+pow(force[Y],2)+pow(force[Z],2)));*/

	/*for(int i=0;i<3;i++)	force[i] = force_image[i] * force_null[i];

	for(int i=0;i<3;i++)	printf("%lf\t",force_image[i]);
	printf("\n");
	for(int i=0;i<3;i++)	printf("%lf\t",force_null[i]);
	printf("\n");
	for(int i=0;i<3;i++)	printf("%lf\t",force[i]);
	printf("\n");
	printf("\n");*/

}


//関節可動域の計算　ELim：伸展側の制限角度、FLim：伸展側の制限角度
int Cal_JointLimit(double enc[2][3], double theta_ELim[2][3], double theta_FLim[2][3]){

	double enc_ang[2][3];		//エンコーダのデータをdegに直したもの。

	for(int i=0; i<2; i++){
		for(int j=0; j<2; j++){
			enc_ang[i][j] = enc[i][j]/PI*180;		//radからdegに変換
		}
	}

	double Krammel,
			Krammel1,
			Krammel2;		//クラメル表記法　<A>=A (A>0のとき), <A>=0 (A<0のとき)

	//膝はエンコーダ計測値が屈曲側が負となってるので符号に注意

	//****** 股間節 ******
	//*** 伸展 ***
	//右
	Krammel = -enc_ang[R][K]-(93.49-0.316*AGE);
	if(Krammel<0) Krammel =0;
	theta_ELim[R][H] = 0.282*Krammel+0.308*AGE-40.62;

	//左
	Krammel = -enc_ang[L][K]-(93.49-0.316*AGE);
	if(Krammel<0) Krammel =0;
	theta_ELim[L][H] = 0.282*Krammel+0.308*AGE-40.62;
	
	//*** 屈曲 *** 
	//右
	Krammel = (84.07+0.135*AGE)+enc_ang[R][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[R][H] = -0.733*Krammel-0.183*AGE+144.7;
	//左
	Krammel = (84.07+0.135*AGE)+enc_ang[L][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[L][H] = -0.733*Krammel-0.183*AGE+144.7;


	//***** 膝関節 *****
	//*** 伸展 ***
	//右
	Krammel1 = enc_ang[R][H]-(81.06-0.282*AGE);
	if(Krammel1<0) Krammel1 =0;
	Krammel2 = enc_ang[R][A]-23;
	if(Krammel2<0) Krammel2 =0;
	if(Krammel1>Krammel2)	theta_ELim[R][K] = 1.364*Krammel1-2.72;
	else					theta_ELim[R][K] = 0.9*Krammel2-2.72;
	//左
	Krammel1 = enc_ang[L][H]-(81.06-0.282*AGE);
	if(Krammel1<0) Krammel1 =0;
	Krammel2 = enc_ang[L][A]-23;
	if(Krammel2<0) Krammel2 =0;
	if(Krammel1>Krammel2)	theta_ELim[L][K] = 1.364*Krammel1-2.72;	
	else					theta_ELim[L][K] = 0.9*Krammel2-2.72;
	//*** 屈曲 ***
	//右
	Krammel = (-19.85+0.397*AGE)-enc_ang[R][H];
	if(Krammel<0) Krammel =0;
	theta_FLim[R][K] = -3.546*Krammel+167.14;
	//左
	Krammel = (-19.85+0.397*AGE)-enc_ang[L][H];
	if(Krammel<0) Krammel =0;
	theta_FLim[L][K] = -3.546*Krammel+167.14;

	//***** 足関節 *****
	//*** 屈曲 ***
	theta_ELim[R][A] = -55; 
	theta_ELim[L][A] = -55; 	
	//*** 伸展 ***
	//右
	Krammel = 26+enc_ang[R][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[R][A] = -1.1*Krammel+46;
	//左
	Krammel = 26+enc_ang[L][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[L][A] = -1.1*Krammel+46;


	//degからradに変換
	for(int i=0; i<2; i++){
		for(int j=0; j<3; j++){
			theta_ELim[i][j] = theta_ELim[i][j]*PI/180;
			theta_FLim[i][j] = theta_FLim[i][j]*PI/180;
		}
	}

return 0;
}

void CalculateEnergyMinimization(double enc_data[2][3], double eta[6][2]){
	double theta_ELim[2][3];	//伸展方向の制限角度
	double theta_ELim6[6];			//便宜上ベクトルに直したもの
	double theta_FLim[2][3];	//屈曲方向の制限角度
	double theta_FLim6[6];			//便宜上ベクトルに直したもの

	double theta0[6];				//自然状態の関節角度

	int sup=0;
	int swi=0;

	double ang[6];

	//人の屈曲方向を正（膝だけエンコーダの関係で負としてあるので注意）
	ang[0]=enc_data[sup][A];
	ang[1]=-enc_data[sup][K];
	ang[2]=enc_data[sup][H];
	ang[3]=enc_data[swi][H];
	ang[4]=-enc_data[swi][K];
	ang[5]=enc_data[swi][A];


	//*** 関節可動域の計算 ***
	Cal_JointLimit(enc_data, theta_ELim, theta_FLim);

	/***	関節可動域を考慮した評価関数の作成		***/
	//自然状態の関節角度
	theta0[0]=0;												//足関節
	theta0[1]=(theta_FLim[sup][K]+theta_ELim[sup][K])/2;		//膝関節
	theta0[2]=0;												//股間節
	theta0[3]=0;												//股間節
	theta0[4]=(theta_FLim[swi][K]+theta_ELim[swi][K])/2;		//膝関節
	theta0[5]=0;												//足関節

	//伸展，屈曲方向の制限角度
	theta_ELim6[0] = theta_ELim[sup][A] ;	theta_FLim6[0] = theta_FLim[sup][A] ;
	theta_ELim6[1] = theta_ELim[sup][K] ;	theta_FLim6[1] = theta_FLim[sup][K] ;
	theta_ELim6[2] = theta_ELim[sup][H] ;	theta_FLim6[2] = theta_FLim[sup][H] ;
	theta_ELim6[3] = theta_ELim[swi][H] ;	theta_FLim6[3] = theta_FLim[swi][H] ;
	theta_ELim6[4] = theta_ELim[swi][K] ;	theta_FLim6[4] = theta_FLim[swi][K] ;
	theta_ELim6[5] = theta_ELim[swi][A] ;	theta_FLim6[5] = theta_FLim[swi][A] ;


	double Etatmp=0;
	double eps=pow(2.0,-30);		//微小量
	
	double ke;								//評価関数の偏微分の係数　メインタスクの項のオーダーによって決める	

	//*** ηの計算 ***
	for(int i=0; i<6; i++){
		if(i==1 || i==4)	ke=100000;		//膝だけηが大きくなるので、小さくする
		else				ke=100;			//膝以外
		//伸展側
		if(ang[i]<=theta0[i]){
//			if(i==2)printf("伸展\t");
			Etatmp = cos(ang[i]-theta_ELim6[i]+PI/2);	
			if(fabs(Etatmp) < eps )	eta[i][X] = pow(10.0, 6);		//無限大に発散してしまうので、10^3で代替(モータ出力の飽和を考慮）
			else{
				eta[i][X] = fabs(1/( (cos(ang[i]-theta_ELim6[i]+PI/2))*(cos(ang[i]-theta_ELim6[i]+PI/2)) ) ) * (ang[i]-theta0[i])*(ang[i]-theta0[i])  + 2*fabs(tan(ang[i]-theta_ELim6[i]+PI/2)) * (ang[i]-theta0[i]);
				eta[i][X] /= -ke;
			}
		}
		//屈曲側
		else if(ang[i] > theta0[i]){
			Etatmp = cos(ang[i]-theta_FLim6[i]-PI/2);
			if(fabs(Etatmp) < eps )	eta[i][X] = pow(10.0, 6);		//無限大に発散してしまうので、10^8で代替
			else{
				eta[i][X] = fabs(1/( (cos(ang[i]-theta_FLim6[i]-PI/2))*(cos(ang[i]-theta_FLim6[i]-PI/2)) ) ) * (ang[i]-theta0[i])*(ang[i]-theta0[i]) + 2*fabs(tan(ang[i]-theta_FLim6[i]-PI/2)) * (ang[i]-theta0[i]);
				eta[i][X] /= -ke;
			}
		}
	}
	/***	関節可動域を考慮した評価関数の作成　ここまで	***/

}


void NullSpace_TargetFunc(double enc_data[3], double eta[3]){

	double enc[3];
	for(int i=0;i<3;i++)	enc[i] = enc_data[i]*180/PI;

	//重み　ここを変更することで各関節トルクのウエイト値が変更されエネルギー効率を考慮することになる
	double Coeff[3] = { (5.0*pow(10.0,4.0)), (1.0*pow(10.0,5.0)), (2.0*pow(10.0,6.0)) };
	
	//各関節と評価関数の関係
	double C[3] = { pow(10.0, -6.0)*pow( enc[H]-55.0, 4.0),
					pow(10.0, -6.0)*pow(-enc[K]+65.0, 4.0),		//膝だけエンコーダの向きが逆なので注意
					pow(10.0, -5.0)*pow( enc[A]+15.0, 4.0) };
	//Cを各関節角度で偏微分した値
	double C_diff[3] = { -4.0*pow(10.0, -6.0)*pow( enc[H]-55.0, 3.0),
						  4.0*pow(10.0, -6.0)*pow(-enc[K]+65.0, 3.0),		//膝だけエンコーダの向きが逆なので注意
						 -4.0*pow(10.0, -5.0)*pow( enc[A]+15.0, 3.0) };

	//評価関数
	double E = 1/( C[H] + C[K] + C[A] + 1 );

	//評価関数を各関節角度で偏微分した値
	double E_diff[3];
	for(int i=0;i<3;i++)	E_diff[i] = -1.0*C_diff[i]*pow(E, 2.0);

	for(int i=0;i<3;i++)	eta[i] = Coeff[i]*E_diff[i];
	
}

void NullSpaceVector(double force_null[3], struct jacob_data jacobData, double eta[3]){

	double I[3][3];
	double T[3][3];
	double jacob_tmp[3][3];
	double a[3] = {0,50,0};

	for(int i=0;i<3;i++)	force_null[i] = 0;

	//単位行列の計算
	Matrix_Unit(I);

	//(J#)(J)の計算
	//Matrix33_Product(jacobData.trans,jacobData.pse_trans,jacob_tmp);
	
	//I-J#Jの計算
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			T[i][j] = I[i][j]-jacob_tmp[i][j];
	/*double A[3][3];
	Matrix33_Product(T,jacobData.matrix,A);*/

	for(int i=0;i<3;i++){
		force_null[0] += T[i][0]*eta[i];
		force_null[1] += T[i][1]*eta[i];
		force_null[2] += T[i][2]*eta[i];
	}

	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,jacob_tmp[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/

	/*for(int i=0;i<3;i++)	printf("%lf\t",force_null[i]);
	printf("\n");*/
}







/****************************		動力学関係		*****************************************************************************************************/


/********************************************************
関数名：SetDynamicsParam
説明　：動力学用のパラメータを設定（D-Hパラメータ，擬似慣性行列など）
引数　：double encData[2][3]				 各関節のエンコーダの値
		struct transfer_data *transfer_data  同次変換行列用の構造体
		
出力　：struct transfer_data *transfer_data
********************************************************/
void Calculate_Dynamics(double angData[3][2][3], double targetAcc[3], struct transfer_data *transferData, struct dynamic_data *dynamicData, double torq[3]){

	//D-Hパラメータや擬似慣性行列のパラメータ設定
	SetDynamicsParam(angData[POS], transferData);

	//同次変換行列の計算(0Ti)
	Matrix44_Transfer(transferData);

	//同次変換行列の1階，2階の偏微分計算
	Matrix44_Diff(transferData);
	Matrix44_Diff2(transferData);


	//動力学の計算
	//M:慣性行列，H：コリオリ力・遠心力ベクトル，G：重力ベクトル
	Matrix_Mass(transferData, dynamicData);
	Vector_H(angData, transferData, dynamicData);		
	Vector_Gravity(angData[POS], transferData, dynamicData);
	//simi; here onwards is not there in ayato prog
	double M_torq[3] = {0.0};
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			M_torq[i] += dynamicData->mass[i][j]*targetAcc[j];
			//M_torq[i] += dynamicData->mass[i][j]*angData[ACC][R][j];

	//for(int i=0;i<3;i++)	torq[i] = M_torq[i] + dynamicData->h[i] + dynamicData->gra[i];
	for(int i=0;i<3;i++)	torq[i] = M_torq[i] + dynamicData->h[i];

	//デバック用
	/*printf("%lf %lf %lf\n", M_torq[H], M_torq[K], M_torq[A]);
	printf("%lf %lf %lf\n", targetAcc[H], targetAcc[K], targetAcc[A]);
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,dynamicData->mass[i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}


/********************************************************
関数名：SetDynamicsParam
説明　：動力学用のパラメータを設定（D-Hパラメータ，擬似慣性行列など）
引数　：double encData[2][3]				 各関節のエンコーダの値
		struct transfer_data *transfer_data  同次変換行列用の構造体
		
出力　：struct transfer_data *transfer_data
********************************************************/
void SetDynamicsParam(double encData[2][3], struct transfer_data *transfer_data){

	const int SW=R;
	double q[3];
	for(int i=0;i<3;i++)	q[i] = encData[SW][i];

	//D-Hパラメータの設定
	double DH_P[LINK_NUM][4] = { 
									{0      , 0, 0, q[0]-PI/2},
									{LEN_H_K, 0, 0, q[1]	 },
									{LEN_K_A, 0, 0, q[2]+PI/2}
							   };
	//i-1Tiの同次変換行列の計算
	double T[LINK_NUM][4][4];
	for(int i=0; i<LINK_NUM; i++){
		T[i][0][0] = cos(DH_P[i][3])				;	T[i][0][1] =-sin(DH_P[i][3])				;	T[i][0][2] = 0				;	T[i][0][3] = DH_P[i][0]				   ;
		T[i][1][0] = cos(DH_P[i][1])*sin(DH_P[i][3]);	T[i][1][1] = cos(DH_P[i][1])*cos(DH_P[i][3]);	T[i][1][2] =-sin(DH_P[i][1]);	T[i][1][3] =-DH_P[i][2]*sin(DH_P[i][1]);
		T[i][2][0] = sin(DH_P[i][1])*sin(DH_P[i][3]);	T[i][2][1] = sin(DH_P[i][1])*cos(DH_P[i][3]);	T[i][2][2] = cos(DH_P[i][1]);	T[i][2][3] = DH_P[i][2]*cos(DH_P[i][1]);
		T[i][3][0] = 0								;	T[i][3][1] = 0								;	T[i][3][2] = 0				;	T[i][3][3] = 1						   ;
	}


	/***  擬似慣性行列のパラメータ設定  ***/
	double link[LINK_NUM] = {LEN_H_K, LEN_K_A, LEN_A_T};
	double mass[LINK_NUM] = {MASS_R_THIGH, MASS_R_LOWER, MASS_R_FOOT};
	double H[LINK_NUM][4][4];

	//遊脚　[0]:股関節-膝リンク　[1]:膝-足首リンク　[2]:足首-つま先リンク
	for(int i=0;i<LINK_NUM;i++){
		H[i][0][0] = link[i]*link[i]/3;		H[i][0][1] = 0;		H[i][0][2] = 0;		H[i][0][3] = link[i]/2;
		H[i][1][0] = 0;						H[i][1][1] = 0;		H[i][1][2] = 0;		H[i][1][3] = 0;
		H[i][2][0] = 0;						H[i][2][1] = 0;		H[i][2][2] = 0;		H[i][2][3] = 0;
		H[i][3][0] = link[i]/2;				H[i][3][1] = 0;		H[i][3][2] = 0;		H[i][3][3] = 1;
	}


	//データの保存
	for(int i=0;i<LINK_NUM;i++){
		for(int j=0;j<4;j++){
			for(int k=0;k<4;k++){
				transfer_data->T[i][j][k] = T[i][j][k];
				transfer_data->H[i][j][k] = mass[i]*H[i][j][k];
			}
		}
	}
}


/********************************************************
関数名：Matrix44_transfer
説明　：0からiリンクまでの同次変換行列の計算
引数　：struct transfer_data *T  同次変換行列用の構造体
		
出力　：struct transfer_data *T
********************************************************/
void Matrix44_Transfer(struct transfer_data *T){

	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)
			T->T0[0][i][j] = T->T[0][i][j];

	for(int i=1;i<LINK_NUM;i++)
		Matrix44_Product(T->T0[i-1],T->T[i],T->T0[i]);


	/*for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			printf("[%d][%d]%lf\t",i,j,T->T0[1][i][j]);
		}
		printf("\n");
	}
	printf("\n");*/
}


/********************************************************
関数名：Matrix44_Diff
説明　：同次変換行列を関節iで1階偏微分
引数　：struct transfer_data *T  同次変換行列用の構造体
		
出力　：struct transfer_data *T
********************************************************/
void Matrix44_Diff(struct transfer_data *T){
	double T_diff_tmp[4][4];
	//偏微分のための行列（関節iが回転行列のとき）
	double Q[4][4]={ 
		{0, -1, 0, 0},
		{1,  0, 0, 0},
		{0,  0, 0, 0},
		{0,  0, 0, 0}
	};

	//i: 0からi番目までの同時変換
	for(int i =0; i<LINK_NUM; i++){
		//j: 関節jで偏微分する
		for(int j=0; j<=i; j++){
			//１階微分（0からリンクｊの微分成分Qまで）
			Matrix44_Product(T->T0[j], Q, T->T_diff[i][j]);

			if(j<i){
				//1階微分Qをかけた残りの部分
				for(int h=j+1; h<=i; h++){
					//一時保存、更新
					for(int k=0; k<4; k++){
						for(int l=0; l<4; l++){
							T_diff_tmp[k][l] = T->T_diff[i][j][k][l];
						}
					}
					
					Matrix44_Product(T_diff_tmp, T->T[h], T->T_diff[i][j]);
				}
			}
		}
	}


	for(int i=0;i<LINK_NUM;i++)
		for(int j=0;j<LINK_NUM;j++)
			Matrix44_Trans(T->T_diff[i][j],T->T_diff_trans[i][j]);

}


/********************************************************
関数名：Matrix44_Diff2
説明　：同次変換行列を関節iで2階偏微分
引数　：struct transfer_data *T  同次変換行列用の構造体
		
出力　：struct transfer_data *T
********************************************************/
void Matrix44_Diff2(struct transfer_data *T){

	double T_diff2_tmp[4][4];
	//偏微分のための行列（関節iが回転行列のとき）
	double Q[4][4]={ 
		{0, -1, 0, 0},
		{1,  0, 0, 0},
		{0,  0, 0, 0},
		{0,  0, 0, 0}
	};

	//********* ２階偏微分(qのjとm(j<=m)で偏微分) ************
		//i: 0からi番目までの同時変換
	for(int i =0; i<LINK_NUM; i++){
		//j: 関節jで偏微分する
		for(int j=0; j<=i; j++){
		//２階微分(j<=m)　m: 関節mで偏微分する
			for(int m=j; m<=i; m++){
				//jの偏微分成分まで計算
				Matrix44_Product(T->T0[j], Q, T_diff2_tmp);
				//jとmの間の計算
				for(int h=j+1; h<=m; h++){
					Matrix44_Product(T_diff2_tmp, T->T[h], T->T_diff2[i][j][m]);
					//更新
					for(int k=0; k<4; k++){
						for(int l=0; l<4; l++){
							T_diff2_tmp[k][l] = T->T_diff2[i][j][m][k][l];
						}
					}
				}
				//mの偏微分成分まで計算
				Matrix44_Product(T_diff2_tmp, Q, T->T_diff2[i][j][m]);
				//残りの部分の計算（あれば）
				for(int h=m+1; h<=i; h++){
					//更新
					for(int k=0; k<4; k++){
						for(int l=0; l<4; l++){
							T_diff2_tmp[k][l] = T->T_diff2[i][j][m][k][l];	
						}			
					}					
					Matrix44_Product(T_diff2_tmp, T->T[h], T->T_diff2[i][j][m]);
				}	
			}
			// mとjは可換(m>jのときでもok)
			for(int m=0; m<j; m++){
					for(int k=0; k<4; k++){
						for(int l=0; l<4; l++){
							T->T_diff2[i][j][m][k][l] = T->T_diff2[i][m][j][k][l];	
						
						}			
					}
			}
		}
	}
}


void Matrix_Mass(struct transfer_data *T, struct dynamic_data *D){

	int kmax;
	double H_tmp[4][4]={0};
	double M_tmp[4][4]={0};
	double M_trace;
	//******** 質量行列 Mの計算 *************
	for(int i=0; i<LINK_NUM; i++){
		for(int j=0;j<LINK_NUM; j++){
			D->mass[i][j] = 0;

			if(i>j) kmax=i;
			else    kmax=j;
			for(int k=kmax; k<LINK_NUM; k++){
				//質量行列の計算
				Matrix44_Product(T->T_diff[k][j], T->H[k], H_tmp);
				Matrix44_Product(H_tmp, T->T_diff_trans[k][i], M_tmp);

				Matrix44_Trace(M_tmp, &M_trace);

				D->mass[i][j] += M_trace;							
			
			}
		}
	}
	
	////デバック用
	//for(int i=0;i<4;i++){
	//	for(int j=0;j<4;j++){
	//		printf("[%d][%d]%lf\t",i,j,H_tmp[i][j]);
	//	}
	//	printf("\n");
	//}
	//printf("\n");

}


void Vector_H(double angData[3][2][3], struct transfer_data *T, struct dynamic_data *D){

	const int SW = 0;
	double q[3], qv[3]={0};
	for(int i=0;i<3;i++)	q[i]  = angData[POS][SW][i];
	for(int i=0;i<3;i++)	qv[i] = angData[VEL][SW][i];

	double htmp1[4][4];
	double htmp2[4][4];

	for(int i=0; i<LINK_NUM; i++){
		D->h[i] = 0;
		
		for(int k=i; k<LINK_NUM; k++){//jとlはkをこえていいの？こえたら０か
			for(int j=0;j<=k; j++){
				for(int l=0;l<=k; l++){					
					//(d2T/dqjdl = d2T/dqldqj ：可換）

					Matrix44_Product(T->T_diff2[k][j][l], T->H[k], htmp1);					
					Matrix44_Product(htmp1, T->T_diff_trans[k][i], htmp2);
					
					
					D->h[i] += (htmp2[0][0] + htmp2[1][1] + htmp2[2][2]+ htmp2[3][3]) *qv[j]*qv[l] ;
					
				}
			}
		}
	}

	/*for(int j=0;j<4;j++){
		printf("[%d]%lf\t",j,D->h[j]);
	}
	printf("\n");*/
}


void Vector_Gravity(double encData[2][3], struct transfer_data *T, struct dynamic_data *D){

	//この辺グローバル変数にした方がいいかも
//	double g[6];
	double gbar[4];
	double gtmp1[4];
	double gtmp2[3];
	double lbar[3][4];
	double mass_h[3];
	
	gbar[X]=0;
	gbar[Y]=-GRAVITY;
	gbar[2]=0;
	gbar[3]=0;
	lbar[0][X]=LEN_HG;
	lbar[1][X]=LEN_KG;
	lbar[2][X]=LEN_AG;

	for(int i=0; i<LINK_NUM; i++){
		lbar[i][Y]=0;
		lbar[i][2]=0;		
		lbar[i][3]=1;		
	}
	

	//人の質量（この行列使って慣性行列の計算も少しシンプルにできるよね…まぁとりあえずこのまま）	
	mass_h[0]=MASS_THIGH;
	mass_h[1]=MASS_LOWER;
	mass_h[2]=MASS_FOOT;


	// ************** 重力項 gの計算 ************
	
	for(int i=0; i<LINK_NUM; i++){
		D->gra[i]=0;
		
		for(int j=i; j<LINK_NUM; j++){
			for(int k=0; k<4; k++){
				gtmp1[k]=0;
				for(int l=0; l<4; l++){
					gtmp1[k] += gbar[l]*T->T_diff[j][i][l][k];
				}
			}
			gtmp2[j]=0;
			for(int k=0; k<4; k++){
				gtmp2[j] += gtmp1[k]*lbar[j][k];		
			}

			D->gra[i] += mass_h[j]*gtmp2[j];
			
		}

		D->gra[i] = -D->gra[i];
	
	}

	/*for(int j=0;j<4;j++){
		printf("[%d]%lf\t",j,D->gra[j]);
	}
	printf("\n");*/
}




/****************************		行列計算関係		*****************************************************************************************************/

///////////		3×3行列　ヤコビや擬似逆行列などに使用		///////////

/********************************************************
関数名：Matrix_Unit
説明　：3×3の単位行列の計算
引数　：double I[3][3]		3×3の単位行列
		
出力　：double I[3][3]
********************************************************/
void Matrix_Unit(double I[3][3]){
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			if(i==j) I[i][j] = 1.0;
			else     I[i][j] = 0.0;
		}
	}
}


/********************************************************
関数名：Matrix33_Trans
説明　：3×3行列の転置
引数　：double M[3][3]		　3×3の行列
		double M_trans[3][3]  3×3行列の転置
		
出力　：double M_trans[3][3]
********************************************************/
void Matrix33_Trans(double M[3][3], double M_trans[3][3]){
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			M_trans[j][i] = M[i][j];

}

/********************************************************
関数名：Matrix22_Trans
説明　：2×2行列の転置
引数　：double M[2][2]		　2×2の行列
		double M_trans[2][2]  2×2行列の転置

出力　：double M_trans[2][2]
********************************************************/
void Matrix22_Trans(double M[2][2], double M_trans[2][2]) {

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			M_trans[j][i] = M[i][j];

}

/********************************************************
関数名：Matrix66_Trans
説明　：3×3行列の転置
引数　：double M[6][6]		　6x6の行列
		double M_trans[6][6]  6x6行列の転置
		
出力　：double M_trans[6][6]
********************************************************/
void Matrix66_Trans(double M[6][6], double M_trans[6][6]){
	
	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
			M_trans[j][i] = M[i][j];

}

/********************************************************
関数名：Matrix33_Det
説明　：3×3行列の行列式の計算
引数　：double M[3][3]		　3×3の行列
		
戻り値：return det;		//行列式
********************************************************/
double Matrix33_Det(double M[3][3]){
	double det=0;

	for(int i=0; i<3; i++){
		det += M[0][i] * ( M[1][(i+1)%3]*M[2][(i+2)%3] - M[1][(i+2)%3]*M[2][(i+1)%3] );
	}

	return det;
}


/********************************************************
関数名：Matrix22_Inv
説明　：2×2行列の逆行列の計算
引数　：double M[3][3]		　3×3の行列
		double M_inv[3][3]	  3×3の逆行列（3行，3列には0が入る）

出力　：double M_inv[3][3]	
戻り値：return det;		//行列式
********************************************************/
double Matrix22_Inv(double M[3][3], double M_inv[3][3]){
	double det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
	double M_cof[3][3] = {
		{ M[1][1],-M[1][0],0 },
		{-M[0][1], M[0][0],0 },
		{       0,       0,0 },
	};

	if(det == 0){
		printf("逆行列の計算ができませんでした\n");
		return det;
	}else{
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				M_inv[i][j] = M_cof[i][j]/det;
	}
	return det;
}


/********************************************************
関数名：Matrix33_Inv
説明　：3×3行列の逆行列の計算
引数　：double M[3][3]		　3×3の行列
		double M_inv[3][3]	  3×3の逆行列
		
出力　：double M_inv[3][3]
********************************************************/
int Matrix33_Inv(double M[3][3], double M_inv[3][3]){

	double M_cof[3][3];  //余因子行列
	double det;
	
	M_cof[0][0] =   M[1][1]*M[2][2] - M[1][2]*M[2][1];	  M_cof[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]);   M_cof[0][2] =   M[0][1]*M[1][2] - M[0][2]*M[1][1];
	M_cof[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]);	  M_cof[1][1] =   M[0][0]*M[2][2] - M[0][2]*M[2][0];	M_cof[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]);
	M_cof[2][0] =   M[1][0]*M[2][1] - M[1][1]*M[2][0];	  M_cof[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]);   M_cof[2][2] =   M[0][0]*M[1][1] - M[0][1]*M[1][0];
	
	det = Matrix33_Det(M);

	if(det == 0){
		printf("逆行列の計算ができませんでした\n");
		return -1;
	}else{
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				M_inv[i][j] = M_cof[i][j]/det;
	}

	return 0;

}


/********************************************************
関数名：Matrix33_Product
説明　：3×3行列の積の計算
引数　：double A[3][3]	  3×3の行列(左)
		double B[3][3]	  3×3の行列(右)
		double M[3][3]	  計算後の3×3の行列
		
出力　：double M[3][3]
********************************************************/
void Matrix33_Product(double A[3][3], double B[3][3], double M[3][3]){
	
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			M[i][j]=0;
			for(int k=0; k<3; k++){
				M[i][j] += A[i][k] * B[k][j];			
			}			
		}			
	}
}


/********************************************************
関数名：Matrix33_PseudoInverse
説明　：3×3行列の擬似逆行列の計算(2×3行列)
		M_pse=M_trans×(M×M_trans)_inv
		M×M_pse=I
引数　：double M[3][3]	    3×3の行列(3行目は0)
		double M_pse[3][3]	3×3の擬似逆行列(3列目は0)
		
出力　：double M_pse[3][3]
********************************************************/
int Matrix33_PseudoInverse(double M[3][3], double M_pse[3][3]){

	double det;
	double M_trans[3][3];		//Aの転置行列
	double M_pro[3][3];			
	double M_pro_inv[3][3];

	Matrix33_Trans(M,M_trans);

	Matrix33_Product(M,M_trans,M_pro);

	if(M_pro[0][2]==M_pro[1][2]==M_pro[2][2]==M_pro[2][0]==M_pro[2][1]==0){
		det = Matrix22_Inv(M_pro,M_pro_inv);
	}else{
		printf("擬似逆行列の計算ができませんでした\n");
		return -1;
	}
	
	if(det==0){
		printf("擬似逆行列の計算ができませんでした\n");
		return -1;
	}else{
		Matrix33_Product(M_trans,M_pro_inv,M_pse);
	}

	/*for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			printf("[%d][%d]%lf\t",i,j,M_pse);
		}
		printf("\n");
	}
	printf("\n");*/

	return 0;
}


///////////		4×4行列　同次変換行列や動力学などに使用		///////////

/********************************************************
関数名：Matrix44_Trace
説明　：4×4行列のトレース
引数　：double M[4][4]		　4×4の行列
		double *M_trace		  3×3行列のトレース
		
出力　：double *M_trans
********************************************************/
void Matrix44_Trace(double M[4][4], double *M_trace){

	*M_trace = 0;
	for(int i=0;i<4;i++)	*M_trace += M[i][i];
}


/********************************************************
関数名：Matrix44_Trans
説明　：4×4行列の転置
引数　：double M[4][4]		　4×4の行列
		double M_trans[4][4]  4×4行列の転置
		
出力　：double M_trans[4][4]
********************************************************/
void Matrix44_Trans(double M[4][4], double M_trans[4][4]){
	
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			M_trans[j][i] = M[i][j];

}


/********************************************************
関数名：Matrix44_Product
説明　：4×4行列の積の計算
引数　：double A[4][4]	  4×4の行列(左)
		double B[4][4]	  4×4の行列(右)
		double M[4][4]	  計算後の4×4の行列
		
出力　：double M[4][4]
********************************************************/
void Matrix44_Product(double A[4][4], double B[4][4], double M[4][4]){
	
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			M[i][j]=0;
			for(int k=0; k<4; k++){
				M[i][j] += A[i][k] * B[k][j];			
			}			
		}			
	}
}


//重力補償を計算（ロボットの自重を支える）
void Calculate_Gravity_Compensation(int LegState, struct torq_data *torq, double enc_ang[2][3]){

	double ang_b=0;   
	
	int SW=0;
	int SUP=0;
	
	if(LegState==Stand){
	
		(*torq).gra[R][H] = MASS_R_BODY/2*GRAVITY*LEN_BODY*sin(enc_ang[R][H]-enc_ang[R][K]+enc_ang[R][A]);
		(*torq).gra[R][K] = (*torq).gra[R][H] + MASS_R_THIGH*GRAVITY*LEN_HG*sin(enc_ang[R][K]-enc_ang[R][A]);
		(*torq).gra[R][A] = (*torq).gra[R][K] + MASS_R_LOWER*GRAVITY*LEN_KG*sin(enc_ang[R][A]); 
		
		(*torq).gra[L][H] = MASS_R_BODY/2*GRAVITY*LEN_BODY*sin(enc_ang[L][H]-enc_ang[L][K]+enc_ang[R][A]);
		(*torq).gra[L][K] = (*torq).gra[L][H] + MASS_R_THIGH*GRAVITY*LEN_HG*sin(enc_ang[L][K]-enc_ang[R][A]);
		(*torq).gra[L][A] = (*torq).gra[L][K] + MASS_R_LOWER*GRAVITY*LEN_KG*sin(enc_ang[R][A]); 

		//デバッグ用
		(*torq).gra[R][H] *= 1;
		(*torq).gra[R][K] *= 1;
		(*torq).gra[R][A] *= 1; 
		
		(*torq).gra[L][H] *= 1;
		(*torq).gra[L][K] *= 1;
		(*torq).gra[L][A] *= 1; 

	}

	else{
		if(LegState==SwingR){
			SUP = L;
			SW = R;
		}
		else if(LegState==SwingL){
			SUP = R;
			SW = L;
		}
		ang_b = Calculation_Body_Orientation( enc_ang[SUP]);

		/*(*torq).gra[SW][H] =  MASS_R_THIGH*GRAVITY*LEN_HG*sin( enc_ang[SW][H]-ang_b )
			                +(MASS_R_LOWER+MASS_R_FOOT)*GRAVITY*LEN_H_K*sin( enc_ang[SW][H]-ang_b ) + LEN_KG*MASS_R_LOWER*sin( enc_ang[SW][H]-ang_b+enc_ang[SW][K] );
							+ MASS_R_FOOT*GRAVITY*LEN_K_A*sin( enc_ang[SW][H]-ang_b+enc_ang[SW][K] )
							+ MASS_R_FOOT+GRAVITY*LEN_AG*sin( enc_ang[SW][H]-ang_b+enc_ang[SW][K] + enc_ang[SW][A] + (PI/2));
		(*torq).gra[SW][K] = -MASS_R_LOWER*GRAVITY*+LEN_KG*sin( enc_ang[SW][H]-ang_b+enc_ang[SW][K] );
		(*torq).gra[SW][A] =  MASS_R_FOOT+GRAVITY*LEN_AG*( enc_ang[SW][H]-ang_b+enc_ang[SW][K] + enc_ang[SW][A] + (PI/2));

		(*torq).gra[SUP][H] = (*torq).gra[SW][H] +  MASS_R_BODY*GRAVITY*LEN_BODY*sin(ang_b);
		(*torq).gra[SUP][K] = (*torq).gra[SUP][H] + MASS_R_THIGH*GRAVITY*LEN_HG*sin(enc_ang[SUP][K]-enc_ang[SUP][A]);
		(*torq).gra[SUP][A] = (*torq).gra[SUP][K] + MASS_R_LOWER*GRAVITY*LEN_KG*sin(enc_ang[SUP][A]);*/

		(*torq).gra[SW][H] =  MASS_R_THIGH*GRAVITY*LEN_HG*sin( enc_ang[SW][H]-ang_b )
			                +MASS_R_LOWER*GRAVITY*( LEN_HG*sin( enc_ang[SW][H]-ang_b )+LEN_KG*sin( enc_ang[SW][H]-ang_b+enc_ang[SW][K] ) );
		(*torq).gra[SW][K] = -MASS_R_LOWER*GRAVITY*LEN_KG*sin( enc_ang[SW][H]-ang_b+enc_ang[SW][K] );
		(*torq).gra[SW][A] =  MASS_R_FOOT+GRAVITY*LEN_AG*( enc_ang[SW][H]-ang_b+enc_ang[SW][K] + enc_ang[SW][A] + (PI/2));

		(*torq).gra[SUP][H] = (*torq).gra[SW][H] +  MASS_R_BODY*GRAVITY*LEN_BODY*sin(ang_b);
		(*torq).gra[SUP][K] = (*torq).gra[SUP][H] + MASS_R_THIGH*GRAVITY*LEN_HG*sin(enc_ang[SUP][K]-enc_ang[SUP][A]);
		(*torq).gra[SUP][A] = (*torq).gra[SUP][K] + MASS_R_LOWER*GRAVITY*LEN_KG*sin(enc_ang[SUP][A]);

	}

	if((*torq).gra[SW][H]>7.0) (*torq).gra[SW][H] = (*torq).gra[SW][H]-0.67*4.8924;


}
