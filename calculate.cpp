/*****************************************************************************************************************************
******************************************************************************************************************************
���l�v�Z�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��



******************************************************************************************************************************
*****************************************************************************************************************************/

#include "calculate.h"
//#include "myFilter.cpp"


/****************************		���x�E�����x�CZMP�֌W		*****************************************************************************************************/
float time_sum;
float sum_time;

/********************************************************
�֐����FCalculate_Joint_VelAcc
�����@�F�ʒu�f�[�^���瑬�x�C�����x�̌v�Z
�����@�Fconst int count					����
		double pos[2]					�ʒu�f�[�^
		struct motion_data *motion		�ʒu�C���x�C�����x�f�[�^
		
�o��  �Fstruct motion_data *motion	
********************************************************/
void Calculate_Joint_VelAcc(const int count, double q[3][2][3]){//only for joints, not uppThigh,lowThigh and foot
	
	static double pre[3][2][3];
	static float buf_enc[3][2][3][2][FILTER_LEN];
	time_sum += SAMPLING_TIME;
	//���݁i2016/10/24�j�f�[�^�̕��������s���Ă��Ȃ����ߕK�v�ɉ����Ď������Ă�������!! Data smoothing not done
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
	//�f�[�^�̍X�V
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
�֐����FCalculate_VelocityAcceleration
�����@�F�ʒu�f�[�^���瑬�x�C�����x�̌v�Z
�����@�Fconst int count					����
		double pos[2]					�ʒu�f�[�^
		struct motion_data *motion		�ʒu�C���x�C�����x�f�[�^
		
�o��  �Fstruct motion_data *motion	
********************************************************/
void Calculate_VelocityAcceleration(const int count, double pos[2], struct motion_data *motion, float buf[2][2][3][FILTER_LEN]){
	//buf: Body, RUpp,LUpp,...: pos, vel, acc:X, Y

	//���݁i2016/10/24�j�f�[�^�̕��������s���Ă��Ȃ����ߕK�v�ɉ����Ď������Ă�������!! As of (2016/10/24) data smoothing is not performed, please implement as needed !!

	//�f�[�^�̏�����
	if(count == 0){
		for(int i=0;i<2;i++){
			motion->prePos[i] = motion->pos[i];//simi to stop init vel, acc jump
			motion->preVel[i] = motion->vel[i];
			motion->preAcc[i] = motion->acc[i];
		}
	}

	else {

	//���x�C�����x�̌v�Z
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

		//�f�[�^�̍X�V
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
�֐����FCalculate_ZMP
�����@�F�ʒu�f�[�^���瑬�x�C�����x�̌v�Z
�����@�Fconst int count					����
		double pos[2]					�ʒu�f�[�^
		struct motion_data *motion		�ʒu�C���x�C�����x�f�[�^
		
�o��  �Fstruct motion_data *motion	
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



//	**************	CoG�̌v�Z	**************************//
//
//	*****************************************************//
int Calculation_Center_of_Gravity( struct body_posi *body_posi ){

	//��ځA���ځA���̂��ꂼ��̏d�S�̌v�Z(R/L, X,Y)
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
	
	
	//�f�o�b�O�p
	//printf("�ACoG[X] =%lf\t, %lf\t\n ", (*body_posi).CoG[X],(*body_posi).CoG[Y] );

	
	return 0;
}


//	**************	ZMP��CoG�̌v�Z	**************************//
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
�֐����FCalculate_JacobMatrix
�����@�F�Ҋ֐ߊ�̃��R�r�s��̌v�Z
		���C�����[�v���Ŏg�p
�����@�Fdouble enc_data[2][3]		    3�~3�̒P�ʍs��
		struct leg_state *Leg			�V�r�C���r�̔���
		struct jacob_data *jacobData    ���R�r�s��֌W�̍\����
		
�o�́@�Fstruct jacob_data *jacobData
********************************************************/
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state *Leg, struct jacob_data *jacobData){

	//�n�ʁix-y���W�n�j�ɑ΂��Ă̊p�x�Ȃ̂Œ��� Note that the angle is with respect to the ground (x-y coordinate system)
	double q[2][3];
	double dq[2][3];
	double body_ang = 0;
	
	/*[(HK * cos(t1 + t2)) / 2 - HK / 2 + KA * cos(t1 - t2 + t3 - t4 + t5) - KA * cos(t1) - (HK * cos(t1 - t2)) / 2 - (HK * cos(2 * t1)) / 2 + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), (HK * cos(t1 + t2)) / 2 - HK / 2 + KA * cos(t1 - t2 + t3 - t4 + t5) - (HK * cos(t1 - t2)) / 2 - (HK * cos(2 * t1)) / 2 + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), KA * cos(t1 - t2 + t3 - t4 + t5) + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), KA * cos(t1 - t2 + t3 - t4 + t5) + AT * sin(t1 - t2 + t3 - t4 + t5 + t6) + HK * cos(t1 - t2 + t3 - t4), KA * cos(t1 - t2 + t3 - t4 + t5) + AT * sin(t1 - t2 + t3 - t4 + t5 + t6), AT * sin(t1 - t2 + t3 - t4 + t5 + t6)]
	[KA * sin(t1) - (HK * sin(t1 + t2)) / 2 - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4) + (HK * sin(t1 - t2)) / 2 + AT * cos(t1 - t2 + t3 - t4 + t5 + t6) + (HK * sin(2 * t1)) / 2, (HK * sin(t1 - t2)) / 2 - (HK * sin(t1 + t2)) / 2 - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4) + AT * cos(t1 - t2 + t3 - t4 + t5 + t6) + (HK * sin(2 * t1)) / 2, AT * cos(t1 - t2 + t3 - t4 + t5 + t6) - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4), AT * cos(t1 - t2 + t3 - t4 + t5 + t6) - KA * sin(t1 - t2 + t3 - t4 + t5) - HK * sin(t1 - t2 + t3 - t4), AT * cos(t1 - t2 + t3 - t4 + t5 + t6) - KA * sin(t1 - t2 + t3 - t4 + t5), AT * cos(t1 - t2 + t3 - t4 + t5 + t6)]
	[0, 0, 0, 0, 0, 0]
	[0, 0, 0, 0, 0, 0]
	[0, 0, 0, 0, 0, 0]
	[1, 1, 1, 1, 1, 1]*/
	

	//�V�r�̒n�ʊ�̊֐ߊp�x Ground angle of swing leg
	//Calculation_Swing_Angle(angData[POS], Leg, q, body_ang); //simi commented
	for (int j=0;j<3;j++){
		q[R][j] = angData[POS][R][j];//simi added to make R H origin, indep of left leg
		dq[R][j] = angData[VEL][R][j];
		q[L][j] = angData[POS][L][j];
		dq[L][j] = angData[VEL][L][j];
	}
		
	printf("H %lf, K %lf, A %lf\n", q[R][H], q[R][K], q[R][A]); //print cur
	//���R�r�̌v�Z(2�~3)�@�n�ʁix-y����)�ł̍�����R�r�A��(�s��v�Z�̊ȒP���̂���3�~3)
	//Jacobi calculation (2 �~ 5) �� ankle reference Jacobian on the ground (x-y plane) (3 �~ 5 to simplify matrix calculation)
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
	
	////���R�r�̔����̌v�Z(2�~3)�@�n�ʁix-y����)�ł̍�����R�r�A��(�s��v�Z�̊ȒP���̂���3�~5)
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
	// //���R�r�̔����̌v�Z(2�~3)�@�n�ʁix-y����)�ł̍�����R�r�A��(�s��v�Z�̊ȒP���̂���3�~3)
	// jacobData->diff[0][0] =  -LEN_H_K*( dq[H] )*sin( q[H] ) - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	// jacobData->diff[0][1] =                                 - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );       
	// jacobData->diff[0][2] =													                               - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	// jacobData->diff[1][0] =   LEN_H_K*( dq[H] )*cos( q[H] ) + LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );                                     
	// jacobData->diff[1][1] =                                   LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] ); 
	// jacobData->diff[1][2] =                                                                                  LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );
	// jacobData->diff[2][0] = 0.0;
	// jacobData->diff[2][1] = 0.0;
	// jacobData->diff[2][2] = 0.0;

	//���R�r�s��̓]�u�̌v�Z
	//Matrix33_Trans(jacobData->matrix, jacobData->trans);
	Matrix66_Trans(jacobData->matrix, jacobData->trans);
	
	//���R�r�s��̋[���t�s��̌v�Z
	//Matrix33_PseudoInverse(jacobData->matrix, jacobData->pse);

	//�[���t�s��̓]�u�̌v�Z
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
�֐����FCalculate_JacobMatrix
�����@�F�Ҋ֐ߊ�̃��R�r�s��̌v�Z
		���C�����[�v���Ŏg�p
�����@�Fdouble enc_data[2][3]		    3�~3�̒P�ʍs��
		struct leg_state *Leg			�V�r�C���r�̔���
		struct jacob_data *jacobData    ���R�r�s��֌W�̍\����
		struct jacob_data *jacobData    ���R�r�s��֌W�̍\����
		
�o�́@�Fstruct jacob_data *jacobData
********************************************************/
void Calculate_JacobMatrix(double angData[3][2][3], struct leg_state *Leg, struct jacob_hip *jh,struct jacob_body *jb){//sit ankle base- SUP leg

	//�n�ʁix-y���W�n�j�ɑ΂��Ă̊p�x�Ȃ̂Œ��� Note that the angle is with respect to the ground (x-y coordinate system)
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
	//�V�r�̒n�ʊ�̊֐ߊp�x Ground angle of swing leg
	//Calculation_Swing_Angle(angData[POS], Leg, q, body_ang); //simi commented
	for (int j=0;j<3;j++)
		q[j] = angData[POS][L][j];//simi added to make R H origin, indep of left leg. NOTE: SUP(L) angles may be opp of SW angles, in which case make these neg
	for(int i=0;i<3;i++) dq[i] = angData[VEL][L][i];
	printf("H %lf, K %lf, A %lf\n", q[H], q[K], q[A]); //print cur
	//���R�r�̌v�Z(2�~3)�@�n�ʁix-y����)�ł̍�����R�r�A��(�s��v�Z�̊ȒP���̂���3�~3)
	//Jacobi calculation (2 �~ 3) �� waist reference Jacobian on the ground (x-y plane) (3 �~ 3 to simplify matrix calculation)
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
	
	////���R�r�̔����̌v�Z(2�~3)�@�n�ʁix-y����)�ł̍�����R�r�A��(�s��v�Z�̊ȒP���̂���3�~3)
	//Jb->diff[0][0] =  -LEN_H_K*( dq[H] )*sin( q[H] ) - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	//Jb->diff[0][1] =                                 - LEN_K_A*( dq[H] + dq[K] )*sin( q[H] + q[K] ) - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );       
	//Jb->diff[0][2] =													                               - LEN_A_T*( dq[H] + dq[K] + dq[A] )*sin( q[H] + q[K] + q[A] );
	//Jb->diff[1][0] =   LEN_H_K*( dq[H] )*cos( q[H] ) + LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );                                     
	//Jb->diff[1][1] =                                   LEN_K_A*( dq[H] + dq[K] )*cos( q[H] + q[K] ) + LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] ); 
	//Jb->diff[1][2] =                                                                                  LEN_A_T*( dq[H] + dq[K] + dq[A] )*cos( q[H] + q[K] + q[A] );
	//Jb->diff[2][0] = 0.0;
	//Jb->diff[2][1] = 0.0;
	//Jb->diff[2][2] = 0.0;

	//���R�r�s��̓]�u�̌v�Z
	Matrix33_Trans(jb->matrix, jb->trans);
	Matrix22_Trans(jh->matrix, jh->trans);
	////���R�r�s��̋[���t�s��̌v�Z
	//Matrix33_PseudoInverse(jacobData->matrix, jacobData->pse);

	////�[���t�s��̓]�u�̌v�Z
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
	//���R�r�̌v�Z(2�~3)�@�n�ʁix-y����)�ł̍�����R�r�A��(�s��v�Z�̊ȒP���̂���3�~3)
	//Jacobi calculation (2 �~ 3) �� waist reference Jacobian on the ground (x-y plane) (3 �~ 3 to simplify matrix calculation)
	jacob_data->matrix[0][0] = LEN_H_K * cos(q[H]) + LEN_K_A * cos(q[H] + q[K]) + LEN_A_T * sin(q[H] + q[K] + q[A]);
	jacob_data->matrix[0][1] = LEN_K_A * cos(q[H] + q[K]) + LEN_A_T * sin(q[H] + q[K] + q[A]);
	jacob_data->matrix[0][2] = LEN_A_T * sin(q[H] + q[K] + q[A]);
	jacob_data->matrix[1][0] = LEN_H_K * sin(q[H]) + LEN_K_A * sin(q[H] + q[K]) + LEN_A_T * cos(q[H] + q[K] + q[A]);
	jacob_data->matrix[1][1] = LEN_K_A * sin(q[H] + q[K]) + LEN_A_T * cos(q[H] + q[K] + q[A]);
	jacob_data->matrix[1][2] = LEN_A_T * cos(q[H] + q[K] + q[A]);
	jacob_data->matrix[2][0] = 0;
	jacob_data->matrix[2][1] = 0;
	jacob_data->matrix[1][2] = 0;

	//���R�r�s��̓]�u�̌v�Z
	Matrix33_Trans(jacob_data->matrix, jacob_data->trans);

}


/****************************		�C���s�[�_���X����֌W		*****************************************************************************************************/

//int Impedance_Control_Acc(double foot[3][2][3], double target[3][2][3], double force[3]){
//
//
//	//�C���s�[�_���X����
//	//��������x����
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
//	//�C���s�[�_���X����
//	//����͂���
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
�֐����FConvertForceToAcc
�����@�F�ܐ�̗͂���ܐ�̉����x�ɕϊ�
�����@�Fdouble force[3]		//�ܐ�ɉ�����
		double acc[3]		//�ܐ�ɐ���������x
	�@�@
�o�́@�Fdouble acc[3]
********************************************************/
void ConvertForceToAcc(double force[3], double acc[3]){

	//MASS_FOOT�����ʍs��ɕύX����K�v����?
	acc[X] = force[X]/MASS_FOOT;
	acc[Y] = force[Y]/MASS_FOOT;
	acc[Z] = force[Z]/MASS_FOOT;
}





/****************************		�璷�����p�֌W		*****************************************************************************************************/


/********************************************************
�֐����FCalculate_UsingRedundancy2
�����@�F�g���N�Ɋւ���璷�����p���v�Z����֐�
		���C�����[�v���Ŏg�p
�����@�Fdouble jointVal[2][3]			�G���R�[�_�̒l�����ɂ����ʒu�C���x
		double targetAcc[3]				�F�m�A�V�X�g�ɂ��^����������x
		struct jacob_data jacobData		���R�r�s��֌W�̍\����
		double redundancyAcc[3]			�璷�����p�v���O�����ɂ�蓾��ꂽ�֐ߊp�����x
		
�o�́@�Fdouble redundancyAcc[3]
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


	//�P�ʍs��̌v�Z
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

	
	//�[����ԃx�N�g���̌v�Z( (I-J#J)k )
	Calculate_NullSpace(jointVal, J, nullSpaceVector);
	
	double M[3][3]={0}, h[3]={0};
	double M_tmp[3][3];
	//Matrix33_Product(J->pse_trans, D->mass, M_tmp);
	//Matrix33_Product(M_tmp, J->pse, M);


	double V_tmp1[3]={0}, V_tmp2[3]={0}, V_tmp3[3]={0};		//�ꎞ�ۑ��̂��߂̃x�N�g��
	//�璷���𗘗p�����֐ߊp�����x�̌v�Z
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
		

	//�f�o�b�N
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
�֐����FCalculate_NullSpace
�����@�F�璷�����p�ɂ�����[����ԃx�N�g���̌v�Z( (I-J#J)k )
�����@�Fdouble jointVal[3][2][3]	�֐߂̊p�x�C�p���x�C�p�����x
		double null_space[3][3]		���R�r�s��֌W�̍\����
		double nullSpaceVector[3]	�[����ԃx�N�g��
		
�o�́@�Fdouble nullSpaceVector[3]
********************************************************/
void Calculate_NullSpace(double jointVal[3][2][3], struct jacob_data *jacobData, double nullSpaceVector[3]){

	double eta[3];		//�[����ԃx�N�g���̔C�ӌW����
	
	//�[����Ԃ̌v�Z( (I-J#J) )
	NullSpace(jacobData);
	
	//�[����ԃx�N�g���̂��߂̕]���֐��̌v�Z( k )
	EvaluationFunc_NullSpace(jointVal[POS][R], eta);

	//�[����ԃx�N�g���̌v�Z( (I-J#J)k )
	//Calculate_NullSpaceVector(jacobData->null_space, eta, nullSpaceVector);


	//�f�o�b�N
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
�֐����FCalculate_UsingRedundancy
�����@�F�֐ߊp�����x�Ɋւ���璷�����p���v�Z����֐�
		���C�����[�v���Ŏg�p
�����@�Fdouble jointVal[2][3]			�G���R�[�_�̒l�����ɂ����ʒu�C���x
		double targetAcc[3]				�F�m�A�V�X�g�ɂ��^����������x
		struct jacob_data jacobData		���R�r�s��֌W�̍\����
		double redundancyAcc[3]			�璷�����p�v���O�����ɂ�蓾��ꂽ�֐ߊp�����x
		
�o�́@�Fdouble redundancyAcc[3]
********************************************************/
void Calculate_UsingRedundancy(double jointVal[3][2][3], double targetAcc[3], struct jacob_data *jacobData, double redundancyAcc[3]){

	double eta[3];
	double nullSpaceVector[3];
	
	//�[����Ԃ̌v�Z( (I-J#J) )
	NullSpace(jacobData);
	
	//�[����ԃx�N�g���̂��߂̕]���֐��̌v�Z( k )
	EvaluationFunc_NullSpace(jointVal[POS][R], eta);

	//�[����ԃx�N�g���̌v�Z( (I-J#J)k )
	//	Calculate_NullSpaceVector(jacobData->null_space, eta, nullSpaceVector);

	//�璷�����p��̊e�֐߂̊p�����x�̌v�Z(  d2q = J#(d2r-dJdq) + (I-J#J)k  )
	Calculate_JointAcc_UsingRedundancy(targetAcc, jointVal[VEL][R], jacobData, nullSpaceVector, redundancyAcc);

	//�f�o�b�N
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
�֐����FNullSpace
�����@�F�璷�����p�ɂ�����[����Ԃ̌v�Z(���̍s�񂪃[���s��łȂ���Ώ璷�������p�ł���)
�����@�Fstruct jacob_data jacobData		���R�r�s��֌W�̍\����
		
�o�́@�Fstruct jacob_data jacobData
********************************************************/
void NullSpace(struct jacob_data *jacobData){

	double I[3][3];		//�P�ʍs��
	double M[3][3];		//�ꎞ�ۑ��̂��߂̍s��
	double N[3][3];

	//�P�ʍs��̌v�Z
	Matrix_Unit(I);

	//(J#)(J)�̌v�Z
	//Matrix33_Product(jacobData->pse, jacobData->matrix, M);
	
	//I-J#J�̌v�Z
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
//�֐����FNullSpace
//�����@�F�璷�����p�ɂ�����[����Ԃ̂��߂̕]���֐��̌v�Z
//�����@�Fstruct jacob_data jacobData		���R�r�s��֌W�̍\����
//		
//�o�́@�Fstruct jacob_data jacobData
//********************************************************/
//void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]){
//
//	double enc_min[3] = { 50, -60, -12.5 };							//�e�֐߂̊֐��̍ŏ��l
//	for(int i=0;i<3;i++)	enc_min[i] = enc_min[i]*PI/180;		//deg��rad
//	
//	//�d�݁@������ύX���邱�ƂŊe�֐߃g���N�̃E�G�C�g�l���ύX����G�l���M�[�������l�����邱�ƂɂȂ�
//	double Coeff[3][3] = { 
//							{100.0, 0.0, 0.0},
//							{0.0, 0.0, 0.0},
//							{0.0, 0.0, 100.0}  };
//	
//	//�e�֐߂ƕ]���֐��̊֌W
//	double C[3] = {   (1.0/10)*pow( enc_data[H]-enc_min[H], 10.0),
//					  (1.0/25)*pow(-enc_data[K]+enc_min[K], 10.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
//					(1.0/0.01)*pow( enc_data[A]-enc_min[A], 10.0) };
//	//C���e�֐ߊp�x�ŕΔ��������l
//	double C_diff[3] = { (10.0/10)*pow( enc_data[H]-enc_min[H], 9.0),
//						 -(10.0/25)*pow(-enc_data[K]+enc_min[K], 9.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
//						 (10.0/0.01)*pow( enc_data[A]-enc_min[A], 9.0) };
//
//	//�]���֐�
//	double E = ( C[H] + C[K] + C[A] );
//
//	//�]���֐����e�֐ߊp�x�ŕΔ��������l
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
//	//�f�o�b�N�p
//	//printf("%lf %lf %lf\n",C_diff[0], C_diff[1], C_diff[2]);
//	printf("%lf %lf %lf\n",eta[0], eta[1], eta[2]);
//}



/********************************************************
�֐����FNullSpace
�����@�F�璷�����p�ɂ�����[����Ԃ̂��߂̕]���֐��̌v�Z
�����@�Fstruct jacob_data jacobData		���R�r�s��֌W�̍\����
		
�o�́@�Fstruct jacob_data jacobData
********************************************************/
void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]){

	double enc_min[3] = { 20, -50, -12.5 };							//�e�֐߂̊֐��̍ŏ��l
	for(int i=0;i<3;i++)	enc_min[i] = enc_min[i]*PI/180;		//deg��rad
	
	//�d�݁@������ύX���邱�ƂŊe�֐߃g���N�̃E�G�C�g�l���ύX����G�l���M�[�������l�����邱�ƂɂȂ�
	double Coeff[3][3] = { 
							{1.0, 0.0, 0.0},
							{0.0, 1.0, 0.0},
							{0.0, 0.0, 10.0}  };
	
	//�e�֐߂ƕ]���֐��̊֌W
	double C[3] = {   (1.0/2)*pow( enc_data[H]-enc_min[H], 10.0),
					  (1.0/150)*pow(-enc_data[K]+enc_min[K], 10.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
					(1.0/0.1)*pow( enc_data[A]-enc_min[A], 10.0) + (1.0/2.0)*pow( enc_data[A]-15*PI/180, 2.0 ) };
	//C���e�֐ߊp�x�ŕΔ��������l
	double C_diff[3] = { (10.0/2)*pow( enc_data[H]-enc_min[H], 9.0),
						 -(10.0/150)*pow(-enc_data[K]+enc_min[K], 9.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
						 (10.0/0.1)*pow( enc_data[A]-enc_min[A], 9.0) + (2.0/2.0)*pow( enc_data[A]-15*PI/180, 1.0 ) };

	//�]���֐�
	double E = ( 1 / ( C[H] + C[K] + C[A] + 1 ) );

	//�]���֐����e�֐ߊp�x�ŕΔ��������l
	double E_diff[3];
	for(int i=0;i<3;i++)	E_diff[i] = -C_diff[i]/(E*E);

	for(int i=0;i<3;i++){
		eta[i] = 0.0;
		for(int j=0;j<3;j++){
			eta[i] += Coeff[i][j]*E_diff[j];
		}
	}

	//for(int i=0;i<3;i++)	eta[i] = 0.0;


	//�f�o�b�N�p
	//printf("%lf %lf %lf\n",C_diff[H], C_diff[K], C_diff[A]);
	//printf("%lf %lf %lf %lf\n",eta[0], eta[1], eta[2], E);
}


///********************************************************
//�֐����FNullSpace
//�����@�F�璷�����p�ɂ�����[����Ԃ̂��߂̕]���֐��̌v�Z
//�����@�Fstruct jacob_data jacobData		���R�r�s��֌W�̍\����
//		
//�o�́@�Fstruct jacob_data jacobData
//********************************************************/
//void EvaluationFunc_NullSpace(double enc_data[3], double eta[3]){
//
//	double enc[3];
//	for(int i=0;i<3;i++)	enc[i] = enc_data[i]*180/PI;
//
//	//�d�݁@������ύX���邱�ƂŊe�֐߃g���N�̃E�G�C�g�l���ύX����G�l���M�[�������l�����邱�ƂɂȂ�
//	double Coeff[3] = { (5.0*pow(10.0,4.0)), (1.0*pow(10.0,5.0)), (2.0*pow(10.0,6.0)) };
//	
//	//�e�֐߂ƕ]���֐��̊֌W
//	double C[3] = { pow(10.0, -6.0)*pow( enc[H]-55.0, 4.0),
//					pow(10.0, -6.0)*pow(-enc[K]+65.0, 4.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
//					pow(10.0, -5.0)*pow( enc[A]+15.0, 4.0) };
//	//C���e�֐ߊp�x�ŕΔ��������l
//	double C_diff[3] = { -4.0*pow(10.0, -6.0)*pow( enc[H]-55.0, 3.0),
//						  4.0*pow(10.0, -6.0)*pow(-enc[K]+65.0, 3.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
//						 -4.0*pow(10.0, -5.0)*pow( enc[A]+15.0, 3.0) };
//
//	//�]���֐�
//	double E = 1/( C[H] + C[K] + C[A] + 1 );
//
//	//�]���֐����e�֐ߊp�x�ŕΔ��������l
//	double E_diff[3];
//	for(int i=0;i<3;i++)	E_diff[i] = -1.0*C_diff[i]*pow(E, 2.0);
//
//	for(int i=0;i<3;i++)	eta[i] = Coeff[i]*E_diff[i];
//
//}


/********************************************************
�֐����FNullSpace
�����@�F�璷�����p�ɂ�����[����ԃx�N�g���̌v�Z
�����@�Fdouble null_space[3][3]		���R�r�s��֌W�̍\����
		double eta[3]				�]���֐�
		double vector[3]			�[����ԃx�N�g��
		
�o�́@�Fstruct jacob_data jacobData
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

	//�f�o�b�N�p
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
�֐����FCalculate_JointAcc_UsingRedundancy
�����@�F�璷���𗘗p���ăT�u�^�X�N��^�����e�֐߂̊p�����x���v�Z����֐�
�����@�Fdouble targetAcc[3]				�F�m�A�V�X�g�ɂ��^����������x
		double jointVel[3]				���݂̊֐ߑ��x
		struct jacob_data jacobData		���R�r�s��֌W�̍\����
		double nullSpaceVector[3]		�[����ԃx�N�g��
		double jointVal[3][3]			�璷�����p�v���O�����ɂ�蓾��ꂽ�֐ߊp���x�Ɗp�����x
		
�o�́@�Fdouble jointVal[3][3]
********************************************************/
void Calculate_JointAcc_UsingRedundancy(double targetAcc[3], double jointVel[3], struct jacob_data *jacobData, double nullSpaceVector[3], double jointAcc[3]){

	double V_tmp1[3]={0}, V_tmp2[3]={0};		//�ꎞ�ۑ��̂��߂̃x�N�g��
	

	//�璷���𗘗p�����֐ߊp�����x�̌v�Z
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


	//�f�o�b�N�p
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
�֐����FMatrix_Unit
�����@�F3�~3�̒P�ʍs��̌v�Z
�����@�Fdouble I[3][3]		3�~3�̒P�ʍs��
		
�o�́@�Fdouble I[3][3]
********************************************************/
void EnergyMinimization(double enc_data[2][3], struct leg_state *Leg, struct jacob_data *jacobData, double torq[3], double force[3]){
	
	//�n�ʁix-y���W�n�j�ɑ΂��Ă̊p�x�Ȃ̂Œ���
	double theta[3];
	double eta[3];
	double force_tmp[3];
	double force_null[3] = {0,0,0};
	double force_image[3] = {0,0,0};
	double force_image_tmp[3] = {0,0,0};
	double body_ang = 0;
	Calculation_Swing_Angle(enc_data, Leg, theta, body_ang);

	//�[����ԃx�N�g���̂��߂̕]���֐��̌v�Z
	NullSpace_TargetFunc(theta, eta);

	//�[����ԃx�N�g���̌v�Z
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


//�֐߉���̌v�Z�@ELim�F�L�W���̐����p�x�AFLim�F�L�W���̐����p�x
int Cal_JointLimit(double enc[2][3], double theta_ELim[2][3], double theta_FLim[2][3]){

	double enc_ang[2][3];		//�G���R�[�_�̃f�[�^��deg�ɒ��������́B

	for(int i=0; i<2; i++){
		for(int j=0; j<2; j++){
			enc_ang[i][j] = enc[i][j]/PI*180;		//rad����deg�ɕϊ�
		}
	}

	double Krammel,
			Krammel1,
			Krammel2;		//�N�������\�L�@�@<A>=A (A>0�̂Ƃ�), <A>=0 (A<0�̂Ƃ�)

	//�G�̓G���R�[�_�v���l�����ȑ������ƂȂ��Ă�̂ŕ����ɒ���

	//****** �ҊԐ� ******
	//*** �L�W ***
	//�E
	Krammel = -enc_ang[R][K]-(93.49-0.316*AGE);
	if(Krammel<0) Krammel =0;
	theta_ELim[R][H] = 0.282*Krammel+0.308*AGE-40.62;

	//��
	Krammel = -enc_ang[L][K]-(93.49-0.316*AGE);
	if(Krammel<0) Krammel =0;
	theta_ELim[L][H] = 0.282*Krammel+0.308*AGE-40.62;
	
	//*** ���� *** 
	//�E
	Krammel = (84.07+0.135*AGE)+enc_ang[R][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[R][H] = -0.733*Krammel-0.183*AGE+144.7;
	//��
	Krammel = (84.07+0.135*AGE)+enc_ang[L][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[L][H] = -0.733*Krammel-0.183*AGE+144.7;


	//***** �G�֐� *****
	//*** �L�W ***
	//�E
	Krammel1 = enc_ang[R][H]-(81.06-0.282*AGE);
	if(Krammel1<0) Krammel1 =0;
	Krammel2 = enc_ang[R][A]-23;
	if(Krammel2<0) Krammel2 =0;
	if(Krammel1>Krammel2)	theta_ELim[R][K] = 1.364*Krammel1-2.72;
	else					theta_ELim[R][K] = 0.9*Krammel2-2.72;
	//��
	Krammel1 = enc_ang[L][H]-(81.06-0.282*AGE);
	if(Krammel1<0) Krammel1 =0;
	Krammel2 = enc_ang[L][A]-23;
	if(Krammel2<0) Krammel2 =0;
	if(Krammel1>Krammel2)	theta_ELim[L][K] = 1.364*Krammel1-2.72;	
	else					theta_ELim[L][K] = 0.9*Krammel2-2.72;
	//*** ���� ***
	//�E
	Krammel = (-19.85+0.397*AGE)-enc_ang[R][H];
	if(Krammel<0) Krammel =0;
	theta_FLim[R][K] = -3.546*Krammel+167.14;
	//��
	Krammel = (-19.85+0.397*AGE)-enc_ang[L][H];
	if(Krammel<0) Krammel =0;
	theta_FLim[L][K] = -3.546*Krammel+167.14;

	//***** ���֐� *****
	//*** ���� ***
	theta_ELim[R][A] = -55; 
	theta_ELim[L][A] = -55; 	
	//*** �L�W ***
	//�E
	Krammel = 26+enc_ang[R][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[R][A] = -1.1*Krammel+46;
	//��
	Krammel = 26+enc_ang[L][K];
	if(Krammel<0) Krammel =0;
	theta_FLim[L][A] = -1.1*Krammel+46;


	//deg����rad�ɕϊ�
	for(int i=0; i<2; i++){
		for(int j=0; j<3; j++){
			theta_ELim[i][j] = theta_ELim[i][j]*PI/180;
			theta_FLim[i][j] = theta_FLim[i][j]*PI/180;
		}
	}

return 0;
}

void CalculateEnergyMinimization(double enc_data[2][3], double eta[6][2]){
	double theta_ELim[2][3];	//�L�W�����̐����p�x
	double theta_ELim6[6];			//�֋X��x�N�g���ɒ���������
	double theta_FLim[2][3];	//���ȕ����̐����p�x
	double theta_FLim6[6];			//�֋X��x�N�g���ɒ���������

	double theta0[6];				//���R��Ԃ̊֐ߊp�x

	int sup=0;
	int swi=0;

	double ang[6];

	//�l�̋��ȕ����𐳁i�G�����G���R�[�_�̊֌W�ŕ��Ƃ��Ă���̂Œ��Ӂj
	ang[0]=enc_data[sup][A];
	ang[1]=-enc_data[sup][K];
	ang[2]=enc_data[sup][H];
	ang[3]=enc_data[swi][H];
	ang[4]=-enc_data[swi][K];
	ang[5]=enc_data[swi][A];


	//*** �֐߉���̌v�Z ***
	Cal_JointLimit(enc_data, theta_ELim, theta_FLim);

	/***	�֐߉�����l�������]���֐��̍쐬		***/
	//���R��Ԃ̊֐ߊp�x
	theta0[0]=0;												//���֐�
	theta0[1]=(theta_FLim[sup][K]+theta_ELim[sup][K])/2;		//�G�֐�
	theta0[2]=0;												//�ҊԐ�
	theta0[3]=0;												//�ҊԐ�
	theta0[4]=(theta_FLim[swi][K]+theta_ELim[swi][K])/2;		//�G�֐�
	theta0[5]=0;												//���֐�

	//�L�W�C���ȕ����̐����p�x
	theta_ELim6[0] = theta_ELim[sup][A] ;	theta_FLim6[0] = theta_FLim[sup][A] ;
	theta_ELim6[1] = theta_ELim[sup][K] ;	theta_FLim6[1] = theta_FLim[sup][K] ;
	theta_ELim6[2] = theta_ELim[sup][H] ;	theta_FLim6[2] = theta_FLim[sup][H] ;
	theta_ELim6[3] = theta_ELim[swi][H] ;	theta_FLim6[3] = theta_FLim[swi][H] ;
	theta_ELim6[4] = theta_ELim[swi][K] ;	theta_FLim6[4] = theta_FLim[swi][K] ;
	theta_ELim6[5] = theta_ELim[swi][A] ;	theta_FLim6[5] = theta_FLim[swi][A] ;


	double Etatmp=0;
	double eps=pow(2.0,-30);		//������
	
	double ke;								//�]���֐��̕Δ����̌W���@���C���^�X�N�̍��̃I�[�_�[�ɂ���Č��߂�	

	//*** �ł̌v�Z ***
	for(int i=0; i<6; i++){
		if(i==1 || i==4)	ke=100000;		//�G�����ł��傫���Ȃ�̂ŁA����������
		else				ke=100;			//�G�ȊO
		//�L�W��
		if(ang[i]<=theta0[i]){
//			if(i==2)printf("�L�W\t");
			Etatmp = cos(ang[i]-theta_ELim6[i]+PI/2);	
			if(fabs(Etatmp) < eps )	eta[i][X] = pow(10.0, 6);		//������ɔ��U���Ă��܂��̂ŁA10^3�ő��(���[�^�o�̖͂O�a���l���j
			else{
				eta[i][X] = fabs(1/( (cos(ang[i]-theta_ELim6[i]+PI/2))*(cos(ang[i]-theta_ELim6[i]+PI/2)) ) ) * (ang[i]-theta0[i])*(ang[i]-theta0[i])  + 2*fabs(tan(ang[i]-theta_ELim6[i]+PI/2)) * (ang[i]-theta0[i]);
				eta[i][X] /= -ke;
			}
		}
		//���ȑ�
		else if(ang[i] > theta0[i]){
			Etatmp = cos(ang[i]-theta_FLim6[i]-PI/2);
			if(fabs(Etatmp) < eps )	eta[i][X] = pow(10.0, 6);		//������ɔ��U���Ă��܂��̂ŁA10^8�ő��
			else{
				eta[i][X] = fabs(1/( (cos(ang[i]-theta_FLim6[i]-PI/2))*(cos(ang[i]-theta_FLim6[i]-PI/2)) ) ) * (ang[i]-theta0[i])*(ang[i]-theta0[i]) + 2*fabs(tan(ang[i]-theta_FLim6[i]-PI/2)) * (ang[i]-theta0[i]);
				eta[i][X] /= -ke;
			}
		}
	}
	/***	�֐߉�����l�������]���֐��̍쐬�@�����܂�	***/

}


void NullSpace_TargetFunc(double enc_data[3], double eta[3]){

	double enc[3];
	for(int i=0;i<3;i++)	enc[i] = enc_data[i]*180/PI;

	//�d�݁@������ύX���邱�ƂŊe�֐߃g���N�̃E�G�C�g�l���ύX����G�l���M�[�������l�����邱�ƂɂȂ�
	double Coeff[3] = { (5.0*pow(10.0,4.0)), (1.0*pow(10.0,5.0)), (2.0*pow(10.0,6.0)) };
	
	//�e�֐߂ƕ]���֐��̊֌W
	double C[3] = { pow(10.0, -6.0)*pow( enc[H]-55.0, 4.0),
					pow(10.0, -6.0)*pow(-enc[K]+65.0, 4.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
					pow(10.0, -5.0)*pow( enc[A]+15.0, 4.0) };
	//C���e�֐ߊp�x�ŕΔ��������l
	double C_diff[3] = { -4.0*pow(10.0, -6.0)*pow( enc[H]-55.0, 3.0),
						  4.0*pow(10.0, -6.0)*pow(-enc[K]+65.0, 3.0),		//�G�����G���R�[�_�̌������t�Ȃ̂Œ���
						 -4.0*pow(10.0, -5.0)*pow( enc[A]+15.0, 3.0) };

	//�]���֐�
	double E = 1/( C[H] + C[K] + C[A] + 1 );

	//�]���֐����e�֐ߊp�x�ŕΔ��������l
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

	//�P�ʍs��̌v�Z
	Matrix_Unit(I);

	//(J#)(J)�̌v�Z
	//Matrix33_Product(jacobData.trans,jacobData.pse_trans,jacob_tmp);
	
	//I-J#J�̌v�Z
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







/****************************		���͊w�֌W		*****************************************************************************************************/


/********************************************************
�֐����FSetDynamicsParam
�����@�F���͊w�p�̃p�����[�^��ݒ�iD-H�p�����[�^�C�[�������s��Ȃǁj
�����@�Fdouble encData[2][3]				 �e�֐߂̃G���R�[�_�̒l
		struct transfer_data *transfer_data  �����ϊ��s��p�̍\����
		
�o�́@�Fstruct transfer_data *transfer_data
********************************************************/
void Calculate_Dynamics(double angData[3][2][3], double targetAcc[3], struct transfer_data *transferData, struct dynamic_data *dynamicData, double torq[3]){

	//D-H�p�����[�^��[�������s��̃p�����[�^�ݒ�
	SetDynamicsParam(angData[POS], transferData);

	//�����ϊ��s��̌v�Z(0Ti)
	Matrix44_Transfer(transferData);

	//�����ϊ��s���1�K�C2�K�̕Δ����v�Z
	Matrix44_Diff(transferData);
	Matrix44_Diff2(transferData);


	//���͊w�̌v�Z
	//M:�����s��CH�F�R���I���́E���S�̓x�N�g���CG�F�d�̓x�N�g��
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

	//�f�o�b�N�p
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
�֐����FSetDynamicsParam
�����@�F���͊w�p�̃p�����[�^��ݒ�iD-H�p�����[�^�C�[�������s��Ȃǁj
�����@�Fdouble encData[2][3]				 �e�֐߂̃G���R�[�_�̒l
		struct transfer_data *transfer_data  �����ϊ��s��p�̍\����
		
�o�́@�Fstruct transfer_data *transfer_data
********************************************************/
void SetDynamicsParam(double encData[2][3], struct transfer_data *transfer_data){

	const int SW=R;
	double q[3];
	for(int i=0;i<3;i++)	q[i] = encData[SW][i];

	//D-H�p�����[�^�̐ݒ�
	double DH_P[LINK_NUM][4] = { 
									{0      , 0, 0, q[0]-PI/2},
									{LEN_H_K, 0, 0, q[1]	 },
									{LEN_K_A, 0, 0, q[2]+PI/2}
							   };
	//i-1Ti�̓����ϊ��s��̌v�Z
	double T[LINK_NUM][4][4];
	for(int i=0; i<LINK_NUM; i++){
		T[i][0][0] = cos(DH_P[i][3])				;	T[i][0][1] =-sin(DH_P[i][3])				;	T[i][0][2] = 0				;	T[i][0][3] = DH_P[i][0]				   ;
		T[i][1][0] = cos(DH_P[i][1])*sin(DH_P[i][3]);	T[i][1][1] = cos(DH_P[i][1])*cos(DH_P[i][3]);	T[i][1][2] =-sin(DH_P[i][1]);	T[i][1][3] =-DH_P[i][2]*sin(DH_P[i][1]);
		T[i][2][0] = sin(DH_P[i][1])*sin(DH_P[i][3]);	T[i][2][1] = sin(DH_P[i][1])*cos(DH_P[i][3]);	T[i][2][2] = cos(DH_P[i][1]);	T[i][2][3] = DH_P[i][2]*cos(DH_P[i][1]);
		T[i][3][0] = 0								;	T[i][3][1] = 0								;	T[i][3][2] = 0				;	T[i][3][3] = 1						   ;
	}


	/***  �[�������s��̃p�����[�^�ݒ�  ***/
	double link[LINK_NUM] = {LEN_H_K, LEN_K_A, LEN_A_T};
	double mass[LINK_NUM] = {MASS_R_THIGH, MASS_R_LOWER, MASS_R_FOOT};
	double H[LINK_NUM][4][4];

	//�V�r�@[0]:�Ҋ֐�-�G�����N�@[1]:�G-���񃊃��N�@[2]:����-�ܐ惊���N
	for(int i=0;i<LINK_NUM;i++){
		H[i][0][0] = link[i]*link[i]/3;		H[i][0][1] = 0;		H[i][0][2] = 0;		H[i][0][3] = link[i]/2;
		H[i][1][0] = 0;						H[i][1][1] = 0;		H[i][1][2] = 0;		H[i][1][3] = 0;
		H[i][2][0] = 0;						H[i][2][1] = 0;		H[i][2][2] = 0;		H[i][2][3] = 0;
		H[i][3][0] = link[i]/2;				H[i][3][1] = 0;		H[i][3][2] = 0;		H[i][3][3] = 1;
	}


	//�f�[�^�̕ۑ�
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
�֐����FMatrix44_transfer
�����@�F0����i�����N�܂ł̓����ϊ��s��̌v�Z
�����@�Fstruct transfer_data *T  �����ϊ��s��p�̍\����
		
�o�́@�Fstruct transfer_data *T
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
�֐����FMatrix44_Diff
�����@�F�����ϊ��s����֐�i��1�K�Δ���
�����@�Fstruct transfer_data *T  �����ϊ��s��p�̍\����
		
�o�́@�Fstruct transfer_data *T
********************************************************/
void Matrix44_Diff(struct transfer_data *T){
	double T_diff_tmp[4][4];
	//�Δ����̂��߂̍s��i�֐�i����]�s��̂Ƃ��j
	double Q[4][4]={ 
		{0, -1, 0, 0},
		{1,  0, 0, 0},
		{0,  0, 0, 0},
		{0,  0, 0, 0}
	};

	//i: 0����i�Ԗڂ܂ł̓����ϊ�
	for(int i =0; i<LINK_NUM; i++){
		//j: �֐�j�ŕΔ�������
		for(int j=0; j<=i; j++){
			//�P�K�����i0���烊���N���̔�������Q�܂Łj
			Matrix44_Product(T->T0[j], Q, T->T_diff[i][j]);

			if(j<i){
				//1�K����Q���������c��̕���
				for(int h=j+1; h<=i; h++){
					//�ꎞ�ۑ��A�X�V
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
�֐����FMatrix44_Diff2
�����@�F�����ϊ��s����֐�i��2�K�Δ���
�����@�Fstruct transfer_data *T  �����ϊ��s��p�̍\����
		
�o�́@�Fstruct transfer_data *T
********************************************************/
void Matrix44_Diff2(struct transfer_data *T){

	double T_diff2_tmp[4][4];
	//�Δ����̂��߂̍s��i�֐�i����]�s��̂Ƃ��j
	double Q[4][4]={ 
		{0, -1, 0, 0},
		{1,  0, 0, 0},
		{0,  0, 0, 0},
		{0,  0, 0, 0}
	};

	//********* �Q�K�Δ���(q��j��m(j<=m)�ŕΔ���) ************
		//i: 0����i�Ԗڂ܂ł̓����ϊ�
	for(int i =0; i<LINK_NUM; i++){
		//j: �֐�j�ŕΔ�������
		for(int j=0; j<=i; j++){
		//�Q�K����(j<=m)�@m: �֐�m�ŕΔ�������
			for(int m=j; m<=i; m++){
				//j�̕Δ��������܂Ōv�Z
				Matrix44_Product(T->T0[j], Q, T_diff2_tmp);
				//j��m�̊Ԃ̌v�Z
				for(int h=j+1; h<=m; h++){
					Matrix44_Product(T_diff2_tmp, T->T[h], T->T_diff2[i][j][m]);
					//�X�V
					for(int k=0; k<4; k++){
						for(int l=0; l<4; l++){
							T_diff2_tmp[k][l] = T->T_diff2[i][j][m][k][l];
						}
					}
				}
				//m�̕Δ��������܂Ōv�Z
				Matrix44_Product(T_diff2_tmp, Q, T->T_diff2[i][j][m]);
				//�c��̕����̌v�Z�i����΁j
				for(int h=m+1; h<=i; h++){
					//�X�V
					for(int k=0; k<4; k++){
						for(int l=0; l<4; l++){
							T_diff2_tmp[k][l] = T->T_diff2[i][j][m][k][l];	
						}			
					}					
					Matrix44_Product(T_diff2_tmp, T->T[h], T->T_diff2[i][j][m]);
				}	
			}
			// m��j�͉�(m>j�̂Ƃ��ł�ok)
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
	//******** ���ʍs�� M�̌v�Z *************
	for(int i=0; i<LINK_NUM; i++){
		for(int j=0;j<LINK_NUM; j++){
			D->mass[i][j] = 0;

			if(i>j) kmax=i;
			else    kmax=j;
			for(int k=kmax; k<LINK_NUM; k++){
				//���ʍs��̌v�Z
				Matrix44_Product(T->T_diff[k][j], T->H[k], H_tmp);
				Matrix44_Product(H_tmp, T->T_diff_trans[k][i], M_tmp);

				Matrix44_Trace(M_tmp, &M_trace);

				D->mass[i][j] += M_trace;							
			
			}
		}
	}
	
	////�f�o�b�N�p
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
		
		for(int k=i; k<LINK_NUM; k++){//j��l��k�������Ă����́H��������O��
			for(int j=0;j<=k; j++){
				for(int l=0;l<=k; l++){					
					//(d2T/dqjdl = d2T/dqldqj �F���j

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

	//���̕ӃO���[�o���ϐ��ɂ���������������
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
	

	//�l�̎��ʁi���̍s��g���Ċ����s��̌v�Z�������V���v���ɂł����ˁc�܂��Ƃ肠�������̂܂܁j	
	mass_h[0]=MASS_THIGH;
	mass_h[1]=MASS_LOWER;
	mass_h[2]=MASS_FOOT;


	// ************** �d�͍� g�̌v�Z ************
	
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




/****************************		�s��v�Z�֌W		*****************************************************************************************************/

///////////		3�~3�s��@���R�r��[���t�s��ȂǂɎg�p		///////////

/********************************************************
�֐����FMatrix_Unit
�����@�F3�~3�̒P�ʍs��̌v�Z
�����@�Fdouble I[3][3]		3�~3�̒P�ʍs��
		
�o�́@�Fdouble I[3][3]
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
�֐����FMatrix33_Trans
�����@�F3�~3�s��̓]�u
�����@�Fdouble M[3][3]		�@3�~3�̍s��
		double M_trans[3][3]  3�~3�s��̓]�u
		
�o�́@�Fdouble M_trans[3][3]
********************************************************/
void Matrix33_Trans(double M[3][3], double M_trans[3][3]){
	
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			M_trans[j][i] = M[i][j];

}

/********************************************************
�֐����FMatrix22_Trans
�����@�F2�~2�s��̓]�u
�����@�Fdouble M[2][2]		�@2�~2�̍s��
		double M_trans[2][2]  2�~2�s��̓]�u

�o�́@�Fdouble M_trans[2][2]
********************************************************/
void Matrix22_Trans(double M[2][2], double M_trans[2][2]) {

	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++)
			M_trans[j][i] = M[i][j];

}

/********************************************************
�֐����FMatrix66_Trans
�����@�F3�~3�s��̓]�u
�����@�Fdouble M[6][6]		�@6x6�̍s��
		double M_trans[6][6]  6x6�s��̓]�u
		
�o�́@�Fdouble M_trans[6][6]
********************************************************/
void Matrix66_Trans(double M[6][6], double M_trans[6][6]){
	
	for(int i=0; i<6; i++)
		for(int j=0; j<6; j++)
			M_trans[j][i] = M[i][j];

}

/********************************************************
�֐����FMatrix33_Det
�����@�F3�~3�s��̍s�񎮂̌v�Z
�����@�Fdouble M[3][3]		�@3�~3�̍s��
		
�߂�l�Freturn det;		//�s��
********************************************************/
double Matrix33_Det(double M[3][3]){
	double det=0;

	for(int i=0; i<3; i++){
		det += M[0][i] * ( M[1][(i+1)%3]*M[2][(i+2)%3] - M[1][(i+2)%3]*M[2][(i+1)%3] );
	}

	return det;
}


/********************************************************
�֐����FMatrix22_Inv
�����@�F2�~2�s��̋t�s��̌v�Z
�����@�Fdouble M[3][3]		�@3�~3�̍s��
		double M_inv[3][3]	  3�~3�̋t�s��i3�s�C3��ɂ�0������j

�o�́@�Fdouble M_inv[3][3]	
�߂�l�Freturn det;		//�s��
********************************************************/
double Matrix22_Inv(double M[3][3], double M_inv[3][3]){
	double det = M[0][0]*M[1][1] - M[0][1]*M[1][0];
	double M_cof[3][3] = {
		{ M[1][1],-M[1][0],0 },
		{-M[0][1], M[0][0],0 },
		{       0,       0,0 },
	};

	if(det == 0){
		printf("�t�s��̌v�Z���ł��܂���ł���\n");
		return det;
	}else{
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				M_inv[i][j] = M_cof[i][j]/det;
	}
	return det;
}


/********************************************************
�֐����FMatrix33_Inv
�����@�F3�~3�s��̋t�s��̌v�Z
�����@�Fdouble M[3][3]		�@3�~3�̍s��
		double M_inv[3][3]	  3�~3�̋t�s��
		
�o�́@�Fdouble M_inv[3][3]
********************************************************/
int Matrix33_Inv(double M[3][3], double M_inv[3][3]){

	double M_cof[3][3];  //�]���q�s��
	double det;
	
	M_cof[0][0] =   M[1][1]*M[2][2] - M[1][2]*M[2][1];	  M_cof[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]);   M_cof[0][2] =   M[0][1]*M[1][2] - M[0][2]*M[1][1];
	M_cof[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]);	  M_cof[1][1] =   M[0][0]*M[2][2] - M[0][2]*M[2][0];	M_cof[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]);
	M_cof[2][0] =   M[1][0]*M[2][1] - M[1][1]*M[2][0];	  M_cof[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]);   M_cof[2][2] =   M[0][0]*M[1][1] - M[0][1]*M[1][0];
	
	det = Matrix33_Det(M);

	if(det == 0){
		printf("�t�s��̌v�Z���ł��܂���ł���\n");
		return -1;
	}else{
		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				M_inv[i][j] = M_cof[i][j]/det;
	}

	return 0;

}


/********************************************************
�֐����FMatrix33_Product
�����@�F3�~3�s��̐ς̌v�Z
�����@�Fdouble A[3][3]	  3�~3�̍s��(��)
		double B[3][3]	  3�~3�̍s��(�E)
		double M[3][3]	  �v�Z���3�~3�̍s��
		
�o�́@�Fdouble M[3][3]
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
�֐����FMatrix33_PseudoInverse
�����@�F3�~3�s��̋[���t�s��̌v�Z(2�~3�s��)
		M_pse=M_trans�~(M�~M_trans)_inv
		M�~M_pse=I
�����@�Fdouble M[3][3]	    3�~3�̍s��(3�s�ڂ�0)
		double M_pse[3][3]	3�~3�̋[���t�s��(3��ڂ�0)
		
�o�́@�Fdouble M_pse[3][3]
********************************************************/
int Matrix33_PseudoInverse(double M[3][3], double M_pse[3][3]){

	double det;
	double M_trans[3][3];		//A�̓]�u�s��
	double M_pro[3][3];			
	double M_pro_inv[3][3];

	Matrix33_Trans(M,M_trans);

	Matrix33_Product(M,M_trans,M_pro);

	if(M_pro[0][2]==M_pro[1][2]==M_pro[2][2]==M_pro[2][0]==M_pro[2][1]==0){
		det = Matrix22_Inv(M_pro,M_pro_inv);
	}else{
		printf("�[���t�s��̌v�Z���ł��܂���ł���\n");
		return -1;
	}
	
	if(det==0){
		printf("�[���t�s��̌v�Z���ł��܂���ł���\n");
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


///////////		4�~4�s��@�����ϊ��s��⓮�͊w�ȂǂɎg�p		///////////

/********************************************************
�֐����FMatrix44_Trace
�����@�F4�~4�s��̃g���[�X
�����@�Fdouble M[4][4]		�@4�~4�̍s��
		double *M_trace		  3�~3�s��̃g���[�X
		
�o�́@�Fdouble *M_trans
********************************************************/
void Matrix44_Trace(double M[4][4], double *M_trace){

	*M_trace = 0;
	for(int i=0;i<4;i++)	*M_trace += M[i][i];
}


/********************************************************
�֐����FMatrix44_Trans
�����@�F4�~4�s��̓]�u
�����@�Fdouble M[4][4]		�@4�~4�̍s��
		double M_trans[4][4]  4�~4�s��̓]�u
		
�o�́@�Fdouble M_trans[4][4]
********************************************************/
void Matrix44_Trans(double M[4][4], double M_trans[4][4]){
	
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			M_trans[j][i] = M[i][j];

}


/********************************************************
�֐����FMatrix44_Product
�����@�F4�~4�s��̐ς̌v�Z
�����@�Fdouble A[4][4]	  4�~4�̍s��(��)
		double B[4][4]	  4�~4�̍s��(�E)
		double M[4][4]	  �v�Z���4�~4�̍s��
		
�o�́@�Fdouble M[4][4]
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


//�d�͕⏞���v�Z�i���{�b�g�̎��d���x����j
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

		//�f�o�b�O�p
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
