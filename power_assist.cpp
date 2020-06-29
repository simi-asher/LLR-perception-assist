/*****************************************************************************************************************************
******************************************************************************************************************************
�p���[�A�V�X�g�̌v�Z�ɕK�v�Ȋ֐����܂Ƃ߂��w�b�_�t�@�C��


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "power_assist.h"



/********************************************************
�֐����FCalculate_Power_Assist
�����@�F�p���[�A�V�X�g�v�Z�p�̊֐�
		���C�����[�v���Ŏg�p
�����@�Fconst int learnFlag					  �E�G�C�g�l���w�K�����邩�ǂ����̃t���O
		double weight_emg[3][EMG_CH]		�@�e�؂̋ؓd�ɑ΂���E�G�C�g�l
		double emg[2][EMG_CH]				�@�e�؂̋ؓd��rms�l
		double forceData[2][3][2]			  �̓Z���T�̃f�[�^
�@�@�@�@double torq[2][3]�@�@�@�@�@�@�@�@�@�@ ����������֐߃g���N
		
�o�́@�Fdouble torq[2][3]
********************************************************/
void Calculate_Power_Assist(const int learnFlag, _weight *weight_fuzzy, double encData[2][3], double weight_emg[3][EMG_CH], double emg[2][EMG_CH], double forceData[2][3][3], double torq[2][3]){

	double torq_forceSensor[2][3], torq_emg[2][3];
	double weight_assist[2];
	
	//double weight_p[3][EMG_CH];		//���������E�G�C�g�l

	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			torq_forceSensor[i][j] = 0.0;
			torq_emg[i][j]		   = 0.0;
		}
	}

#ifdef USE_EMG
	//�t�@�W�B�j���[��������̌v�Z
	FuzzyNeuro(learnFlag, weight_fuzzy, encData[R], emg[R], forceData[R], weight_emg);
	
	//�ؓd�x�[�X�̃p���[�A�V�X�g�̌v�Z
	//EMG_BaseAssist(weight_emg, emg, torq_emg);
	EMG_BaseAssist(weight_fuzzy->ori, emg, torq_emg);
	
#endif

#ifdef USE_FORCESENSOR
	//�̓Z���T�x�[�X�̃p���[�A�V�X�g�̌v�Z
	ForceSensorBaseAssist(forceData, torq_forceSensor);

#ifndef USE_EMG
	for(int i=0;i<EMG_CH;i++)	emg[R][i] = 0;
#endif

#endif

	//��ڋؖ����؂̋ؓd�l����̓Z���T�x�[�X�Ƌؓd�x�[�X�̃E�G�C�g�l���v�Z
	weight_assist[R] = Weight_ForceSensorAndEMG(emg[R][0]);
	weight_assist[L] = Weight_ForceSensorAndEMG(emg[L][0]);

	if(emg[R][0]>emg[L][0])	 weight_assist[L]=0;
	else					 weight_assist[R]=0;

	//�̓Z���T�x�[�X�Ƌؓd�x�[�X�̔䗦���l�������g���N�̌v�Z
	Sum_ForceSensorAndEMG_BaseAssist(weight_assist, torq_forceSensor, torq_emg, torq);
	
	/*for(int i=0;i<3;i++){
			printf("%lf\t",torq_forceSensor[R][i]);
	}
	printf("\n");*/
}


/********************************************************
�֐����FForceSensorBaseAssist
�����@�F�̓Z���T�x�[�X�̃p���[�A�V�X�g
�����@�Fdouble forceData[2][3][2]			�̓Z���T�̃f�[�^
�@�@�@�@double torq[2][3]�@�@�@�@�@�@�@�@�@�@ ����������֐߃g���N
		
�o�́@�Fdouble torq[2][3]
********************************************************/
void ForceSensorBaseAssist(double forceData[2][3][3], double torq[2][3]){

	double force[2][3];

	//�̓Z���T�̒l���}0.01�ȉ��Ȃ�0�Ƃ���i�̓Z���T��X�������g�p���Ă��Ȃ�����Y���͌v�Z���Ȃ��j
	for(int i=0;i<2;i++)
		for(int j=0;j<3;j++)
			if(forceData[i][j][X]<0.01 && forceData[i][j][X]>-0.01)	forceData[i][j][X] = 0;
		
	//�̓Z���T�̒l��[V]����[kg]�ɕϊ�
	Compensation_ForceSensor( forceData );


	//����������͂̌v�Z�ƃg���N�ւ̕ϊ�
	for(int i=0;i<2;i++){
		//�l���̃t���[���ƃ��{�b�g���̃t���[���ŗ̓Z���T�̒l�����]���邽��(-1)��������
		force[i][H] = P_Control(GAIN_P_H, -1*forceData[i][H][X], 0);
		force[i][K] = P_Control(GAIN_P_K, -1*-1*forceData[i][K][X], 0);		//�G�̂݉�]�������Ⴄ����(-1)��������
		force[i][A] = P_Control(GAIN_P_A, -1*forceData[i][A][X], 0);

		/*force[i][H] = PD_Control(gain[H], current[i][H], target[i][H]);
		force[i][K] = PD_Control(gain[K], current[i][K], target[i][K]);
		force[i][A] = PD_Control(gain[A], current[i][A], target[i][A]);*/

		torq[i][H] = LEN_H_HF * force[i][H];
		torq[i][K] = LEN_K_KF * force[i][K];
		torq[i][A] = LEN_A_AF * force[i][A];
	}

}

/************************************************************
�̓Z���T���␳�s��
************************************************************/
//2018/9/26���COEF_COMPENSATION_FORCE�̑���Ɏg�p+

double CompensationMatrix_Force_RH[3][3] = { { 191.1, 3.902, 2.341 },
{ -39.58, 211.1, 24.26 },
{ -7.973, -6.904, 325.8 } };

double CompensationMatrix_Force_RK[3][3] = { { 209.0, 66.12, 0 },//Z�������Ă���̂�Z�̏o�͂𖳎��i�̓Z���T�ԍ�UL130301�j
{ 6.767, 253.2, 0 },
{ 0, 0, 0 } };

double CompensationMatrix_Force_RA[3][3] = { { 0, 0, 0 },//����͌v���ł��Ă��Ȃ��̂ŕs�g�p�i2018/1/30�j
{ 0, 0, 0 },
{ 0, 0, 0 } };

double CompensationMatrix_Force_LH[3][3] = { { 179.6, 0.2106, 24.61 },
{ 0.5468, 207.4, 5.516 },
{ 9.874, -23.02, 310.5 } };

double CompensationMatrix_Force_LK[3][3] = { { 178.0, 3.946, 65.99 },
{ 0.5641, 186.4, 36.97 },
{ 30.37, -35.68, 802.0 } };

double CompensationMatrix_Force_LA[3][3] = { { 0, 0, 0 },
{ 0, 0, 0 },
{ 0, 0, 0 } };
/********************************************************
�֐����FCompensation_ForceSensor
�����@�F�̓Z���T�̕␳ [V]��[N]�ւ̕ϊ�
�����@�Fdouble forceData[2][3][2]			�̓Z���T�̃f�[�^
		
�o�́@�Fdouble forceData[2][3][2]
********************************************************/


void Compensation_ForceSensor(double forceData[2][3][3]) {

	//forceData[R][H][X] = COEFF_COMPENSATION_FORCE_RH*forceData[R][H][X];
	//forceData[R][K][X] = COEFF_COMPENSATION_FORCE_RK*forceData[R][K][X];
	//forceData[R][A][X] = COEFF_COMPENSATION_FORCE_RA*forceData[R][A][X];
	//forceData[L][H][X] = COEFF_COMPENSATION_FORCE_LH*forceData[L][H][X];
	//forceData[L][K][X] = COEFF_COMPENSATION_FORCE_LK*forceData[L][K][X];
	//forceData[L][A][X] = COEFF_COMPENSATION_FORCE_LA*forceData[L][A][X];

	//tempforce[RL][HKA][XYZ]
	double tempforce[2][3][3] = {};//���ꂼ��̗̓Z���T���W�n�ɂ�����̓Z���T�l[N]�i��ŉ������{�b�g�̍��W�n�ɕϊ�����j

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			tempforce[R][H][i] += CompensationMatrix_Force_RH[i][j] * forceData[R][H][j];
			tempforce[R][K][i] += CompensationMatrix_Force_RK[i][j] * forceData[R][K][j];
			tempforce[R][A][i] += CompensationMatrix_Force_RA[i][j] * forceData[R][A][j];
			tempforce[L][H][i] += CompensationMatrix_Force_LH[i][j] * forceData[L][H][j];
			tempforce[L][K][i] += CompensationMatrix_Force_LK[i][j] * forceData[L][K][j];
			tempforce[L][A][i] += CompensationMatrix_Force_LA[i][j] * forceData[L][A][j];
		}
	}

	for (int i = 0; i < 3; i++)//forceData��tempforce�������鎞�ɁA�������{�b�g�̍��W�n�ɂ��킹��
	{
			forceData[R][i][X] = tempforce[R][i][Y];
			forceData[R][i][Y] = -1 * tempforce[R][i][X];
			forceData[R][i][Z] = tempforce[R][i][Z];
			forceData[L][i][X] = -1 * tempforce[L][i][Y];
			forceData[L][i][Y] = tempforce[L][i][X];
			forceData[L][i][Z] = tempforce[L][i][Z];
	}

}

/********************************************************
�֐����FEMG_BaseAssist
�����@�F�ؓd�x�[�X�̃p���[�A�V�X�g
�����@�Fdouble weight[3][EMG_CH]			�e�؂̋ؓd�ɑ΂���E�G�C�g�l
�@�@�@�@double emg[2][EMG_CH]				rms���������ؓd�l
		double torq[2][3]�@�@�@�@�@�@�@�@�@ ����������֐߃g���N
		
�o�́@�Fdouble torq[2][3]
********************************************************/
void EMG_BaseAssist(double weight[3][EMG_CH], double emg[2][EMG_CH], double torq[2][3]){

	//�g���N�̏�����
	for(int i=0;i<2;i++)
		for(int j=0;j<3;j++)
			torq[i][j] = 0;

	//�e�ؓd����g���N�̐���
	for(int i=0;i<EMG_CH;i++){
		torq[R][H] += weight[H][i]*emg[R][i];
		torq[R][K] += weight[K][i]*emg[R][i];
		torq[R][A] += weight[A][i]*emg[R][i];
		torq[L][H] += weight[H][i]*emg[L][i];
		torq[L][K] += weight[K][i]*emg[L][i];
		torq[L][A] += weight[A][i]*emg[L][i];
	}
	//std::cout << torq[R][H] << "\n";
	
}


/********************************************************
�֐����FSum_ForceSensorAndEMG_BaseAssist
�����@�F�ؓd�x�[�X�̃p���[�A�V�X�g
�����@�Fconst double weight     			�̓Z���T�x�[�X�Ƌؓd�x�[�X�̃E�G�C�g�l
�@�@�@�@double torq_forceSensor[2][3]		�̓Z���T�x�[�X�̃g���N
		double torq_emg[2][3]				�ؓd�x�[�X�̃g���N
		double torq[2][3]�@�@�@�@�@�@�@�@�@ ����������֐߃g���N
		
�o�́@�Fdouble torq[2][3]
********************************************************/
void Sum_ForceSensorAndEMG_BaseAssist(const double weight[2], double torq_forceSensor[2][3], double torq_emg[2][3], double torq[2][3]){

	//�p���[�A�V�X�g�g���N�̌v�Z
	/*for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			torq_forceSensor[i][j] *= (1-weight[i]);
			torq_emg[i][j]         *= (weight[i]);
			torq[i][j]              = torq_forceSensor[i][j] + torq_emg[i][j];
		}
	}*/
	for (int i = 0; i<2; i++) {
		for (int j = 0; j<3; j++) {

			torq[i][j] = torq_emg[i][j];
		}
	}

}


/********************************************************
�֐����FWeight_ForceSensorAndEMG
�����@�FEMG��ForceSensor�̃E�G�C�g��
		EMG��Forcesensor�x�[�X�̐���̃E�G�C�g�������� �V�O���C�h�֐��g�p
//		(�ؓd�̒l��1.5[V]��0.5,	2[V]���炢��1.0�ɂȂ�悤�ɒ���)�K��
�����@�Fdouble emg					��ڋؖ����؂̋ؓd��rms���������l
		
�߂�l�F1.0/(1.0+exp(-1.0*(-12.0+8*emg)))	EMG��weight(ForceSensor�́@1-weight )
********************************************************/
inline double Weight_ForceSensorAndEMG(double emg){
	
	return 1.0/(1.0+exp(-1.0*8*(-1.5+emg*1)));		//1.0����㏸���n��2.0�ŏ㏸���I���
	//return 1.0/(1.0+exp(-1.0*(-7.5+9*emg*1.0)));		//0.5����㏸���n��1.25�ŏ㏸���I���
	
}

/********************************************************
�֐����FP_Controlr
�����@�FP����̃C�����C���֐�
�����@�Fgain		 �e���W��
�@�@�@�@currentPos�@ ���݂̈ʒu
�@�@�@�@targetPos    �ڕW�̈ʒu
		
�߂�l�F(gain)*( (targetPos) - (currentPos) )
********************************************************/
inline double P_Control(double gain, double currentPos, double targetPos){
	
	return	(  (gain)*( (targetPos) - (currentPos) )  );

}


/********************************************************
�֐����FPD_Control
�����@�FPD����̊֐�
�����@�Fgain		 �e���W��
�@�@�@�@currentPos�@ ���݂̈ʒu
�@�@�@�@targetPos    �ڕW�̈ʒu
		
�߂�l�F(gain)*( (targetPos) - (currentPos) )
********************************************************/
double PD_Control(double gain[2], double current[2], double target[2]){
	const int P=0, D=1;

	return	(  gain[P]*(target[P] - current[P]) + gain[D]*(target[D] - current[D]) );

}