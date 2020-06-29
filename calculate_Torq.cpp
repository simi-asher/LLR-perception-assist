/*****************************************************************************************************************************
******************************************************************************************************************************
�g���N�v�Z�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��



******************************************************************************************************************************
*****************************************************************************************************************************/

#include "calculate_Torq.h"


/********************************************************
�֐����FInitialize_Torq
�����@�F�g���N�̏������Ɩ��C�ۏ�̐ݒ�
		���C�����[�v�ŌĂяo��
�����@�Fstruct torq_data *torq		�g���N�v�Z�p�̍\����
	�@�@
�o�́@�Fstruct torq_data *torq
********************************************************/
void Initialize_Torq(struct torq_data *torq){

	Reset_torque(torq);

	//Friction_Compensation(ON, torq);

}


/********************************************************
�֐����FCalculate_Torq
�����@�F�o�̓g���N�̌v�Z
		���C�����[�v�ŌĂяo��
�����@�Fstruct torq_data *torq		�g���N�v�Z�p�̍\����
	�@�@
�o�́@�Fstruct torq_data *torq
********************************************************/
void Calculate_Torq(struct torq_data *torq){
	Friction_Compensation(OFF, torq);//prev ON
	Calculate_Final_Torque(torq);

	Calculate_Output_Torque(torq);

}


/********************************************************
�֐����FReset_torque
�����@�F�g���N�̏�����
�����@�Fstruct torq_data *torq		�g���N�v�Z�p�̍\����
	�@�@
�o�́@�Fstruct torq_data *torq
********************************************************/
void Reset_torque(struct torq_data *torq){

	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			(*torq).pow[i][j]          = 0.0;
			(*torq).per[i][j]          = 0.0;
			(*torq).gra[i][j]	       = 0.0;
			(*torq).friction_for[i][j] = 0.0;
			(*torq).friction_rev[i][j] = 0.0;
			(*torq).per_comp = 0.0;
		}
	}
		
}



/********************************************************
�֐����FReset_torque
�����@�F�g���N�̏�����
�����@�Fconst int flag				���C�ۏ�����邩�ǂ���
		struct torq_data *torq		�g���N�v�Z�p�̍\����
	�@�@
�o�́@�Fstruct torq_data *torq
********************************************************/
void Friction_Compensation(const int flag, struct torq_data *torq){

	switch(flag){
	case ON:
		if(torq->per[R][H]!=0){
		//�@���[�^�̖��C�ۏ�(���])�@
		(*torq).friction_for[R][H] = FRICTION_FORWARD_RH;
		(*torq).friction_for[R][K] = FRICTION_FORWARD_RK;
		(*torq).friction_for[R][A] = FRICTION_FORWARD_RA;
		(*torq).friction_for[L][H] = FRICTION_FORWARD_LH;
		(*torq).friction_for[L][K] = FRICTION_FORWARD_LK;
		(*torq).friction_for[L][A] = FRICTION_FORWARD_LA;
		//�@���[�^�̖��C�ۏ�(�t�])
		(*torq).friction_rev[R][H] = FRICTION_REVERSE_RH;
		(*torq).friction_rev[R][K] = FRICTION_REVERSE_RK;
		(*torq).friction_rev[R][A] = FRICTION_REVERSE_RA;
		(*torq).friction_rev[L][H] = FRICTION_REVERSE_LH;
		(*torq).friction_rev[L][K] = FRICTION_REVERSE_LK;
		(*torq).friction_rev[L][A] = FRICTION_REVERSE_LA;
		}
	//	printf("%lf\n",(*torq).friction_for[R][H]);
		break;

	case OFF:
		break;
	}

}


/********************************************************
�֐����FCalculate_Final_Torque
�����@�F�ŏI�I�ȃg���N�̌v�Z
�����@�Fstruct torq_data *torq		�g���N�v�Z�p�̍\����
	�@�@
�o�́@�Fstruct torq_data *torq
********************************************************/
void Calculate_Final_Torque(struct torq_data *torq){
				

	// total torque = power-assist torque + perception-assist torque + gravity compensation torque
	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			(*torq).fin[i][j] = (*torq).pow[i][j] + (*torq).per[i][j] + (*torq).gra[i][j];
		}
	}	
	//std::cout << (*torq).fin[R][H] << "\n";
	// + friction compensation torque
	for(int i=0; i<2; i++){
		for(int j=0; j<3;j++){
			if( (*torq).fin[i][j] >= 0.0){
				(*torq).fin[i][j] += (*torq).friction_for[i][j];
			}else{
				(*torq).fin[i][j] -= (*torq).friction_rev[i][j];
			}
		}
	}
}


/********************************************************
�֐����FCalculate_Output_Torque
�����@�F�o�̓g���N�̌v�Z
�����@�Fstruct torq_data *torq		�g���N�v�Z�p�̍\����
	�@�@
�o�́@�Fstruct torq_data *torq
********************************************************/
void Calculate_Output_Torque(struct torq_data *torq){

	//�g���N��d���ɕϊ�
	//switched RH and RA
	(*torq).out[R][H] = 1.1*((*torq).fin[R][H] - (*torq).per_comp*0.35) / TORQUE_CONSTANT_HIP * ASSIST_RATE;//
	(*torq).out[R][K] = 1.1*((*torq).fin[R][K] - (*torq).per_comp*0.3) / TORQUE_CONSTANT_KNEE * ASSIST_RATE; //-ve for asc and desc //change at run time
	(*torq).out[R][A] = 0.4*(*torq).fin[R][A] / TORQUE_CONSTANT_ANKLE * ASSIST_RATE;
	(*torq).out[L][H] = 1.1*(*torq).fin[L][H] / TORQUE_CONSTANT_HIP * ASSIST_RATE;
	(*torq).out[L][K] = 1.1*(*torq).fin[L][K] / TORQUE_CONSTANT_KNEE * ASSIST_RATE;
	(*torq).out[L][A] = 0.4*(*torq).fin[L][A] / TORQUE_CONSTANT_ANKLE * ASSIST_RATE;

	//std::cout << (*torq).out[R][H] << "\n";
	//����̐ݒ�
	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			if     ( (*torq).out[i][j]> VOLT_LIMIT ) (*torq).out[i][j] =  VOLT_LIMIT;
			else if( (*torq).out[i][j]<-VOLT_LIMIT ) (*torq).out[i][j] = -VOLT_LIMIT;
		}
	}


	//�o�̓g���N(-�̏ꍇ+�ɕϊ�)�Ɖ�]�����̐ݒ�
	for(int i=0; i<2; i++){
		for(int j=0; j<3;j++){
			if( (*torq).out[i][j] >= 0.0){
				(*torq).out[i][j]  = (*torq).out[i][j];
				(*torq).sw[i][j]   = 0.0;
			}else{
				(*torq).out[i][j]  = -(*torq).out[i][j];//prev -ve
				(*torq).sw[i][j]   = 5.0;
			}
		}
	}
}


/********************************************************
�֐����FConvert_Torque_to_Force
�����@�F�e�֐߂̃g���N����ܐ�̗͂ɕϊ�
�����@�Fstruct leg_state *Leg	//���̏�ԁi���r�C�V�r�j
		double jacob_ti[2][3]	//���R�r�̋[���t�s��̓]�u
		double torq[2][3]		//�e�֐߂̃g���N
		double force[2]			//�ܐ�ɉ�����
	�@�@
�o�́@�Fdouble force[2]
********************************************************/
void Convert_Torque_to_Force(struct leg_state *Leg, double jacob_ti[2][3], double torq[2][3], double force[2]){

	const int SUP = Leg->support;
	const int SW  = Leg->swing;

	force[X] = torq[SW][H]*jacob_ti[0][0] + torq[SW][K]*jacob_ti[0][1] + torq[SW][A]*jacob_ti[0][2];		//+�ȋC������E�E�E
	force[Y] = torq[SW][H]*jacob_ti[1][0] + torq[SW][K]*jacob_ti[1][1] + torq[SW][A]*jacob_ti[1][2];

}


/********************************************************
�֐����FConvert_Torque_to_Force
�����@�F�ܐ�̗͂���e�֐߂̃g���N�ɕϊ�
�����@�Fstruct leg_state *Leg	//���̏�ԁi���r�C�V�r�j
		double jacob_t[3][2]	//���R�r�s��̓]�u
		double force[2]			//�ܐ�ɉ�����
		double torq[2][3]		//�e�֐߂̃g���N
	�@�@
�o�́@�Fdouble torq[2][3]
********************************************************/
//void Convert_force_to_torque(struct leg_state *Leg, double jacob_t[3][2], double force[2], double torq[2][3]){
//
//	const int SUP = Leg->support;
//	const int SW  = Leg->swing;
//
//	torq[SW][H] = force[X]*jacob_t[0][0] + force[Y]*jacob_t[0][1];
//	torq[SW][K] = force[X]*jacob_t[1][0] + force[Y]*jacob_t[1][1];			//+�ȋC������
//	torq[SW][A] = force[X]*jacob_t[2][0] + force[Y]*jacob_t[2][1];	
//
//}


/********************************************************
�֐����FConvertTorqueToForce
�����@�F�e�֐߂̃g���N����ܐ�̗͂ɕϊ�
�����@�Fstruct leg_state *Leg	     ���̏�ԁi���r�C�V�r�j
		struct jacob_data *jacobData ���R�r�s��
		double torq[2][3]		     �e�֐߂̃g���N
		double force[3]				 �ܐ�ɉ�����
	�@�@
�o�́@�Fdouble force[3]
********************************************************/
void ConvertTorqueToForce(struct leg_state *Leg, struct jacob_data *jacobData, double torq[2][3], double force[3]){

	const int SUP = Leg->support;
	const int SW  = Leg->swing;

	for(int i=0;i<3;i++)	force[i] = 0;

	for(int i=0;i<3;i++){
		force[X] += torq[SW][i]*jacobData->pse_trans[0][i];
		force[Y] += torq[SW][i]*jacobData->pse_trans[1][i];
		force[Z] += torq[SW][i]*jacobData->pse_trans[2][i];
	}
}


/********************************************************
�֐����FConvertForceToTorque
�����@�F�ܐ�̗͂���e�֐߂̃g���N�ɕϊ�
�����@�Fstruct leg_state *Leg	//���̏�ԁi���r�C�V�r�j
		double jacob_t[3][2]	//���R�r�s��̓]�u
		double force[2]			//�ܐ�ɉ�����
		double torq[2][3]		//�e�֐߂̃g���N
	�@�@
�o�́@�Fdouble torq[2][3]
********************************************************/
void ConvertForceToTorque(struct leg_state *Leg, struct jacob_data *jacobData, double force[2], double torq[2][3]){

	const int SUP = Leg->support;
	const int SW  = Leg->swing;

	//for(int i=0;i<2;i++)	torq[SUP][i] = 0; //prev SW
	
	for (int i = 0;i < 2;i++) {//prev 3
		torq[SUP][A] += force[i] * jacobData->trans[0][i];
		torq[SUP][K] += force[i] * jacobData->trans[1][i];
		torq[SUP][H] += force[i] * jacobData->trans[2][i];
		torq[SW][H] += force[i] * jacobData->trans[3][i];
		torq[SW][K] += force[i] * jacobData->trans[4][i];
		torq[SW][A] += force[i] * jacobData->trans[5][i];
	}
	
	
	
}

void ConvertForceToTorque(struct leg_state* Leg, struct jacob_hip* jh, struct jacob_body* jb, double force[2], double torq[2][3]) {
	const int SUP = Leg->support;
	const int SW = Leg->swing;

	//for(int i=0;i<2;i++)	torq[SUP][i] = 0; //prev SW
	//so that SW gets SUP values even tho SUP angles used to calc
	for (int i = 0;i < 2;i++) {//prev 3
		torq[SW][A] += force[i] * jb->trans[0][i];
		torq[SW][K] += force[i] * jb->trans[1][i];
		torq[SW][H] += force[i] * jb->trans[2][i];
		
	}
	
	torq[SUP][H] = -torq[SW][H];

	for (int i = 0;i < 2;i++) {//prev 3
		torq[SW][A] += force[i] * jh->trans[0][i];
		torq[SUP][K] += force[i] * jh->trans[1][i];
	}

	torq[SUP][A] = -torq[SW][A];
	torq[SW][K] = -torq[SUP][K];
	//make SUP side have -ve val of SW
	
}

void ConvertForceToTorque(struct leg_state* Leg, struct jacob_body* jacobData, double force[2], double torq[2][3]) {
	const int SUP = Leg->support;
	const int SW = Leg->swing;

	for (int i = 0;i < 2;i++) {//prev 3
		torq[SW][H] += force[i] * jacobData->trans[0][i];
		torq[SW][K] += force[i] * jacobData->trans[1][i];
		torq[SW][A] += force[i] * jacobData->trans[2][i];

	}
}

void Calculate_Perception_Compensation(struct leg_state* Leg, struct torq_data* torq, struct body_posi* body_posi, double enc_data[2][3]) {
	const int SUP = Leg->support;
	const int SW = Leg->swing;
	double f_body = 0.0,d_t,d_l,d_b, ang_body;

	d_t = sqrt(pow(body_posi->Hip[SW][X] - (*body_posi).UppThigh[SW].pos[X], 2.0) + pow(body_posi->Hip[SW][Y] - (*body_posi).UppThigh[SW].pos[Y], 2.0));
	d_l = sqrt(pow(body_posi->Knee[SW][X] - (*body_posi).LowThigh[SW].pos[X], 2.0) + pow(body_posi->Knee[SW][Y] - (*body_posi).LowThigh[SW].pos[Y], 2.0));
	d_b= sqrt(pow(body_posi->Hip[SW][X] - (*body_posi).Body.pos[X], 2.0) + pow(body_posi->Hip[SW][Y] - (*body_posi).Body.pos[Y], 2.0));
	ang_body = Calculation_Body_Orientation(enc_data[SUP]); //simi the old way-use for sit squat

	f_body = ((*torq).per[SW][H] / d_t) * ((*body_posi).UppThigh[SW].pos[X] * sin(enc_data[SW][K]) - (*body_posi).UppThigh[SW].pos[Y] * cos(enc_data[SW][H])) 
		/ ((*body_posi).Body.pos[X] * sin(ang_body) - (*body_posi).Body.pos[Y] * cos(ang_body))	- ((*torq).per[SW][K] / d_l) * ((*body_posi).LowThigh[SW].pos[X] 
			* sin(enc_data[SW][K] - enc_data[SW][H]) - (*body_posi).LowThigh[SW].pos[Y] * cos(enc_data[SW][K] - enc_data[SW][H])) / ((*body_posi).Body.pos[X] 
			* sin(ang_body) - (*body_posi).Body.pos[Y] * cos(ang_body));
	torq->per_comp = f_body * d_b;
}