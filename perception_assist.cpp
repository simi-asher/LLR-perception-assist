/*****************************************************************************************************************************
******************************************************************************************************************************
�F�m�A�V�X�g�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��
urg.cpp�Ōv�Z������Q���܂ł̋����C������
calculate.cpp�Ōv�Z��������ʒu����F�m�A�V�X�g�ɕK�v�ȗ͂��v��


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "perception_assist.h"

double virWall_height=0, virWall_distance=0,x_dist=0, vw_grad =0,vw_height=0;
/********************************************************
�֐����FCalculate_Perception_Assist
�����@�F���C�����[�v�̒��ŌĂяo���֐�
		URG�̃f�[�^�����╨�̌��o�͂��̊֐����ōs��
�����@�Furg_t *urg							URG�̐ݒ�p�̍\����
	�@�@long *length_data					�����f�[�^
	    const int *length_data_size			�f�[�^�̌�(for����̈�m�ۂɎg�p)
		int *time							�v���ɂ�����������
		struct URG_data *urgData			�����f�[�^��X-Y���W�ɕϊ������l(�������Ȃǂ̏������܂�)
		struct Obstacle_data *obstacleData	���̌��o��̃f�[�^(��Q���̍����Ȃ�)

�o�́@�Fstruct Obstacle_data *obstacleData
********************************************************/
void Calculate_Perception_Assist(struct Obstacle_data *obstacleData, struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData, int judge){

	FromGroundToOrigin(obstacleData, jointPos, Leg, perData);
	
	//DetectMotion(jointPos, perData);//CCC
	(*perData).flag   = true;

	//SetVirtualWall(perData,judge);

	DetectDanger(jointPos, Leg, perData, judge);
	
	printf("height: %lf distance: %lf\n",(*perData).height, (*perData).distance);
	//printf("%lf  %lf\n",(*perData).height, (*perData).distance);
}


/********************************************************
�֐����FFromGroundToOrigin
�����@�F�Z���T�^���̈ʒu���烍�{�b�g�̌��_�ɕϊ�����
�����@�Fstruct Obstacle_data *obstacleData	���̌��o��̃f�[�^(��Q���̍����Ȃ�)
		struct body_posi *jointPos0
		const struct leg_state *Leg
		struct Per_data *perData			�F�m�A�V�X�g�ɕK�v�ȃf�[�^

�o�́@�Fstruct Per_data *perData
********************************************************/
void FromGroundToOrigin(struct Obstacle_data *obstacleData, struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData){

	const int SUP = Leg->support;
	const int SW  = Leg->swing;
	//imp
	//every value except x=0 for obstacleData[x].distance is trash //to make measurement easier (set 0 to toe) -LEN_A_T
	(*perData).distance = obstacleData[0].distance; // +jointPos->Urg[SUP][X]; // + LEN_K_U instead of URG  //check if URG part still needed 2020
	(*perData).height   = obstacleData[0].height;
	(*perData).depth    = obstacleData[0].depth;
	//printf("%lf  %lf %lf\n", (*perData).height, (*perData).distance, (*perData).depth);
	
	//(*perData).distance = 0.35+LEN_A_T;
	//(*perData).height   = 0.16;
	//(*perData).depth    = 0.3;
	
	

}


/********************************************************
�֐����FDetectMotion
�����@�F���ǂ���������Ȃ̂��̔���
�����@�Fstruct body_posi *jointPos
		struct Per_data *perData			�F�m�A�V�X�g�ɕK�v�ȃf�[�^

�o�́@�Fstruct Per_data *perData
********************************************************/
void DetectMotion(struct body_posi *jointPos, struct Per_data *perData){

	if(jointPos->XZmp > DETECT_MOTION_THRESHOLD){
		(*perData).motion = GAIT_MOTION;
		(*perData).flag   = true;
	}else{
		(*perData).motion = FOOT_MOTION;
		(*perData).flag   = false;
	}

	(*perData).motion = GAIT_MOTION;
	(*perData).flag   = true;
}


/********************************************************
�֐����FSetVirtualWall
�����@�F���z�ǂ̐ݒ�
�����@�Fstruct Per_data *perData			�F�m�A�V�X�g�ɕK�v�ȃf�[�^

�o�́@�Fstruct Per_data *perData
********************************************************/
void SetVirtualWall(struct Per_data *perData, int judge){
	if(judge==1){ //asc
		(*perData).virWall.grad = 1.3808*(*perData).height - 0.10775;
		(*perData).virWall.intr = ( (*perData).height + OVER_HEIGHT ) - (*perData).virWall.grad*VIRWALL_DEPTH;
		//(*perData).virWall.depth = VIRWALL_DEPTH;
		printf("wall: asc");
	}
	else if (judge ==2){//desc
		(*perData).virWall.grad = 0;//flat rectangle
		(*perData).virWall.intr = 2*(*perData).height ;
		//(*perData).virWall.depth = VIRWALL_DEPTH; //=(+)0.3
		printf("wall: desc");
	}
	else if (judge ==3){//sit
		(*perData).virWall.grad = 0;//flat rectangle
		(*perData).virWall.intr = 2*(*perData).height ;//bc height -ve check again if negative??
		//(*perData).virWall.depth = VIRWALL_DEPTH; //=(+)0.3
		printf("wall: sit");
	}
	

}


/********************************************************
�֐����FDetectDanger
�����@�F�F�m�A�V�X�g���K�v���ǂ����̔���
�����@�Fstruct Obstacle_data *obstacleData	���̌��o��̃f�[�^(��Q���̍����Ȃ�)
		struct body_posi *jointPos
		const struct leg_state *Leg
		struct Per_data *perData			�F�m�A�V�X�g�ɕK�v�ȃf�[�^

�o�́@�Fstruct Per_data *perData
********************************************************/
void DetectDanger(struct body_posi *jointPos, const struct leg_state *Leg, struct Per_data *perData, int judge){

	static bool flag=false; //false //CCC
	
	const int SUP = Leg->support;
	const int SW  = Leg->swing;

	//pre_distance=virWall_distance;
	//pre_height=virWall_height;
	if(judge==1){ //only for asc
		//virWall_distance = jointPos->Toe[SW][X] - ((*perData).distance - VIRWALL_DEPTH); //v wall X dep on toe
		//virWall_height   = (*perData).virWall.grad*( virWall_distance ) + (*perData).virWall.intr; //v wall Y dep on toe
		//(*perData).virWall.target = ((*perData).virWall.intr + TARGET_HEIGHT); //prev  (*perData).virWall.grad*( VIRWALL_DEPTH - virWall_distance ) + ( (*perData).virWall.intr + TARGET_HEIGHT );
		////prev x_dist = jointPos->Toe[SW][X] - ((*perData).distance - VIRWALL_DEPTH);
		//printf("dist %lf, height %lf, target %lf\n", virWall_distance, virWall_height, (*perData).virWall.target); //simi cur print
		////if( virWall_distance < VIRWALL_DEPTH && virWall_distance>-0.05 && (*perData).distance > 0.3 && jointPos->Toe[SW][Y] < (*perData).virWall.target){//prev dist 0.4
		(*perData).virWall.end_y = (*perData).height + 0.03;//height of vw near step
		(*perData).virWall.start_x = (*perData).distance - VIRWALL_DEPTH; //start of vw
		(*perData).virWall.end_x = (*perData).distance;
		(*perData).virWall.grad = 0.4;
		(*perData).virWall.start_y = (*perData).virWall.end_y - (VIRWALL_DEPTH * (*perData).virWall.grad); //height at start of v_wall
		vw_height = (*perData).virWall.grad * (jointPos->Toe[SW][X] - (*perData).virWall.start_x) + (*perData).virWall.start_y; //height of v wall at toe x pos
		x_dist = jointPos->Toe[SW][X] - (*perData).virWall.start_x;
		if(jointPos->Toe[SW][X] > (*perData).virWall.start_x && jointPos->Toe[SW][Y] < vw_height && jointPos->Toe[SW][X]< (*perData).distance) {
		//if (virWall_distance < (*perData).distance && virWall_distance>0 && jointPos->Toe[SW][Y] < (*perData).virWall.target) {//prev dist 0.4
			PerceptionForce(jointPos, Leg, P_CTRL, perData, x_dist,judge);
			flag = true;
		}else{
			(*perData).virWall.force[X] = 0;
			(*perData).virWall.force[Y] = 0;
			(*perData).virWall.error = 0.0;
			flag = false;
		}
	}
	else if(judge==2){//desc
		//(*perData).virWall.target=(*perData).height+0.05;
		//virWall_distance = jointPos->Toe[SW][X] - ((*perData).distance + VIRWALL_DEPTH); //v wall X dep on toe
		//virWall_height = jointPos->Toe[SW][Y]; //prev (*perData).virWall.grad * (virWall_distance)+(*perData).virWall.intr; //v wall Y dep on toe
		////x_dist=jointPos->Toe[SW][X]-((*perData).distance-(*perData).depth+0.05);//check if obstacleData[0].distance works better bc of bending of SUP Ankle
		(*perData).virWall.grad = 0;
		//(*perData).virWall.height = 0.02;
		(*perData).virWall.start_y = -(*perData).height + 0.02;
		(*perData).virWall.start_x = ((*perData).distance - (*perData).depth); //start of vw
		(*perData).virWall.end_x = (*perData).virWall.start_x +0.2;
		(*perData).virWall.end_y = (*perData).virWall.start_y + (*perData).virWall.grad * ((*perData).virWall.end_x - (*perData).virWall.start_x);
		x_dist = jointPos->Toe[SW][X] - (*perData).virWall.start_x;
		if (jointPos->Toe[SW][X] > (*perData).virWall.start_x && jointPos->Toe[SW][X] < (*perData).virWall.end_x && jointPos->Toe[SW][Y] < (*perData).virWall.start_y) {
		//if(virWall_distance >=0 && jointPos->Toe[SW][Y] > (*perData).virWall.target ){//if foot is in virtual wall
			PerceptionForce(jointPos, Leg, P_CTRL, perData, x_dist,judge);
			flag = true;
		}else{
		(*perData).virWall.force[X] = 0;
		(*perData).virWall.force[Y] = 0;
		(*perData).virWall.error = 0.0;
		flag = false;
		}
	}
	else if (judge == 3) {//sit
		//(*perData).virWall.target = (*perData).height + 0.05;
		//virWall_distance = -jointPos->Hip[SUP][X] + ((*perData).distance + VIRWALL_DEPTH); //v wall X dep on toe, +ve val
		//virWall_height = jointPos->Hip[SUP][Y]; //prev (*perData).virWall.grad * (virWall_distance)+(*perData).virWall.intr; //v wall Y dep on toe
		////x_dist=jointPos->Toe[SW][X]-((*perData).distance-(*perData).depth+0.05);//check if obstacleData[0].distance works better bc of bending of SUP Ankle
		(*perData).virWall.start_y = (*perData).height + 0.4;
		(*perData).virWall.grad = 0;
		(*perData).virWall.start_x = ((*perData).distance + 0.5); //start of vw
		(*perData).virWall.end_x = ((*perData).distance); //end of vw
		(*perData).virWall.end_y = (*perData).virWall.start_y + (*perData).virWall.grad * ((*perData).virWall.end_x - (*perData).virWall.start_x);
		x_dist = 0;
		if (jointPos->Hip[SUP][X]<(*perData).virWall.start_x && jointPos->Hip[SUP][X]>(*perData).distance && jointPos->Hip[SUP][Y] < (*perData).virWall.start_y) {
		//if (virWall_distance > 0 && virWall_distance < VIRWALL_DEPTH) {
			PerceptionForce(jointPos, Leg, P_CTRL, perData, x_dist, judge);
			flag = true;
		}
		else {
			(*perData).virWall.force[X] = 0;
			(*perData).virWall.force[Y] = 0;
			(*perData).virWall.error = 0.0;
			flag = false;
		}
	}
		(*perData).perFlag = flag;
	

}


/********************************************************
�֐����FPerceptionForce
�����@�F�F�m�A�V�X�g�ɂ�蔭�������
�����@�Fstruct body_posi *jointPos
		const struct leg_state *Leg
		struct Per_data *perData			�F�m�A�V�X�g�ɕK�v�ȃf�[�^

�o�́@�Fstruct Per_data *perData
********************************************************/
void PerceptionForce(struct body_posi *jointPos, const struct leg_state *Leg, int mode, struct Per_data *perData, double x_dist, int judge){

	const int SUP = Leg->support;
	const int SW  = Leg->swing;
	//check if -ve X dot makes motion difficult
	//PD control
	//asc k=300,B=100
	const double ELASTIC_COEFF_X = 10,ELASTIC_COEFF_Y=100, VISCOSITY_COEFF = 10;//prev ELASTIC 300
	if (judge==1){//asc
		(*perData).virWall.force[X] = -1.5*x_dist-1.5*((*jointPos).Foot[R].vel[X]); //prev 0
		(*perData).virWall.force[Y] = 12; //prev 50 //ELASTIC_COEFF_Y* ((*perData).virWall.target - jointPos->Toe[SW][Y]);
	}
	else if(judge==2){//desc
		(*perData).virWall.force[X] = -5 * x_dist - 5 * ((*jointPos).Foot[R].vel[X]); //prev 0
	 	(*perData).virWall.force[Y] = -5;

	}
	else if (judge == 3) {//sit
		
		(*perData).virWall.force[Y] = -5;//-ve to make user squat, +ve if stand
		if ((*jointPos).XZmp < -0.15) {
			(*perData).virWall.force[X] = 5; //50;
		}
		else if ((*jointPos).XZmp > -0.05) {
			(*perData).virWall.force[X] = 1; //10;
		}
		else {
			(*perData).virWall.force[X] = (-(5-1) * (*jointPos).XZmp/0.1) - 1; //-400 is slope,10 is 
		}
	}
	// switch(mode){
	// case P_CTRL:
	// 	(*perData).virWall.force[X] = ELASTIC_COEFF*x_dist; //prev 0
	// 	(*perData).virWall.force[Y] = ELASTIC_COEFF*( (*perData).virWall.target - jointPos->Toe[SW][Y] );
	// 	if((*perData).virWall.force[Y]>50) (*perData).virWall.force[Y] = 50; //prev 100
	// 	break;
	// case PD_CTRL:
	// 	break;
	// case CONST_CTRL:
	// 	(*perData).virWall.force[X] = 0;//ELASTIC_COEFF*( (*perData).virWall.target - jointPos->Toe[SW][X] )-VISCOSITY_COEFF*;
	// 	(*perData).virWall.force[Y] = 10;
	// 	break;
	// }


}
