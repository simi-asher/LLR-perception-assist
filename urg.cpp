/*****************************************************************************************************************************
******************************************************************************************************************************
���[�U�[�����W�t�@�C���_(URG-04LX)�g�p�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��
�\�t�g�E�F�A�Furg_library-1.2.0�g�p


******************************************************************************************************************************
*****************************************************************************************************************************/

#include <algorithm> 
#include "urg.h"
//simi
/*#include <iostream>
#include <fstream>
using namespace std;
ofstream fil;
fil.open("bump.dat",ios::out | ios::binary);*/

/********************************************************
�֐����FCalculate_URG
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
void Calculate_URG(urg_t *urg, long *length_data, int *length_data_size, int *time, struct URG_data *urgData, struct Obstacle_data *obstacleData){
	
	
	//�����̌v��	 Distance measurement
	//�}���`�X���b�h���Ŏg�p���Ă��邽�߂��̊֐����ł̓R�����g�A�E�g���Ă���
	//Measure_URG(urg, length_data, length_data_size, time);

	//�v���f�[�^�̏���
	ChangeZeroPoint(length_data, *length_data_size); //Changes distance to every pt that is less than 4cm and greater than 4m is 0
	
	ChangeCoordinate(urg, length_data, *length_data_size, urgData); //Convert distance data to X-Y coordinates. front is positive. i goes from front to back

	DeleateOfNotMeasureData(*length_data_size, urgData); //if X&Y is 0 then make same as neighbor
	
	FromSensorToGround(URG_ON, urg, *length_data_size, urgData); //obstacle y is -ve. Convert the origin from the sensor position to the ground directly below the sensor

	//���f�[�^�ł�������x�̐��x�Ō��o�ł����̂Ńt�B���^�[�͎g�p���Ă��Ȃ�
	Smoothing_Data(NonFilter, *length_data_size, urgData); //nothing for non-filter

	//�T���p�Ƀf�[�^�̏C��
	ChangeData(urgData, urg, *length_data_size, obstacleData); //simi array ordering centre->front->center->back,y sign reversed to make positive 

	//��Q���̒T��
	DetectLine_SAM(obstacleData, *length_data_size);

	printf(" %lf\n",obstacleData[0].distance); //simi uncommented
}


/********************************************************
�֐����FInit_URG
�����@�F���C�����[�v�ɓ���O�ɌĂяo���֐�(�������̈���m�ۂ���O�ɌĂяo������)
		URG�̏����ݒ�
�����@�Furg_t *urg						URG�̐ݒ�p�̍\����
	�@�@const long connect_baudrate		�ʐM���x
	    const char connect_device[]		�|�[�g�ԍ�

�o�́@�Furg_t *urg
********************************************************/
void Init_URG(urg_t *urg, const long connect_baudrate, const char connect_device[]){
	int ret=0;
	int first_step;
	int last_step;

// URG�̐ڑ��Ɗe��ݒ�
	ret = urg_open(urg, URG_SERIAL, connect_device, connect_baudrate);
	if(ret<0){
		printf("URG���ڑ�����Ă��܂���\n�R���|�[�g���m�F���Ă�������\n\n");
		getchar();
		exit(-1);
	}
	first_step = urg_deg2step(urg, URG_FirstDeg);
	last_step  = urg_deg2step(urg, URG_LastDeg);
	urg_set_scanning_parameter(urg, first_step, last_step, SkipStep);
}


/********************************************************
�֐����FMeasure_URG
�����@�F�����̌v��
�����@�Furg_t *urg						URG�̐ݒ�p�̍\����
	�@�@long *length_data				�����f�[�^
	    int *length_data_size			�f�[�^�̌�
		int *time						�v���ɂ�����������

�o�́@�Flong *length_data
		int *length_data_size
		int *time
********************************************************/
void Measure_URG(urg_t *urg, long *length_data, int *length_data_size, int *time){
	long timestamp;
	static long preTimestamp=0;

	//�������v������
	urg_start_measurement(urg, URG_DISTANCE, ScanTimes, SkipScan);
	//�f�[�^�̒���
    *length_data_size = urg_get_distance(urg, length_data, &timestamp);

	//�v���ɂ�����������
	*time = timestamp - preTimestamp;
	
	preTimestamp = timestamp;
}


/********************************************************
�֐����FKeepData
�����@�F�f�[�^�̊m��
�����@�Fstruct URG_data *beforeData		�c���Ă��������f�[�^
	    const int length_data_size		�f�[�^�̌�
		struct URG_data *afterData		�m�ۗp�̃f�[�^

�o�́@�Fstruct URG_data *afterData
********************************************************/
void KeepData(struct URG_data *beforeData, const int length_data_size, struct URG_data *afterData){
	
	for(int i=0; i<length_data_size; i++){
		afterData[i].x = beforeData[i].x;	afterData[i].y = beforeData[i].y;
	}
}



/********************************************************
�֐����FEndMeasure
�����@�F���C�����[�v�I����ɌĂяo���֐�(�������̈���J������O�ɌĂяo������)
		URG�̐ڑ������
�����@�Furg_t *urg						URG�̐ݒ�p�̍\���́@
********************************************************/
void EndMeasure(urg_t *urg){

	urg_close(urg);		//URG�̐ڑ������
}


/********************************************************
�֐����FChangeZeroPoint
�����@�F�v���͈͊O��0�ցi40mm�`4m�j
�����@�Flong *length_data				�����f�[�^
	    const int *length_data_size		�f�[�^�̌�

�o�́@�Flong *length_data
********************************************************/
void ChangeZeroPoint(long *length_data, const int length_data_size){
	for(int i=0; i<length_data_size; ++i){
		if(length_data[i]<40 || length_data[i]>4000)
			length_data[i] = 0;
	}
}


/********************************************************
�֐����FChangeCoordinate
�����@�F�����f�[�^��X-Y���W�ɕϊ�
		�܂��Z���T���ʂ�length_data_size=0�ɂȂ�悤�ɕϊ�
		Convert distance data to X-Y coordinates
		Also convert the front of the sensor to length_data_size = 0
�����@�Furg_t *urg						URG�̐ݒ�p�̍\����
		long *length_data				�����f�[�^
	    const int *length_data_size		�f�[�^�̌�
		struct URG_data *urgData		�����f�[�^��X-Y���W�ɕϊ������l(���d�v)

�o�́@�Fstruct URG_data *urgData
********************************************************/
void ChangeCoordinate(urg_t *urg, long *length_data, const int length_data_size, struct URG_data *urgData){
	double radian;

	for(int i=0; i<length_data_size; ++i){
		
		radian = urg_index2rad(urg, i*SkipStep);
		
		urgData[i].rawX = -(length_data[i] * sin(radian))/1000; //-(length_data[i]*cos(radian))/1000
		urgData[i].rawY =  (length_data[i] * cos(radian))/1000; //length_data[i]*sin(radian))/1000
		//printf("%d %lf %lf\n", i, urgData[i].rawX, urgData[i].rawY); 
	}
	//printf("\n");
	
}


/********************************************************
�֐����FDeleateOfNotMeasureData
�����@�F�v���͈͊O�̃f�[�^(x=y=0)���폜����
�����@�Fconst int *length_data_size		�f�[�^�̌�
		struct URG_data *urgData		�����f�[�^��X-Y���W�ɕϊ������l(���d�v)

�o�́@�Fstruct URG_data *urgData
********************************************************/
void DeleateOfNotMeasureData(const int length_data_size, struct URG_data *urgData){
	for(int i=length_data_size-2; i>=0; i--){
		if(urgData[i].rawX==0 && urgData[i].rawY==0){
			urgData[i].rawX = urgData[i+1].rawX;
			urgData[i].rawY = urgData[i+1].rawY;
		}
	}
}


/********************************************************
�֐����FFromSensorToGround
�����@�F���_���Z���T�ʒu����Z���T�^���̒n�ʂɕϊ�(ON:�n�ʌ��_, OFF:�Z���T���_)
�����@�Fconst int flag					ON or OFF(���W�ϊ������邩�ǂ���)
		urg_t *urg						URG�̐ݒ�p�̍\����
		const int *length_data_size		�f�[�^�̌�
		struct URG_data *urgData		�����f�[�^��X-Y���W�ɕϊ������l(���d�v)

�o�́@�Fstruct URG_data *urgData
********************************************************/
void FromSensorToGround(const int flag, urg_t *urg, const int length_data_size, struct URG_data *urgData) {
	double translation[2];

	//�Z���T���ʁi�^���j�̃C���f�b�N�X�ԍ����擾
	int front_index = urg_deg2step(urg, (URG_FrontDeg - URG_FirstDeg) / SkipStep);
	if (front_index*2 == length_data_size) //simi *2
	{
		front_index = front_index - 1;
		//printf("yes");
	}
	//printf("%d %d\n", length_data_size, front_index);
	//�Z���T�^��X-Y���W
	translation[X] = urgData[front_index].rawX;	translation[Y] = urgData[front_index].rawY;

	switch(flag){
	//���W�ϊ�����
	case ON:
	for(int i=length_data_size-1; i>=0; i--){
		urgData[i].x = urgData[i].rawX - translation[X];
		urgData[i].y = urgData[i].rawY - translation[Y];
		//printf("raw %d %lf %lf\n", i, urgData[i].x, urgData[i].y); //print
	}
	break;

	//���W�ϊ��Ȃ�
	case OFF:
	for(int i=length_data_size-1; i>=0; i--){
		urgData[i].x = urgData[i].rawX;
		urgData[i].x = urgData[i].rawY;
	}
	break;

	default:
	break;
	}

}


/********************************************************
�֐����FSmoothing_Data
�����@�F���_���Z���T�ʒu����Z���T�^���̒n�ʂɕϊ�(ON:�n�ʌ��_, OFF:�Z���T���_)
�����@�Fconst int filterMode			�t�B���^�̐ݒ�(��{NonFilter��DelayFilter)
		const int *length_data_size		�f�[�^�̌�
		struct URG_data *urgData		�����f�[�^��X-Y���W�ɕϊ������l(���d�v)

�o�́@�Fstruct URG_data *urgData
********************************************************/
void Smoothing_Data(const int filterMode, const int length_data_size, struct URG_data *urgData){
	double sum[2]={0};
	static double pre1[2][1000]={0}, pre2[2][1000]={0};

	switch(filterMode){

	/*************************************************************************
	//					NonFilter
	//					�t�B���^�Ȃ�
	**************************************************************************/
	case NonFilter:
	for(int i=0; i<length_data_size; i++){
		urgData[i].x = urgData[i].x;
		urgData[i].y = urgData[i].y;
	}

	break;

	/*************************************************************************
	//					AveragingFilter
	//					���ω��t�B���^
	**************************************************************************/
	case AveragingFilter:

	urgData[0].x = (urgData[0].x + urgData[1].x)/2;
	urgData[0].y = (urgData[0].y + urgData[1].y)/2;
	urgData[1].x = (urgData[0].x + urgData[1].x + urgData[2].x)/3;
	urgData[1].y = (urgData[0].y + urgData[1].y + urgData[2].y)/3;
		
	for(int j=2;j<length_data_size-2;j++){
		for(int k=-2;k<=2;k++){
			sum[X] += urgData[j+k].x;
			sum[Y] += urgData[j+k].y;
		}
		urgData[j].x = sum[X]/5;
		urgData[j].y = sum[Y]/5;
		sum[X] = 0;
		sum[Y] = 0;
	}
	
	urgData[length_data_size-2].x = (urgData[length_data_size-3].x + urgData[length_data_size-2].x + urgData[length_data_size-1].x)/3;
	urgData[length_data_size-2].y = (urgData[length_data_size-3].y + urgData[length_data_size-2].y + urgData[length_data_size-1].y)/3;
	urgData[length_data_size-1].x = urgData[length_data_size-1].x;
	urgData[length_data_size-1].y = urgData[length_data_size-1].y;
	break;


	/*************************************************************************
	//					GaussianFilter
	//					�K�E�V�A���t�B���^
	**************************************************************************/
	case GaussianFilter:

		urgData[0].x = (2*urgData[0].x + urgData[1].x)/3;
		urgData[0].y = (2*urgData[0].y + urgData[1].y)/3;
	

	for(int j=1;j<length_data_size-1;j++){
		sum[X] = 1*urgData[j-1].x + 7*urgData[j].x + 1*urgData[j+1].x;
		sum[Y] = 1*urgData[j-1].y + 7*urgData[j].y + 1*urgData[j+1].y;
		urgData[j].x = sum[X]/9;
		urgData[j].y = sum[Y]/9;
		sum[X] = 0;		sum[Y] = 0;
	}
	
	urgData[length_data_size-1].x = urgData[length_data_size-1].x;
	urgData[length_data_size-1].y = urgData[length_data_size-1].y;

	break;

	/*************************************************************************
	//					DelayFilter
	**************************************************************************/
	case DelayFilter:
		for(int i=0; i<length_data_size; i++){
			sum[X] = urgData[i].x + pre1[X][i] + pre2[X][i];
			sum[Y] = urgData[i].y + pre1[Y][i] + pre2[Y][i];
			urgData[i].x = sum[X]/3;	urgData[i].y = sum[Y]/3;
			
			pre2[X][i] = pre1[X][i];	pre2[Y][i] = pre1[Y][i]; 
			pre1[X][i] = urgData[i].x;	pre1[Y][i] = urgData[i].y;		
		}
		
	
	break;

	/*************************************************************************
	//					default����
	//					�����I��(-1)
	**************************************************************************/
	default:
	printf("ERROR Smoothing_Obstacle : Filter mode ��I�����Ă�������");
	exit(-1);
	}
}


/********************************************************
�֐����FChangeData
�����@�F�z��̐擪�Ɍ��_�f�[�^������悤�ɕύX
�����@�Fstruct URG_data *urgData
	�@�@const int length_data_size
	    struct Obstacle_data *obstacleData

�o�́@�Fstruct Obstacle_data *obstacleData
********************************************************/
void ChangeData(struct URG_data *urgData, urg_t *urg, const int length_data_size, struct Obstacle_data *obstacleData) {
	//��Q����URG���Ⴂ�ʒu�ɂ���ꍇURG�̋����f�[�^�͒n�ʂ�菬�����Ȃ�̂�Y���W�̒l�ɂ�-�����Ă���
	//URG��荂���ꍇ�����ݍl�����Ă��Ȃ����ߕK�v�ɉ����ďC�����K�v
	// If the obstacle is at a position below the URG, the URG distance data is smaller than the ground, so the Y coordinate value is marked with-
	// If higher than URG is not considered at present, correction is necessary if necessary
	//simi no reordering index
	for (int i = 0; i < length_data_size; i++) {
		obstacleData[i].x = urgData[i].x;
		obstacleData[i].y = -urgData[i].y;
	}
	////simi with reordering
	//int front_index = urg_deg2step(urg, (URG_FrontDeg - URG_FirstDeg) / SkipStep); //simi
	////for(int i=length_data_size-1;i>=0;i--){
	//for (int i = front_index; i >=0; i--)
	//{
	//	//obstacleData[length_data_size-1-i].x = urgData[i].x;	
	//	//obstacleData[length_data_size-1-i].y = -urgData[i].y;
	//	//simi no change in order, only rev sign.
	//	obstacleData[-i+front_index].x = urgData[i].x;
	//	obstacleData[-i+front_index].y= -urgData[i].y;
	//}
	////simi old way of going back->center like rotation
	//for (int i = front_index+1; i < length_data_size; i++) {
	////	obstacleData[i].x = urgData[length_data_size-i+front_index+1].x;
	////	obstacleData[i].y = -urgData[length_data_size - i + front_index + 1].y;
	//	obstacleData[i].x = urgData[i].x;
	//	obstacleData[i].y = -urgData[i].y;
	//}
	
	//for (int i = 0; i<length_data_size; i++) {
	//	printf("mod %d %lf %lf \n", i, obstacleData[i].x, obstacleData[i].y); //print
	//}
}


/********************************************************
�֐����FDetectLine_SAM
�����@�FSplit and Merge�@���g�p
�����@�Fstruct Obstacle_data *obstacleData
	�@�@int data_size
�o�́@�F


********************************************************/
//�ȉ�DetectLine_SAM�ɕK�v�Ȋ֐��Q
void DetectLine_SAM(struct Obstacle_data *obstacleData, int data_size){
	int FINISH_POINT = 0;
	int START_POINT = 0;
	double max=0;
	int max_num=0;
	int point_num=2;
	int prePoint_num;
	int data_num=2*point_num-1;
	int i = 0,j=0;
	//�K���ɔz��20�m�ۂ��Ă���@�G���[���o��Ȃ�Ώ����K�v���� Appropriate array 20 has been secured
	struct Line_data line[100] = { 0 }, new_line[100] = { 0 };

	//current change
	//DetectObject(obstacleData, data_size, &FINISH_POINT); // End the search if the distance between adjacent points is greater than or equal to OBS_THRESHOLE for the second time i.e. end of step
	//simi find start and end point before SAM
	for (i = data_size - 1; i >= 1; i--)
	{
		if (abs(obstacleData[i].x) < 1.3 ) //check &&  obstacleData[i].y < 0.1
		{
			FINISH_POINT = i;
			break;
		}

	}
	for (i = 1; i <data_size - 1; i++)
	{
		if (abs(obstacleData[i].y- obstacleData[i-1].y)>0.1)
		{
			START_POINT = i-1;
			break;
		}
		//printf("slope  %d %lf %lf %lf\n", i, obstacleData[i].x, obstacleData[i].y,(obstacleData[i + 1].y - obstacleData[i].y) / (obstacleData[i + 1].x - obstacleData[i].x)); //printcur
	}
	//printf("%d %lf %lf %d %lf %lf\n", START_POINT, obstacleData[START_POINT].x, obstacleData[START_POINT].y, FINISH_POINT, obstacleData[FINISH_POINT].x, obstacleData[FINISH_POINT].y);
	InitDetectCornerPoint(obstacleData, line, &point_num, START_POINT, FINISH_POINT); //added start point // Connect the start point and end point with a straight line and calculate the distance between the straight line and each point
	// If the maximum distance is greater than or equal to the threshold, that point is taken as a corner
	//point_num=3 if threshold crossed else 2
	//�T���̂��߂̃��[�v�X�^�[�g  Loop start for search
	for(int n=0;n<5;n++){//theoretical max of data_num is 129 at n=5
	prePoint_num = point_num;
	data_num = 2*point_num-1;
	
	
	UpdataValue(line, new_line, data_num); //create new array and fill in 0 and even indexes


	DetectCornerPoint(obstacleData, line, new_line, &point_num, data_num); //if the maximum difference between the line segment obtained in the previous search and the points in the interval is greater than or equal to the threshold, that point is taken as a corner
	
	
	//�l�̍X�V Update value
	for(j=0; j<data_num; j++){
		line[j].data_num = new_line[j].data_num;
		line[j].x = new_line[j].x;
		line[j].y = new_line[j].y;

		if(j<data_num-1){
		line[j].grad = new_line[j].grad;
		line[j].intr = new_line[j].intr;
		
		}
		
	}
	//printf("%d %d %d %d\n", n, point_num,prePoint_num, data_num); //simi
	if(point_num == prePoint_num) break;

	}
	//�T���I�� End search
	
	FittingLine(obstacleData, line, point_num); //pickup //Line fitting by least squares method
	//printf("%d \n", data_num); //simi
	for(i=0;i<data_num;i++){
	obstacleData[i].x = line[i].x;
	obstacleData[i].y = line[i].y;
	obstacleData[i].size = point_num;
	//printf("line[%d]: %lf  %lf\n",line[i].data_num, line[i].x,line[i].y); //print
	}
	for(i=0;i<data_num-1;i++){
		if(line[i+1].data_num != line[i].data_num){
			//printf("line[%d]: %lf  %lf\n",line[i].data_num, line[i].grad,line[i].intr);
			//printf("%lf,  %lf\n", line[i].x, line[i].y); //print
		}
	}
	//printf("%d \n", point_num); //current
	BumpRecognition(obstacleData, point_num);
	
}

//�אڂ���_�̋�����臒l�ȏ�̏ꍇ�T�����I������iOBS_THRESHOLE=0.1m�j
void DetectObject(struct Obstacle_data *obstacleData, int data_size, int *FINISH_POINT){
	double distance=0,a,b;
	int flag = 0;

	for(int i=0;i<data_size;i++){
	a = (obstacleData[i + 1].x - obstacleData[i].x)*(obstacleData[i + 1].x - obstacleData[i].x);
	b = (obstacleData[i + 1].y - obstacleData[i].y)*(obstacleData[i+1].y-obstacleData[i].y);
	distance = sqrt(a + b);
	//if(i>=10 && i<=16){
	//printf("%d %lf %lf %lf \n", i, distance,obstacleData[i].x, obstacleData[i].y);//simi //print
	//}
	if(distance > OBS_THRESHOLE){
		if (flag == 0) {
			//printf("%d %d \n",i,data_size);
			flag = 1;//simi
			continue;
		}
		else {
			//printf("end %d %d \n", i, data_size);
			*FINISH_POINT = i;
			break;
		}
	/*for (int i = 25; i<data_size; i++) {
		a = obstacleData[i + 1].y;
		b = obstacleData[i].y;
		if (a != 0 && abs(a)>(1 + OBS_THRESHOLE)*abs(b)) {
			//printf("%d %d %d \n", i, abs(a),abs(b)); //simi
			*FINISH_POINT = i - 1; //simi i
			break;
		*/
		}
	}
}

//�n�_�ƏI�_�𒼐��Ō��сC���̒����Ɗe�_�Ƃ̋������v�Z
//�ő勗����臒l�ȏ�̂Ƃ����̓_���R�[�i�[�Ƃ���
// Connect the start point and end point with a straight line and calculate the distance between the straight line and each point
// If the maximum distance is greater than or equal to the threshold, that point is taken as a corner
void InitDetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, int *point_num, const int START_POINT, const int FINISH_POINT){

	double error,max=0;
	int max_num=0;
	int data_num=2*(*point_num)-1;
	
	//�����̒���
	line[0].data_num = START_POINT;	line[data_num-1].data_num = FINISH_POINT; //simi START_POINT prev 0

	line[0].x = obstacleData[START_POINT].x;	line[0].y = obstacleData[START_POINT].y; //simi START_POINT prev 0
	line[data_num-1].x = obstacleData[FINISH_POINT].x;	line[data_num-1].y = obstacleData[FINISH_POINT].y;

	line[0].grad = (line[data_num-1].y-line[0].y)/(line[data_num-1].x-line[0].x);
	line[0].intr = (line[data_num-1].x*line[0].y-line[0].x*line[data_num-1].y)/(line[data_num-1].x-line[0].x);

	//�����̒T��
	for(int i = line[0].data_num; i < line[data_num-1].data_num; i++){
		error = abs(line[0].grad*obstacleData[i].x - obstacleData[i].y + line[0].intr)/sqrt(line[0].grad*line[0].grad+1);
		//printf("error[%d]:%lf\n",i,error);
		if(max<error){
			max_num = i;
			max = error;
		}
	}
	if(max>SPLIT_THRESHOLD){
		line[1].data_num = max_num;
		line[1].x = obstacleData[max_num].x;	line[1].y = obstacleData[max_num].y;
		*point_num+=1;
		
	}
	for(int i=0;i<data_num-1;i++){
		line[i].grad = (line[i+1].y-line[i].y)/(line[i+1].x-line[i].x);
		line[i].intr = (line[i+1].x*line[i].y-line[i].x*line[i+1].y)/(line[i+1].x-line[i].x);
		//printf("%d \n", data_num);
		//printf("%lf  %lf\n",line[i].grad,line[i].intr);
	}
}

//�l�̍X�V // Update value
void UpdataValue(struct Line_data *line, struct Line_data *new_line, const int data_num){

	for(int i=0; i<data_num; i+=2){
		new_line[i].data_num = line[i/2].data_num;
		new_line[i].x        = line[i/2].x;
		new_line[i].y        = line[i/2].y;
		//odd indexes of new_line empty (1,3,...)
		if(i<data_num-1){
		new_line[i].grad = line[i/2].grad;
		new_line[i].intr = line[i/2].intr;
		//printf("%lf  %lf\n",new_line[i].grad,new_line[i].intr);
		}
	}
}

//�ȑO�̒T���ŋ��߂������Ƃ��̋�ԓ��̓_�Ƃ̍��̍ő�l�̒T��
//�ő勗����臒l�ȏ�̂Ƃ����̓_���R�[�i�[�Ƃ���
// Search for the maximum difference between the line segment obtained in the previous search and the points in the interval
// If the maximum distance is greater than or equal to the threshold, that point is taken as a corner
void DetectCornerPoint(struct Obstacle_data *obstacleData, struct Line_data *line, struct Line_data *new_line, int *point_num, const int data_num){
	
	double error,max=0;
	int max_num=0;


	//�����̒T�� // Line search
	for(int i=0; i<data_num; i+=2){
		max = 0;
		

		//�ȑO�̒T���ŋ��߂������Ƃ��̋�ԓ��̓_�Ƃ̍��̍ő�l�̒T�� Search for the maximum difference between a line segment obtained in a previous search and a point in that interval
		
		for(int j = new_line[i].data_num; j < new_line[i+2].data_num; j++){
			if(i==data_num-1)	break;
			error = abs(new_line[i].grad*obstacleData[j].x - obstacleData[j].y + new_line[i].intr)/sqrt(new_line[i].grad*new_line[i].grad+1);
			//printf("error[%d]:%lf\n",j,error); 
			if(max<error){
				max_num = j;
				max = error;
			}
		}
		
	
	//�T�������ő�l��臒l���z���Ă��邩(�z���Ă����炻�̓_���R�[�i�[�Ƃ���) Whether the searched maximum value exceeds the threshold (if it exceeds, the point is taken as a corner)
	if(max>SPLIT_THRESHOLD){
		new_line[i+1].data_num = max_num;
		new_line[i+1].x = obstacleData[max_num].x;	new_line[i+1].y = obstacleData[max_num].y;
		*point_num+=1;
	//�z���Ă��Ȃ��Ƃ��͈�O�̓_�̒l����(��ō폜����) // If not, substitute the value of the previous point (delete later)
	}else{
		new_line[i+1].data_num = new_line[i].data_num;
		new_line[i+1].x = new_line[i].x;	new_line[i+1].y = new_line[i].y;
	}
	}
	//�R�[�i�[�ł���ƌ��o���ꂽ�_��p���Ē��������߂� Find a straight line using points detected to be corners
	for(int i=0;i<data_num-1;i++){
		if(new_line[i+1].data_num != new_line[i].data_num){
			new_line[i].grad = (new_line[i+1].y-new_line[i].y)/(new_line[i+1].x-new_line[i].x);
			new_line[i].intr = (new_line[i+1].x*new_line[i].y-new_line[i].x*new_line[i+1].y)/(new_line[i+1].x-new_line[i].x);			
		}else{
			new_line[i].grad = (new_line[i+2].y-new_line[i].y)/(new_line[i+2].x-new_line[i].x);
			new_line[i].intr = (new_line[i+2].x*new_line[i].y-new_line[i].x*new_line[i+2].y)/(new_line[i+2].x-new_line[i].x);	
		}
		
	}

	new_line[0].flag = true;
	for(int i=1;i<data_num;i++){
		if(new_line[i].data_num != new_line[i-1].data_num){
			new_line[i].flag = true;
		}else new_line[i].flag = false;
	}
	

	//�K�v�̂Ȃ��z��̍폜 // Delete unnecessary arrays
	for(int i=1; i<data_num; i++){
		if(new_line[i].flag == false){
			for(int j=i; j<data_num; j++){
				new_line[j].flag = new_line[j+1].flag;
				new_line[j].data_num = new_line[j+1].data_num;
				new_line[j].x = new_line[j+1].x;
				new_line[j].y = new_line[j+1].y;
				new_line[j].grad = new_line[j+1].grad;
				new_line[j].intr = new_line[j+1].intr;
			}
		}
	}
	
}

//�ŏ����@�ɂ�钼���̃t�B�b�e�B���O // Line fitting by least squares method
void FittingLine(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num) {
	int num_ave;

	//���v�C���ρC���U�C�����U�C�X���C�ؕ� Sum, mean, variance, covariance, slope, intercept
	double sumX, sumY, aveX, aveY, var, cov, a, b;

	for (int i = 0; i < point_num - 1; i++) {
		num_ave = line[i + 1].data_num - line[i].data_num + 1;

		sumX = sumY = var = cov = 0;


		for (int j = line[i].data_num; j < line[i + 1].data_num + 1; j++) {
			sumX += obstacleData[j].x;	sumY += obstacleData[j].y;
		}
		aveX = sumX / num_ave;	aveY = sumY / num_ave;
		for (int j = line[i].data_num; j < line[i + 1].data_num + 1; j++) {
			var += (obstacleData[j].x - aveX)*(obstacleData[j].x - aveX);
			cov += (obstacleData[j].x - aveX)*(obstacleData[j].y - aveY);
		}
		var /= (num_ave - 1);	cov /= num_ave;

		a = cov / var;	b = aveY - a * aveX;




		obstacleData[i].grad = a;
		obstacleData[i].intr = b;
		//printf("obs: %lf  %lf\n",obstacleData[i].grad,obstacleData[i].intr);
	}
	CalculateIntersection(obstacleData, line, point_num);
	//for (int i = 0; i < point_num; i++) {
	//	printf("obs:%d %lf  %lf\n", i, obstacleData[i].bumpX, obstacleData[i].bumpY); //print
	//}
}

//��_�̌v�Z Intersection calculation
void CalculateIntersection(struct Obstacle_data *obstacleData, struct Line_data *line, int point_num) {

	//�n�_
	obstacleData[0].bumpX = line[0].x;	obstacleData[0].bumpY = obstacleData[0].grad*line[0].x + obstacleData[0].intr;
	//��_
	for (int i = 1; i < point_num - 1; i++) {
		obstacleData[i].bumpX = -(obstacleData[i - 1].intr - obstacleData[i].intr) / (obstacleData[i - 1].grad - obstacleData[i].grad);
		obstacleData[i].bumpY = (obstacleData[i - 1].grad*obstacleData[i].intr - obstacleData[i].grad*obstacleData[i - 1].intr) / (obstacleData[i - 1].grad - obstacleData[i].grad);
	}
	//�I�_
	obstacleData[point_num - 1].bumpX = line[point_num - 1].x;	obstacleData[point_num - 1].bumpY = obstacleData[point_num - 2].grad*line[point_num - 1].x + obstacleData[point_num - 2].intr;
	for (int i = 0; i < point_num; i++) { //print
		//printf("%lf, %lf\n", obstacleData[i].bumpX, obstacleData[i].bumpY); //print
	}
}

//�����_�̌��o�i��Q���̍����C���s���C��Q���܂ł̋����̌v�Z�p�j Feature point detection (for calculating obstacle height, depth, distance to obstacle)
void BumpRecognition(struct Obstacle_data *obstacleData, int point_num) {
	int flag = 0;
	int temp = 0;
	//double pos[25][2];
	//int max = 0;
	//int max_num = 0;
	int start = 0, fin = 0;
	////std::sort(obstacleData->bumpX, obstacleData->bumpX+point_num);
	//if (obstacleData[0].bumpX < -0.1) {
	//	flag = 2;
	//	start = 0;
	//	fin = 0;
	//}
	/*else if (obstacleData[0].bumpX < 0.75 && obstacleData[0].bumpY > 0.15) {
		start = 0;
		flag = 1;
	}*/
	for (int i = 0; i < point_num - 1; i++) {
		//pos[i][0] = obstacleData[i].bumpX;
		//pos[i][1] = obstacleData[i].bumpY;
		if (flag == 0 && i != 0 && obstacleData[i].bumpY < -0.1 && obstacleData[i].bumpX < 0 && obstacleData[i].bumpX > -0.5) {
			fin = i - 1;
			flag = 3; //meaning desc end pt found
		}
	}
	for (int i = 0; i < point_num - 1; i++) {
		//pos[i][0] = obstacleData[i].bumpX;
		//pos[i][1] = obstacleData[i].bumpY;
		//if (flag == 0 && i!=0 && obstacleData[i].bumpY < -0.1 && obstacleData[i].bumpX < 0) {
		//	fin = i-1;
		//	flag = 3; //meaning desc end pt found
		//}
		//else 
		if (flag == 0 && obstacleData[i].bumpY > 0.11 && obstacleData[i].bumpX < 0.65 && obstacleData[i].bumpX >0 ) {
			start = i;
			flag = 1; //meaning  asc start found
		}
		else if (flag == 0 && obstacleData[i].bumpX < -0.2 && abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY) >0.12) {
			start = i;
			flag = 4; //meaning sitting obs
		}
		else if (flag == 1 && obstacleData[i].bumpX > 0.1 && abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY) > 0.12)
		{
			fin = i;
			//printf("f %d \n", fin);
			flag = 2; //meaning asc end found
		}
		
		printf("%d %lf, %lf, %lf %d\n", i, obstacleData[i].bumpX, obstacleData[i].bumpY, abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY), flag); //printcur
	}
	//std::sort(pos, pos + point_num);
	//for (int i = 0; i < point_num; i++) {
	//	printf("%d %lf, %lf\n", i, pos[i][0], pos[i][1]);
	//}
	//printf("%d %d\n", start, fin);
	if (flag==1 && obstacleData[start].bumpX > 0 && obstacleData[fin].bumpX > 0 && obstacleData[start].bumpX < obstacleData[fin].bumpX)
	{
		temp = fin;
		fin = start;
		start = temp;
		flag = 2;
	}
	//if descending

	if (flag == 3) {
		
		for (int i = 0; i < fin; i++) {
			printf("%d", i);
			if (obstacleData[i].bumpX < 0.3 && abs(obstacleData[i].bumpY - obstacleData[i + 1].bumpY)>0.1) {
				start = i+1;
				
				flag = 5;
				break;
			}
				
		}
	} 
	if (flag == 4) {//for sitting
		for (int i = start+1; i < point_num - 1; i++) { //-1?
			if (obstacleData[i].bumpX >-0.7 && obstacleData[i].bumpY >0.3) {
				fin = i;
				flag = 6;
				//break;
			}

		}
	}
	printf("%d %d %d\n", start, fin,flag);
	//printcur
   //obstacleData[0].bump_p = max_num;
   //obstacleData[0].height = obstacleData[max_num].bumpY;

   //obstacleData[0].distance = obstacleData[max_num].bumpX;
	if (flag==5) { //desc obstacle behind (obstacleData[start].bumpX < 0
		obstacleData[0].bump_p = start;
		obstacleData[0].height = -obstacleData[fin].bumpY+ obstacleData[fin+1].bumpY;
		obstacleData[0].distance = obstacleData[start].bumpX;
		obstacleData[0].depth = obstacleData[fin].bumpX - obstacleData[start].bumpX;
		obstacleData[0].depth = -0.27;
		//obstacleData[0].depth = obstacleData[max_num + 1].bumpX - obstacleData[max_num].bumpX; //should be negative

	}
	else if (flag == 2) { //obstacle in front
		obstacleData[0].bump_p = start;
		obstacleData[0].height = max(obstacleData[start].bumpY, obstacleData[fin].bumpY) - obstacleData[fin + 1].bumpY; //obstacleData[fin].bumpY  - obstacleData[fin + 1].bumpY;; //fin-ground next to fin
		obstacleData[0].distance = obstacleData[fin].bumpX;
		if ((obstacleData[fin].bumpX - obstacleData[fin + 1].bumpX) < 0.05){ //the point after the end of the bump is near the bump
			obstacleData[0].depth = obstacleData[start].bumpX - min(obstacleData[fin + 1].bumpX, obstacleData[fin].bumpX);
		}
		else {//if not then just calc diff b/w start and fin X
			obstacleData[0].depth = obstacleData[start].bumpX - obstacleData[fin].bumpX;
		}
		//obstacleData[0].depth = obstacleData[max_num - 1].bumpX - obstacleData[max_num].bumpX;
	}
	else if (flag == 6) {
		obstacleData[0].bump_p = start;
		obstacleData[0].height = obstacleData[fin].bumpY;
		obstacleData[0].distance = obstacleData[start].bumpX;
		obstacleData[0].depth = obstacleData[fin].bumpX - obstacleData[start].bumpX; 
		//if (abs(obstacleData[0].depth) < 0.1)
		obstacleData[0].depth = -0.45;
	}
	printf("%lf %lf %lf\n", obstacleData[0].height, obstacleData[0].distance, obstacleData[0].depth); //printcur
	
}