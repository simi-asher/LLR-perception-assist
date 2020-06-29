/*******************************************************************************
	

		認知アシスト＋パワーアシストを行うプログラム
		
		製作者　hora
	　　製作日　2016/11/09
		更新日　2016/11/09

		※環境構築　構成プロパティ->VC++ディレクトリ->インクルードディレクトリ		C:\hora\include\URG
		　　　　　　構成プロパティ->VC++ディレクトリ->ライブラリディレクトリ		C:\hora\lib\URG
********************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <conio.h>
#include <math.h>

#include "mainDraw.h"
#include "DrawParam.h"
#include "urg.h"
#include "Parameter.h"
#include "power_assist.h"
#include "fuzzy.h"
#include "perception_assist.h"
#include "calculate_Torq.h"
#include "Calculate_Position.h"
#include "calculate.h"
#include "thread.h"
#include "dataSet.h"
#include "struct.h"
#include "MPU.h" //simi MPU
#include "board.h"
#include "board_IO.h"
#include "NeuralNetwork.h"


#pragma warning(disable:4996)
#define USING_PERCEPTION

#define HIP_BASE
//#define LAYER_NUM (3)

//コンソールを表示しない
//#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")

//静的ライブラリの読み込み
#pragma comment(lib,"laserLarngeFinder.lib")	//urg使用時に必要
#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"setupapi.lib")

#pragma comment(lib,"winmm.lib")


/************************************************************
					グローバル変数
************************************************************/
struct Record_data *recordData = new Record_data[MAX_STOCK];
int  learnFlag  = OFF;
bool recordFlag = true; //simi change
bool loopFlag = true;
int keyFlag;
int t,w,v;
double timeAvg;
float roll, pitch1,pitch2, yaw1,yaw2; //simi MPU
char MPU_port[128] = "\\\\.\\COM11"; //simi MPU
int MPU_init;
int MPU_cnt = 0;
int urg_flag = 0; //simi temp should be 0 with urg on//to be made 1 when object is detected i.e. URG thread is broken
int read_flag = 0; //simi neural network, flag to read weights
int judge=0; //simi to decide which kind of motion/obstacle, 1-front, 2-desc,3-sit
/************************************************************
					センサの読み取りに必要な変数
************************************************************/

struct Sensor_data sensorData;
double encData[2][3];
double angData[3][2][3];	//関節角度のデータ[POS, VEL, ACC][R, L][H, K, A]

double forceData[2][3][3];
double force_cal[2][3][2];

/************************************************************
					トルク計算に必要な変数
************************************************************/
struct torq_data torqData;

/************************************************************
					姿勢計算に必要な変数
************************************************************/
struct leg_state Leg;
struct body_posi jointPos;

/************************************************************
					数値計算に必要な変数--
************************************************************/
#ifndef HIP_BASE
struct jacob_hip Jh;
struct jacob_body Jb;
struct jacob_data jacobData;//simi Jh and Jb for sitsquat
#endif // HIP_BASE

#ifdef HIP_BASE
struct jacob_body jacobData;
struct jacob_hip Jh;
struct jacob_body Jb;
#endif // HIP_BASE

struct transfer_data T;
struct dynamic_data Dynamics;

/************************************************************
					URG計測に必要な変数
************************************************************/
// URGのポート等の設定
const char connect_device[] = "COM12";
const long connect_baudrate = 115200;

urg_t urg;				//urgの生データなど(<urg_sensor.h>で定義)
long *length_data;		//
int length_data_size;
int urgTime;
//double inputs; //neural network

struct URG_data *urgData;
struct URG_data *rawData;

double bump_distance;
double bump_height;


int Bump_num;
struct Obstacle_data *bumpData;

/************************************************************
					認知アシストに必要な変数
************************************************************/
struct Per_data perData;

/************************************************************
					パワーアシストに必要な変数
************************************************************/
_weight weight;					//ファジィニューロ調整器のウェイト（構造体）
double weight_p[3][EMG_CH];		// weight matrix for RMS values

/************************************************************
					Neural Network Variables
************************************************************/
//change
int NEURON_NUM[3][LAYER_NUM] = { {20,20,10,10,1},{20,15,10,10,1}, {15,15,10,10,1}};		//各layerのニューロン数　[0]は入力層、[LAYER_NUM-1]は出力層（ただしバイアスは含まない）
int FUNCTION[LAYER_NUM - 1] = { relu,relu,relu,sigmoid };         //各層の活性化関数   2層～出力層
//end change
double *inputs;
double *k_inputs;
double **inter_outputs; //simi new var to hold intermediate outputs
double outputs[NN_OUTPUTS]; //simi new var to hold outputs //change NN_OUTPUTS
_weights weights[LAYER_NUM - 1];						//各層のウエイト値
double ***layer;
/************************************************************
					マルチスレッドに必要な変数
************************************************************/
CRITICAL_SECTION cs;
HANDLE URG_Handle;
HANDLE Draw_Handle;
HANDLE MPU; //simi MPU
/************************************************************
					プロトタイプの宣言
************************************************************/
void ChangeSensorData();
void ChangeRecordData(struct Record_data *recordData);
void ChangeDrawData( data_struct *data );
void mainKeyboard();
void Read_Weight(_weights* weights, struct Obstacle_data* obstacleData, int &judge);
void Make_Inputs(struct EMG_data &emgData, struct body_posi &jointPos, struct Per_data &perData, float &Yaw1, float &Roll, float &Pitch2, float &Yaw2, double* inputs, int &judge);//float &Pitch1, 
void Judge(struct Obstacle_data* obstacleData, int* judge);

//マルチスレッド用呼び出し関数
unsigned __stdcall URG_Thread(void *p); //simi_urg
unsigned __stdcall MPU_Thread(void *p); //simi mpu
unsigned __stdcall Draw_Thread(void *p);
FILE *fp;

int main() {

	BOARD board; //added
	board.SetAll();	//added

	//変数：barParam，data		(mainDraw.cppのグローバル変数として宣言)
	extern tweakParam  barParam;
	extern data_struct dataStr;

	int per_count = 0;
	//double Error_Force;

	//ファジィニューロ調整器用のウエイトパラメータの読み込み
	Read_Weight_Parameters(&weight);
	InitThread(&cs, &MPU, MPU_Thread); //simi MPU
	Calibrate_Sensor(&board, force_cal);

#ifdef USING_PERCEPTION
	//URGの初期設定
	Init_URG(&urg, connect_baudrate, connect_device); //simi

	//動的領域確保
	length_data = new long[urg_max_data_size(&urg)];
	urgData = new URG_data[urg_max_data_size(&urg)];
	bumpData = new Obstacle_data[urg_max_data_size(&urg)];
	dataStr.urg = new drawURG_struct[urg_max_data_size(&urg)];

	
	//Read_Weight(weights);
	//マルチスレッドの初期化
	InitThread(&cs, &URG_Handle, URG_Thread); //simi_URG
#endif
	
	//マルチスレッドの初期化
	
	
	InitThread(&cs, &Draw_Handle, Draw_Thread);
	printf("***** Start Main Task *****\n");

	//***********   メインタスクループ	*******************
	while (loopFlag) {

		//トルクの初期化
		Initialize_Torq(&torqData);
		//simi_URG when multithread removed
		//Measure_URG(&urg, length_data, &length_data_size, &urgTime);
		//Calculate_URG(&urg, length_data, &length_data_size, &urgTime, urgData, bumpData);
		//Initialize_SerialPort();
		/*if (perData.distance < 0)	perData.distance = 0;
		if (perData.height < 0)	perData.height = 0;
		if (perData.depth < 0)	    perData.depth = 0;*/ //simi


		/***クリティカルセクション開始***/
		EnterCriticalSection(&cs); //CCC
		//データの読み込み
		//RA and RH reading swapped in Read_Sensor
		Read_Sensor(t, &board, force_cal, &sensorData,&yaw1); //reads 8 channels of EMG from left leg and encoder data. //simi MPU added Yaw1
		//筋電のRMS値の計算
		Calculate_RMS(t, &sensorData.emg); //temp
		Calculate_RMS_L(t, &sensorData.emg); //temp
		//各種センサデータを違う変数名に変更

		//if (MPU_cnt==0 ||MPU_cnt%5==0)

		LeaveCriticalSection(&cs);//CCC //simi uncommented
		/***クリティカルセクション終了***/

		//encData[R][H] = barParam.opeParam.angData[R][H]*PI/180;
		//encData[R][K] = barParam.opeParam.angData[R][K]*PI/180;
		//encData[R][A] = barParam.opeParam.angData[R][A]*PI/180;
		//encData[L][H] = 0;
		//encData[L][K] = 0;
		//encData[L][A] = 0;
		//if ((t % (SAMPLING_FREQ / 100)) == 1) { //100Hz //simi
			//printf("calc");
		ChangeSensorData();
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 3; j++)
				angData[POS][i][j] = encData[i][j];

		//関節の速度，加速度の計算
		Calculate_Joint_VelAcc(t, angData);
		//}
		Leg.state = SwingR; //simi previously SwingR //imp
		//姿勢の計算
		EnterCriticalSection(&cs);
		Calculate_Position(encData, &Leg, &jointPos, &yaw1, &roll, &pitch2, &yaw2); //simi removed &pitch1,
		LeaveCriticalSection(&cs);
		//printf("%lf %lf %d\n", sin(-yaw * PI / 180), sin(-pitch * PI / 180), MPU_cnt);
		//MPU_cnt++; //simi MPU
		//速度，加速度，COG，ZMPの計算
		Calculate_ZMP_COG(t, &jointPos);

		
		//冗長性利用 Use of redundancy
		/*double targetAcc[3] = { 0.0, 10.0, 0.0 };
		double redundancyAcc[3];
		double torq_dyna[3];
		Calculate_UsingRedundancy(angData, targetAcc, &jacobData, redundancyAcc);
		Calculate_Dynamics(angData, redundancyAcc, &T, &Dynamics, torqData.pow[R]);*/
		//printf("%lf %lf %lf\n", torqData.pow[R][H], torqData.pow[R][K], torqData.pow[R][Z]); 

		//Calculate_Perception_Assist(bumpData, &jointPos, &Leg, &perData);
		
		if (read_flag == 0){
			Judge(bumpData, &judge);
		}
		if (read_flag == 1) {
			//neural network variable declaration and read weights
			inputs = new double[NN_INPUTS]();
			k_inputs = new double[NEURON_NUM[judge-1][0]];
			inter_outputs = new double* [LAYER_NUM - 2]; //simi neural network, Int_op=[HL_num][HL_neurons]
			for (int i = 1; i < LAYER_NUM - 1; i++) {
				inter_outputs[i - 1] = new double[NEURON_NUM[judge-1][i]];	//default init of asc bc largest
			}
			for (int i = 1; i < LAYER_NUM - 1; i++) {
				for (int j = 0; j < NEURON_NUM[judge-1][i]; j++) {
					inter_outputs[i - 1][j] = 0;			//default init of asc bc largest
				}
			}
			for (int i = 0; i < LAYER_NUM - 1; i++) {
				weights[i].layer = new double* [NEURON_NUM[judge-1][i]];
				//weight[i].delta = new double*[ NEURON_NUM[i] ];
				//weight[i].memo_layer = new double*[ NEURON_NUM[i] ];
				weights[i].bias = new double[NEURON_NUM[judge-1][i + 1]];
				//weight[i].delta_bias = new double[NEURON_NUM[i+1]];
				//weight[i].memo_bias = new double[NEURON_NUM[i+1]];
				for (int j = 0; j < NEURON_NUM[judge-1][i]; j++) {
					weights[i].layer[j] = new double[NEURON_NUM[judge-1][i + 1]];
					//weight[i].delta[j] = new double[ NEURON_NUM[i+1] ];
					//weight[i].memo_layer[j] = new double[ NEURON_NUM[i+1] ];
				}

			}
			read_flag = 2;
		}
		if (read_flag == 2) {
			Read_Weight(weights, bumpData, judge);//judge already determined
		}
		
		//neural network from here
		Make_Inputs(sensorData.emg, jointPos, perData, yaw1, roll, pitch2, yaw2, inputs, judge); //simi removed  pitch1,
		//change
		if (abs(k_inputs[14]) < 1) {//to make sure initialized?
			//Note:Test_NN is called Layers-1 times
			Test_NeuralNet(k_inputs, weights, 0, NEURON_NUM[judge-1][0], NEURON_NUM[judge - 1][1], inter_outputs[0], FUNCTION[0]);
			for (int i = 1; i < LAYER_NUM - 2; i++) {
				Test_NeuralNet(inter_outputs[i - 1], weights, i, NEURON_NUM[judge - 1][i], NEURON_NUM[judge - 1][i + 1], inter_outputs[i], FUNCTION[i]); //between HLs
			}
			Test_NeuralNet(inter_outputs[LAYER_NUM - 3], weights, LAYER_NUM - 2, NEURON_NUM[judge - 1][LAYER_NUM - 2], NEURON_NUM[judge - 1][LAYER_NUM - 1], outputs, FUNCTION[LAYER_NUM - 2]); //2nd layer i.e 1st HL to output 
			
			printf("Output: %lf\n", outputs[0]);
			
		}

#ifdef USING_PERCEPTION
		if(judge!=0){ //simi if unsafe outputs[0]>0.5 && 
		//ヤコビ行列，擬似逆行列の計算 Compute Jacobi matrix and pseudo inverse matrix
		if(judge==1 || judge==2){
			#ifndef HIP_BASE
				Calculate_JacobMatrix(angData, &Leg, &jacobData); //uncomment if not using power assist
			#endif // !HIP_BASE
			#ifdef HIP_BASE
				Calculate_JacobMatrix(angData, &Leg, &jacobData); //uncomment if not using power assist
			#endif // !HIP_BASE
			
		}
		else if(judge==3){//sitting
			
				Calculate_JacobMatrix(angData, &Leg, &Jh, &Jb); //uncomment if not using power assist //function overload not implemented yet
			
		}
			
		/***クリティカルセクション開始***/
			EnterCriticalSection(&cs); //sss //simi prev uncommented
			if (t == 1 * per_count) {
				//認知アシストで発生させる力の計算
				//FromGroundToOrigin()内で障害物の高さ，距離を一定にしている．
				Calculate_Perception_Assist(bumpData, &jointPos, &Leg, &perData, judge); //simi
				per_count++;
				//printf("called t %i,per_count %i\n", t,per_count); //simi
			}
			LeaveCriticalSection(&cs); //sss //simi
			//for (int i = 0; i < 2; i++) {
			//	printf("Vir wall force:%d %lf\t", i,perData.virWall.force[i]);//simi print cur
			//}
			//printf("\n");
			if (judge == 1 || judge == 2) {
				#ifndef HIP_BASE
					ConvertForceToTorque(&Leg, &jacobData, perData.virWall.force, torqData.per); //simi uncommented  //uncomment if not using power assist
				#endif // !HIP_BASE
				#ifdef HIP_BASE
					ConvertForceToTorque(&Leg, &jacobData, perData.virWall.force, torqData.per);
				#endif // !HIP_BASE
			}
			else if (judge == 3) {//sitting
				
					ConvertForceToTorque(&Leg, &Jh, &Jb, perData.virWall.force, torqData.per);
				
			}
			if (judge != 3) {
				Calculate_Perception_Compensation(&Leg, &torqData, &jointPos, angData[POS]);
			}
			else {
				torqData.per_comp = 0;
			}
			//重力保障
			//Calculate_Gravity_Compensation(Leg.state, &torqData, encData);

			/***クリティカルセクション終了***/

			////perData.virWall.force[X] = 0;
			////perData.virWall.force[Y] = 50;
			//
			//ConvertForceToTorque(&Leg, &jacobData, perData.virWall.force, torqData.per); //simi commented

			//if(perData.perFlag == true){
			//double force[3]={0,0,0};
			//EnergyMinimization(encData,&Leg, &jacobData,torqData.per[R+], force);
			//////printf("[X]:%lf, [Y]:%lf, [Z]:%lf, %lf, %lf\n",force[X],force[Y],force[Z],sqrt(force[X]*force[X] + force[Y]*force[Y]), sqrt(force[X]*force[X] + force[Y]*force[Y] + force[Z]*force[Z]));
			//ConvertForceToTorque(&Leg, &jacobData, force, torqData.per);

			////for(int i=0;i<2;i++)	barParam.valParam.perForce[i] = perData.virWall.force[i];
			//for(int i=0;i<2;i++)	barParam.valParam.perForce[i] = force[i];
			//}
		}
		
#endif

		/*for(int i=0;i<3;i++){
		forceData[R][i][X] = 0;
		forceData[L][i][X] = 0;
		if(forceData[R][H][X]<0) forceData[R][H][X] = 0;
		if(forceData[L][H][X]<0) forceData[L][H][X] = 0;
	}*/

	//uncomment the following if using power assist
	//パワーアシストで発生させる力の計算(力センサ)
	//Calculate_Power_Assist(learnFlag, &weight, encData, weight_p, sensorData.emg.rms, forceData, torqData.pow);

	//double force[3] = { 0,50,0};
	//ConvertForceToTorque(&Leg, &jacobData, force, torqData.pow);
	//for(int i=0;i<2;i++)	barParam.valParam.perForce[i] = force[i]; 

	////出力トルクの計算
	Calculate_Torq(&torqData);//change currently all 0 // per comp in o/p torque
	//printf("Output Torque Data\n");
	//for (int i = 0; i < 2; i++) {
	//	for (int j = 0; j < 3; j++) {
	//		
	//		printf("%lf\t", torqData.out[i][j]);//simi print cur
	//	}
	//	printf("\n");
	//} //simi print torque
	


	////モータへの出力 //add line to change output torque only when outputs[0]>0.5 and per_flag==1
	board.motorOutput(ON, torqData.out, torqData.sw);
	//till here

	//printf("%lf		%lf\n",jointPos.Foot[R].acc[X],jointPos.Foot[R].acc[Y]);
	//printf("POS:%lf	%lf %lf	VEL:%lf	%lf %lf\n", angData[POS][R][H], angData[POS][R][K], angData[POS][R][A], angData[VEL][R][H], angData[VEL][R][K], angData[VEL][R][A]);
	//printf("%lf	 %lf  %lf\n",forceData[R][H][X] ,forceData[R][K][X] ,forceData[R][A][X]);
	//printf("%lf	%lf	%lf\n",force[X], force[Y], force[Z]);
	//printf("%lf	 %lf  %lf\n",perData.virWall.force[H] ,perData.virWall.force[K] ,perData.virWall.force[A]);
	//printf("%lf	 %lf  %lf  %lf\n",torqData.perception[R][H] ,torqData.perception[R][K] ,torqData.power[L][H] ,torqData.power[L][K]);
	//printf("%lf	 %lf  %lf  %lf\n",torqData.gra[R][H] ,torqData.gra[R][K] ,torqData.gra[L][H] ,torqData.gra[L][K]);
	//printf("%lf	 %lf  %lf  %lf\n",torqData.pow[R][H] ,torqData.pow[R][K] ,torqData.pow[L][H] ,torqData.pow[L][K]);
	//printf("%lf	 %lf  %lf  %lf\n",torqData.fin[R][H] ,torqData.fin[R][K] ,torqData.fin[R][A]);
	//printf("%lf	 %lf  %lf\n",torqData.out[R][H] ,torqData.out[R][K] ,torqData.out[R][A]);
	//printf("%lf\n",sensorData.emg.rms[R][0] );

	//for(int i=0;i<3;i++)	torqData.pow[R][i] = torqData.gra[R][i];
	
	
	
	//till here
	ChangeRecordData(recordData); //simi MPU,NN recordData struct also changed
	EnterCriticalSection(&cs); //simi
	ChangeDrawData(&dataStr); //NN output also included in dataStr
	LeaveCriticalSection(&cs); //simi
	
	mainKeyboard();

	Adjust_SamplingTime(t, &timeAvg);
	t++;

}
	//***********	メインループここまで　********************

if(learnFlag == ON){
	Write_Weight_Parameters( &weight );
}

// write data //
if (recordFlag == true) {
	printf("ファイル出力中…\n");
	OutputFILEs(w, v, recordData,urg_flag);
	printf("ファイル出力終了\n");
}

//end
printf("end\n");

board.CloseAll();

//CloseHandle(MPU); //simi MPU

#ifdef USING_PERCEPTION
	EndThread(&cs,&URG_Handle); //simi_urg
	EndThread(&cs, &MPU); //simi MPU
	EndThread(&cs,&Draw_Handle); 
	EndMeasure(&urg);
#endif


#ifdef USING_PERCEPTION
	delete[] length_data;
	delete[] urgData;
	delete[] bumpData;
	delete[] recordData;
#endif
	delete[] inputs; //neural network
	delete[] k_inputs;
	delete[] inter_outputs; //simi new var to hold intermediate outputs
	delete[] outputs; //simi new var to hold outputs //change NN_OUTPUTS
	delete[] weights;						//各層のウエイト値
	delete[] layer;
	return 0;
}



void ChangeSensorData(){

	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			encData[i][j] = sensorData.enc_data[i][j];

			//for(int k=0;k<2;k++){
				//forceData[i][j][k] = sensorData.force_data[i][j][k];
			//}
		}
	}

}


void ChangeRecordData(struct Record_data *recordData){
	int SUP = 1; //bc SwingR
	if(recordFlag == true){
			//0.1[sec]毎にデータを記録 Data is recorded every 0.1 [sec]
			//can back obstacle also be measured?
			
			if(v == ( w*SAMPLING_FREQ/SAMPLING_FREQ ) ){
			for(int i=0;i<2;i++){
				for(int j=0;j<3;j++){
					recordData[w].enc[i][j]      = encData[i][j];
					//recordData[w].force[i][j]    = forceData[i][j][X]; //since not read in read sensor function
					if (torqData.sw[i][j]>1) //so output torque is recorded with correct sign
						recordData[w].torq[i][j]     = -torqData.out[i][j];
					else
						recordData[w].torq[i][j] = torqData.out[i][j];
					//recordData[w].torq_pow[i][j] = torqData.pow[i][j];
					recordData[w].torq_per[i][j] = torqData.per[i][j];//perception assist torque
					recordData[w].torq_per_comp = torqData.per_comp;//perception assist torque compensation
				}
				recordData[w].jointPos.Hip[i][X]   = jointPos.Hip[i][X];	recordData[w].jointPos.Knee[i][X] = jointPos.Knee[i][X];	recordData[w].jointPos.Ankle[i][X] = jointPos.Ankle[i][X];	recordData[w].jointPos.Toe[i][X] = jointPos.Toe[i][X];	recordData[w].jointPos.Heel[i][X] = jointPos.Heel[i][X]; //simi 
				recordData[w].jointPos.Hip[i][Y]   = jointPos.Hip[i][Y];	recordData[w].jointPos.Knee[i][Y] = jointPos.Knee[i][Y];	recordData[w].jointPos.Ankle[i][Y] = jointPos.Ankle[i][Y];	recordData[w].jointPos.Toe[i][Y] = jointPos.Toe[i][Y];	recordData[w].jointPos.Heel[i][Y] = jointPos.Heel[i][Y];
				
				recordData[w].jointPos.Urg[SUP][X] = jointPos.Urg[SUP][X]; recordData[w].jointPos.Urg[SUP][Y] = jointPos.Urg[SUP][Y];//simi

				recordData[w].jointPos.UppThigh[i].pos[X] = jointPos.UppThigh[i].pos[X]; recordData[w].jointPos.LowThigh[i].pos[X] = jointPos.LowThigh[i].pos[X]; recordData[w].jointPos.Foot[i].pos[X] = jointPos.Foot[i].pos[X];
				recordData[w].jointPos.UppThigh[i].pos[Y] = jointPos.UppThigh[i].pos[Y]; recordData[w].jointPos.LowThigh[i].pos[Y] = jointPos.LowThigh[i].pos[Y]; recordData[w].jointPos.Foot[i].pos[Y] = jointPos.Foot[i].pos[Y];
				recordData[w].jointPos.UppThigh[i].vel[X] = jointPos.UppThigh[i].vel[X]; recordData[w].jointPos.LowThigh[i].vel[X] = jointPos.LowThigh[i].vel[X]; recordData[w].jointPos.Foot[i].vel[X] = jointPos.Foot[i].vel[X];
				recordData[w].jointPos.UppThigh[i].vel[Y] = jointPos.UppThigh[i].vel[Y]; recordData[w].jointPos.LowThigh[i].vel[Y] = jointPos.LowThigh[i].vel[Y]; recordData[w].jointPos.Foot[i].vel[Y] = jointPos.Foot[i].vel[Y];
				recordData[w].jointPos.UppThigh[i].acc[X] = jointPos.UppThigh[i].acc[X]; recordData[w].jointPos.LowThigh[i].acc[X] = jointPos.LowThigh[i].acc[X]; recordData[w].jointPos.Foot[i].acc[X] = jointPos.Foot[i].acc[X];
				recordData[w].jointPos.UppThigh[i].acc[Y] = jointPos.UppThigh[i].acc[Y]; recordData[w].jointPos.LowThigh[i].acc[Y] = jointPos.LowThigh[i].acc[Y]; recordData[w].jointPos.Foot[i].acc[Y] = jointPos.Foot[i].acc[Y];
				recordData[w].jointPos.Body.pos[i] = jointPos.Body.pos[i];
				recordData[w].jointPos.XZmp = jointPos.XZmp;

			} 
			recordData[w].bump[0] = perData.distance;   	recordData[w].bump[1] = perData.height;	        recordData[w].bump[2] = perData.depth;
			recordData[w].bump[3] = perData.virWall.start_x;	recordData[w].bump[4] = perData.virWall.start_y;	recordData[w].bump[5] = perData.virWall.end_x;
			recordData[w].bump[6] = perData.virWall.end_y;	recordData[w].bump[7] = perData.virWall.force[X];	recordData[w].bump[8] = perData.virWall.force[Y]; 
			recordData[w].roll = roll;  recordData[w].pitch2 = pitch2; recordData[w].yaw1 = yaw1; recordData[w].yaw2 = yaw2;//simi removed recordData[w].pitch1 = pitch1;
			recordData[w].per_flag = perData.perFlag;//flag to see if virtual wall breached//perception assist is on
			for (int i = 0; i<NN_INPUTS; i++) {//simi neural network
				recordData[w].inputs[i] = inputs[i];
			}
			for (int i = 0; i<NN_OUTPUTS; i++) {//simi neural network
				recordData[w].outputs[i] = outputs[i];
			}
			w++;
			}

			
			for(int i=0;i<2;i++){
				for(int j=0;j<EMG_CH;j++){
					recordData[v].emg.raw[i][j] = sensorData.emg.raw[i][j];
					recordData[v].emg.rms[i][j] = sensorData.emg.rms[i][j];
				}
			}
			v++;
			if(v==MAX_STOCK){
				printf("記憶できるメモリの上限です Memory is full\n");
				loopFlag = false; 
			}
		}

}



void ChangeDrawData( data_struct *data ){

	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			data->torq.out[i][j] = torqData.out[i][j];
			data->torq.pow[i][j] = torqData.pow[i][j];
			data->torq.per[i][j] = torqData.per[i][j];
		}
	}


#ifdef USING_PERCEPTION
	//simi temp
	data->urg[0].dataSize = length_data_size;
	for(int i=0;i<length_data_size;i++){
		data->urg[i].length[X] = urgData[i].rawX;
		data->urg[i].length[Y] = urgData[i].rawY;
		data->urg[i].smooth[X] = urgData[i].x;
		data->urg[i].smooth[Y] = urgData[i].y;
	}

	data->urg[0].dataBumpSize = bumpData[0].size;
	for(int i=0;i<data->urg[0].dataBumpSize;i++){
		data->urg[i].position[X] = bumpData[i].x;
		data->urg[i].position[Y] = bumpData[i].y;
	}

	data->bump[DISTANCE] = perData.distance;		data->bump[HEIGHT] = perData.height;		  data->bump[DEPTH] = perData.depth;
	data->bump[VIR_START_X] = perData.virWall.start_x;    data->bump[VIR_START_Y] = perData.virWall.start_y;  
	data->bump[VIR_END_X] = perData.virWall.end_x;	    	data->bump[VIR_END_Y] = perData.virWall.end_y;
#endif
	//後で削除する必要あり
	//simi data->bump[DISTANCE] = perData.distance;		data->bump[HEIGHT] = perData.height;		  data->bump[DEPTH] = perData.depth;
	
	data->drawJoint.Hip[R][X]   = jointPos.Hip[R][X];   data->drawJoint.Hip[R][Y]   = jointPos.Hip[R][Y];   data->drawJoint.Knee[R][X] = jointPos.Knee[R][X]; data->drawJoint.Knee[R][Y] = jointPos.Knee[R][Y]; 
	data->drawJoint.Ankle[R][X] = jointPos.Ankle[R][X]; data->drawJoint.Ankle[R][Y] = jointPos.Ankle[R][Y]; data->drawJoint.Toe[R][X]  = jointPos.Toe[R][X];  data->drawJoint.Toe[R][Y]  = jointPos.Toe[R][Y]; 
	data->drawJoint.Heel[R][X]  = jointPos.Heel[R][X];  data->drawJoint.Heel[R][Y]  = jointPos.Heel[R][Y];

	data->drawJoint.Hip[L][X]   = jointPos.Hip[L][X];   data->drawJoint.Hip[L][Y]   = jointPos.Hip[L][Y];   data->drawJoint.Knee[L][X] = jointPos.Knee[L][X];  data->drawJoint.Knee[L][Y] = jointPos.Knee[L][Y]; 
	data->drawJoint.Ankle[L][X] = jointPos.Ankle[L][X]; data->drawJoint.Ankle[L][Y] = jointPos.Ankle[L][Y]; data->drawJoint.Toe[L][X]  = jointPos.Toe[L][X];   data->drawJoint.Toe[L][Y]  = jointPos.Toe[L][Y]; 
	data->drawJoint.Heel[L][X]  = jointPos.Heel[L][X];  data->drawJoint.Heel[L][Y]  = jointPos.Heel[L][Y];  data->drawJoint.Body[X]    = jointPos.Body.pos[X]; data->drawJoint.Body[Y]    = jointPos.Body.pos[Y];

	for(int i=0;i<data->length;i++){ //to make position of joints diff in Z dir
		for(int j=0;j<2;j++){
			data->drawJoint.Hip[j][Z] = -0.3*j;	data->drawJoint.Knee[j][Z] = -0.3*j;	data->drawJoint.Ankle[j][Z] = -0.3*j;	data->drawJoint.Toe[j][Z] = -0.3*j; data->drawJoint.Heel[j][Z] = -0.3*j;	
		}
		data->drawJoint.Body[Z] = -0.15;
		//data->element[i].Bump[DEPTH] = 0.2;
	}

	for(int i=0;i<EMG_CH;i++)
		for(int j=0;j<RULE;j++)
			data->weight[i][j] = weight.r_d[H][i][j];


	data->time = t/SAMPLING_FREQ;
	data->freq = timeAvg;

	//simi neural network
	data->perflag = perData.perFlag;

	for (int k = 0; k<NN_OUTPUTS; k++)
		data->outputs[k]=outputs[k];

}

void mainKeyboard(){
	//ループ終了処理
	if(_kbhit()){
		keyFlag = getch();

		switch(keyFlag){
		case 'q':
			recordFlag = true; //simi previously not present
			loopFlag = false;
			break;
		case 'r':
			printf("記録開始\n");
			recordFlag = true;
			break;
		case '\n':
		case '\r':
		case 'e':
			printf("記録終了\n");
			recordFlag = true; 
			loopFlag = false;
			break;
		default:
			break;
		}
	}
}


unsigned __stdcall URG_Thread(void *p){
	
	double time=0;
	static LARGE_INTEGER freq,start_time, end_time;
	
    while(loopFlag){
	
	QueryPerformanceFrequency( &freq ); 
    QueryPerformanceCounter( &end_time );
	time = (double)(end_time.QuadPart - start_time.QuadPart)/(double)(freq.QuadPart); //what is time?
	//printf("URG %lf %lf %lf \n", time, (double)(end_time.QuadPart - start_time.QuadPart), (double)(freq.QuadPart));
	//printf("%lf %d \n", time,t); //simi
	//while(t<200){ //why if 80? t is main loop counter //simi remove t loop
	//距離の計測	
	//時間が50[msec]ほどかかっていたためクリティカルセクションの外で使用している
	//※URGの計測データに不具合などがあれば要注意
	//不具合があればCalculate_URG()のMeasure_URG()のコメントアウトを解除し、この関数をコメントアウト　ただしサンプリング周波数がおかしくなる
	/*Distance measurement
	Because it takes about 50 [msec], it is used outside the critical section
	※ Be careful if there is a problem with the URG measurement data
	Uncomment out Measure_URG () of Calculate_URG () if there is a problem, comment out this function, however the sampling frequency will be wrong*/
	Measure_URG(&urg, length_data, &length_data_size, &urgTime);

	//1ループが時間内ならビジーループで時間をつぶす If 1 loop is within given timetime, kill time in busy loop
	while( time<=0.15 ) { //why 0.15?
		
		QueryPerformanceCounter( &end_time );
		time = (double)(end_time.QuadPart - start_time.QuadPart)/(double)(freq.QuadPart);			
	} 
	//EnterCriticalSection(&cs); //simi

    //printf("スレッドに入りました。\n");

	//URGのデータの計測から物体検出の計算まで
	Calculate_URG(&urg, length_data, &length_data_size, &urgTime, urgData, bumpData); //URG data
	
	QueryPerformanceCounter( &start_time );//abs(bumpData[0].distance) >0.1 && //simi sit abs(bumpData[0].depth) > 0.2 && abs(bumpData[0].depth) < 0.5 &&
	//change at run time
	if ( abs(bumpData[0].height) > 0.11 && (bumpData[0].distance) <0) //only for asc && abs(bumpData[0].height) <0.2 
	{//&& (bumpData[0].distance) <0 for sitsquat,abs(bumpData[0].distance) > 0.05 && for all except desc		printf("done");
		urg_flag=1;
		break;
	}
    //LeaveCriticalSection(&cs); //simi
	//} //simi remove t loop
	//if(t==40)		EndMeasure(&urg); //simi
	
	}
    return 0;

}

unsigned __stdcall MPU_Thread(void *p) { //simi MPU

	double time = 0;
	static LARGE_INTEGER freq, start_time, end_time;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start_time);
	printf("MPU Init Start\n");
	MPU_init = Init_MPU(&MPU, MPU_port); //simi MPU	
	QueryPerformanceCounter(&end_time);
	time = (double)(end_time.QuadPart - start_time.QuadPart) / (double)(freq.QuadPart); //what is time?
	printf("MPU Init End\n"); //simi

	while (loopFlag) {
		QueryPerformanceFrequency(&freq);
		Read_MPU(&MPU, &yaw1, &roll, &pitch2, &yaw2);		//simi MPU removed &pitch1,
		QueryPerformanceCounter(&end_time);
		time = (double)(end_time.QuadPart - start_time.QuadPart) / (double)(freq.QuadPart); //what is time?
		//printf("MPU loop %f %f %f %f\n", roll, pitch1, pitch2, yaw);
		//printf("MPU loop %lf %lf %lf \n", time, (double)(end_time.QuadPart - start_time.QuadPart), (double)(freq.QuadPart));
	}

	return 0;
	//if (MPU_init == 1)
	//	printf("Error initializing MPU\n");
}

unsigned __stdcall Draw_Thread(void *p){
	
	draw_main();
		
    return 0;
}

/*************************************************
//	<Judge>				            	//
//	decide where obstacle is					                //
*************************************************/
void Judge(struct Obstacle_data* obstacleData, int *judge) {
	do {
		if (abs(obstacleData->distance) < 1) {
			if (obstacleData->distance > 0 && obstacleData->height > 0) {
				printf("asc\n");
				*judge = 1;
			}
			else if (obstacleData->distance > 0 && obstacleData->height < 0) {
				printf("desc\n");
				*judge = 2;
			}
			else {
				printf("sit\n");
				*judge = 3;
			}
			read_flag = 1;
		}
	} while (read_flag == 0);
}

/*************************************************
//	<Read_Weight>				            	//
//	read weight					                //
*************************************************/
void Read_Weight(_weights* weights, struct Obstacle_data *obstacleData, int &judge) {
	
	
	do {
		printf("Reading weights\n");
		if (judge==1) {
			
			FILE_READ_OPEN(fp,"final-wts/asc_5l_ne.dat" );//simi weight paths
			printf("asc\n");

		}
		else if (judge == 2) {

			FILE_READ_OPEN(fp, "final-wts/desc_5l_ne.dat");//simi weight paths
			printf("desc\n");

		}
		else if (judge == 3) {

			FILE_READ_OPEN(fp, "final-wts/sit_5l_ne.dat");//simi weight paths
			printf("sit\n");

		}
		
		//求めたウェイト値を読み取る
		//FILE_READ_OPEN(fp, "wts_no_emg.dat"); //change //simi neural network
		for (int i = 0; i < LAYER_NUM - 1; i++) {
			for (int j = 0; j < NEURON_NUM[judge-1][i + 1]; j++) {

				for (int k = 0; k < NEURON_NUM[judge - 1][i]; k++) {
					fscanf(fp, "%lf\t", &weights[i].layer[k][j]);
				}
				fscanf(fp, "%lf\t", &weights[i].bias[j]);
				fscanf(fp, "\n");
			}
			fscanf(fp, "\n");
		}


		//ウェイト値の確認
		//for (int num = 0; num < LAYER_NUM - 1; num++) {

		//	for (int j = 0; j < NEURON_NUM[judge - 1][num + 1]; j++) {
		//		printf("%lf, ", weights[num].bias[j]); //simi print cur neural network

		//		for (int i = 0; i < NEURON_NUM[judge - 1][num]; i++) {
		//			printf("%lf, ", weights[num].layer[i][j]);
		//		}

		//		printf("\n");

		//	}

		printf("Weights read\n");
		
		
		/*else {
			printf("nothing");
		}*/
		read_flag = 3;
	} while (read_flag == 2);
	
}

void Make_Inputs(struct EMG_data &emgData, struct body_posi &jointPos, struct Per_data &perData, float &Yaw1, float &Roll, float &Pitch2, float &Yaw2, double* input, int &judge) { //simi removed  float &Pitch1,
	
	//change at test time	
	
	//int cols [] = {0, 1, 2, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 20, 21, 24, 25, 28}; //best 20 asc
	//int cols [] = {8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 20, 21, 24, 25, 26, 27, 28, 31, 33}; //best 20 asc no emg
	//int cols [] = {0, 1, 5, 6, 8, 10, 12, 14, 16, 18, 20, 22, 23, 24, 26, 27, 31, 32, 33, 34}; //best 20 desc
	//int cols [] = {8, 10, 12, 14, 16, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 29, 31, 32, 33, 34}; //best 20 desc no emg
	int cols [] = {0, 2, 7, 8, 9, 10, 11, 12, 13, 21, 26, 28,  31, 33, 34}; //best 15 sit
	//int cols [] = {8, 9, 10, 11, 12, 13, 19, 21, 23, 26, 28, 29, 31, 33, 34}; //best 15 sit no emg
	//simi change at run time
	int k_best;
	if (judge==3)//sitting
		k_best = 15;
	else
		k_best = 20;
	
	for (int j = 0; j < EMG_CH; j++) {
		 input[j]=emgData.rms[R][j]; //R=0
	} //comment out for_no_emg *inputs
	int emg_end = EMG_CH; //0 when not using emg
	input[emg_end] = jointPos.Toe[R][X];
	input[emg_end + 1] = jointPos.Toe[R][Y];
	input[emg_end + 2] = jointPos.Heel[R][X];
	input[emg_end + 3] = jointPos.Heel[R][Y];
	input[emg_end + 4] = jointPos.Foot[R].pos[X];
	input[emg_end + 5] = jointPos.Foot[R].pos[Y];
	input[emg_end + 6] = jointPos.Foot[R].vel[X];
	input[emg_end + 7] = jointPos.Foot[R].vel[Y];
	input[emg_end + 8] = jointPos.Foot[R].acc[X];
	input[emg_end + 9] = jointPos.Foot[R].acc[Y];
	input[emg_end + 10] = jointPos.Hip[R][X];
	input[emg_end + 11] = jointPos.Hip[R][Y];
	input[emg_end + 12] = jointPos.UppThigh[R].vel[X];
	input[emg_end + 13] = jointPos.UppThigh[R].vel[Y];
	input[emg_end + 14] = jointPos.XZmp;
	int val = 15;
	for (int j = 0; j < 3; j++) { //H=0, K=1, A=2
		for (int i = 1; i >= 0; i--) { //R=0, L=1 
			input[emg_end + val] = encData[i][j];
			val++;
		}
	}
	input[emg_end + val] = Roll;
	input[emg_end + val + 1] = 0;// Pitch1; //simi not including 
	input[emg_end + val + 2] = Pitch2;
	input[emg_end + val + 3] = Yaw2;
	input[emg_end + val + 4] = perData.distance;
	input[emg_end + val + 5] = perData.height;
	input[emg_end + val + 6] = perData.depth;
	//printf("*inputs\n");
	for (int k = 0; k < NN_INPUTS; k++) {
		input[k] = round(input[k] * 1000.0) / 1000.0; 
		//printf("%+6.7lf\t", *inputs[k]);
	}
	for (int i = 0; i < k_best; i++) {
		k_inputs[i] = input[cols[i]];
	}
	//printf("kinputs\n");
}