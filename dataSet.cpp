/*****************************************************************************************************************************
******************************************************************************************************************************
データセットのためのヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "dataSet.h"
//#include "struct.h"
#pragma warning(disable:4996)


int OutputFILEs(int w, int v, Record_data recordData[MAX_STOCK], int urg_flag) {
	FILE *fp;
	//int flag = 0; //simi
	int k = 0;    //simi

	FILE_WRITE_OPEN(fp, "output_data/data_size.dat");
	fprintf(fp, "FREQ:%d, FREQ:%d \n", 2000, 10);
	fprintf(fp, "%d  %d", v, w);
	fclose(fp);

	if (REC_FORCE_SENSOR == ON) {
		//	力センサの値	//
		FILE_WRITE_OPEN(fp, "output_data/Sensor/force_sensor.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf %+12.6lf %+12.6lf %+12.6lf\n", recordData[i].force[R][H], recordData[i].force[R][K], recordData[i].force[R][A]
				, recordData[i].force[L][H], recordData[i].force[L][K], recordData[i].force[L][A]);
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_ENCODER == ON) {
		FILE_WRITE_OPEN(fp, "output_data/exp/encoder.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%+12.6lf	%+12.6lf %+12.6lf %+12.6lf	%+12.6lf %+12.6lf\n", recordData[i].enc[L][H], recordData[i].enc[L][K], recordData[i].enc[L][A], recordData[i].enc[R][H], recordData[i].enc[R][K], recordData[i].enc[R][A]); //simi previously R only
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_TORQ == ON) {
		FILE_WRITE_OPEN(fp, "output_data/Torq/torq.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf %+12.6lf %+12.6lf %+12.6lf\n", recordData[i].torq[R][H], recordData[i].torq[R][K], recordData[i].torq[R][A]
				, recordData[i].torq[L][H], recordData[i].torq[L][K], recordData[i].torq[L][A]);
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_TORQ == ON) {
		FILE_WRITE_OPEN(fp, "output_data/Torq/torq_power.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf %+12.6lf %+12.6lf %+12.6lf\n", recordData[i].torq_pow[R][H], recordData[i].torq_pow[R][K], recordData[i].torq_pow[R][A]
				, recordData[i].torq_pow[L][H], recordData[i].torq_pow[L][K], recordData[i].torq_pow[L][A]);
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_TORQ == ON) {
		FILE_WRITE_OPEN(fp, "output_data/Torq/torq_perception.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf\n", recordData[i].torq_per[R][H], recordData[i].torq_per[R][K], recordData[i].torq_per[R][A]);
		}
		fprintf(fp, "\n");
		fclose(fp);
	}
	/*
		if(REC_EMG == ON){
		FILE_WRITE_OPEN( fp, "output_data/exp/emg_raw1R.dat" );
		for(int i=0 ; i<v ; i++ ){
			for(int j=0;j<EMG_CH;j++){
				//printf("here1");
				fprintf( fp, "%+6.3f\t", recordData[i].emg.raw[R][j]);
			}
			fprintf( fp, "\n");
		}
		fclose( fp );
		}

		if(REC_EMG == ON){
		FILE_WRITE_OPEN( fp, "output_data/exp/emg_rmsR.dat" );
		for(int i=0 ; i<v ; i++ ){
			for(int j=0;j<EMG_CH;j++){
				//printf("here2");
				fprintf( fp, "%+6.7lf\t", recordData[i].emg.rms[R][j]);
			}
			fprintf( fp, "\n");
		}
		fclose( fp );
		}
	*/
	if (REC_EMG == ON) {
		FILE_WRITE_OPEN(fp, "output_data/exp/emg_rawR.dat");
		for (int i = 0; i < v; i++) {
			//printf("here3");
			for (int j = 0; j < EMG_CH; j++) {
				fprintf(fp, "%+6.3f\t", recordData[i].emg.raw[R][j]);
			}
			fprintf(fp, "\n");
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_EMG == ON) {
		FILE_WRITE_OPEN(fp, "output_data/exp/emg_rmsR.dat");
		for (int i = 0; i < v; i++) {
			//printf("here4");
			for (int j = 0; j < EMG_CH; j++) {
				fprintf(fp, "%+6.7lf\t", recordData[i].emg.rms[R][j]);
			}
			fprintf(fp, "\n");
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_VIR_WALL == ON) {
		FILE_WRITE_OPEN(fp, "output_data/bump_wT1.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%lf %lf ", recordData[i].jointPos.Toe[R][X], recordData[i].jointPos.Toe[R][Y]);
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq_per[R][H], recordData[i].torq_per[R][K], recordData[i].torq_per[R][A]);
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq_per[L][H], recordData[i].torq_per[L][K], recordData[i].torq_per[L][A]);
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq[R][H], recordData[i].torq[R][K], recordData[i].torq[R][A]);
			fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq[L][H], recordData[i].torq[L][K], recordData[i].torq[L][A]);
			for (int j = 0; j < 9; j++) {
				fprintf(fp, "%+12.6lf ", recordData[i].bump[j]);
			}
			fprintf(fp, "\n");
		}
		fprintf(fp, "\n");
		fclose(fp);
	}

	if (REC_POSITION == ON) {
		FILE_WRITE_OPEN(fp, "output_data/exp/JointPos.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", recordData[i].jointPos.Hip[R][X], recordData[i].jointPos.Hip[R][Y], //simi  3-10 recordData[i].jointPos.Knee[R][X], recordData[i].jointPos.Knee[R][Y], recordData[i].jointPos.Ankle[R][X], recordData[i].jointPos.Ankle[R][Y], recordData[i].jointPos.Toe[R][X], recordData[i].jointPos.Toe[R][Y], recordData[i].jointPos.Heel[R][X], recordData[i].jointPos.Heel[R][Y],
				recordData[i].jointPos.Hip[L][X], recordData[i].jointPos.Hip[L][Y], recordData[i].jointPos.Knee[L][X], recordData[i].jointPos.Knee[L][Y], recordData[i].jointPos.Ankle[L][X], recordData[i].jointPos.Ankle[L][Y], recordData[i].jointPos.Toe[L][X], recordData[i].jointPos.Toe[L][Y], recordData[i].jointPos.Heel[L][X], recordData[i].jointPos.Heel[L][Y], //12
				recordData[i].jointPos.Body.pos[X], recordData[i].jointPos.Body.pos[Y]);
		}
		fprintf(fp, "\n");
		fclose(fp);
	}
	if (REC_PVA == ON) {
		FILE_WRITE_OPEN(fp, "output_data/exp/PVA1.dat");
		for (int i = 0; i < w; i++) {
			fprintf(fp, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n",
				recordData[i].jointPos.UppThigh[L].pos[X], recordData[i].jointPos.UppThigh[L].pos[Y], recordData[i].jointPos.LowThigh[L].pos[X], recordData[i].jointPos.LowThigh[L].pos[Y], recordData[i].jointPos.Foot[L].pos[X], recordData[i].jointPos.Foot[L].pos[Y], //simi remove all R leg values
				recordData[i].jointPos.UppThigh[L].vel[X], recordData[i].jointPos.UppThigh[L].vel[Y], recordData[i].jointPos.LowThigh[L].vel[X], recordData[i].jointPos.LowThigh[L].vel[Y], recordData[i].jointPos.Foot[L].vel[X], recordData[i].jointPos.Foot[L].vel[Y],
				recordData[i].jointPos.UppThigh[L].acc[X], recordData[i].jointPos.UppThigh[L].acc[Y], recordData[i].jointPos.LowThigh[L].acc[X], recordData[i].jointPos.LowThigh[L].acc[Y], recordData[i].jointPos.Foot[L].acc[X], recordData[i].jointPos.Foot[L].acc[Y],
				recordData[i].jointPos.Body.pos[X], recordData[i].jointPos.Body.pos[Y]);
		}
		fprintf(fp, "\n");
		fclose(fp);
	}
	if (REC_CUSTOM == ON) { //simi new funcq
		FILE_WRITE_OPEN(fp, "output_data/jun_sit_10.dat");
		for (int i = 0; i < w; i++) {
			//printf("%d %lf %lf %lf\n", flag, recordData[i].bump[0]- recordData[i].jointPos.Urgq[1][X], recordData[i].bump[1], recordData[i].bump[2]);
			//0-dist, 1-height, 2-depth //simi sit 
			if (abs(recordData[i].bump[0]) > 0.05 && abs(recordData[i].bump[2]) < 0.5 && urg_flag == 1)
			//if(flag==1)
			{
				k = i;
				urg_flag = 2;
				printf("dist %lf \n", recordData[i].bump[0]);
				//simi
				//break; //simi keep commented
			}
			if (urg_flag == 2) { //record only if all positive recordData[0].bump[0], recordData[0].bump[1], recordData[0].bump[2]); 
				fprintf(fp, "%d\t %d\t %f\t", i, recordData[i].per_flag, recordData[i].torq_per_comp);
				fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq_per[R][H], recordData[i].torq_per[R][K], recordData[i].torq_per[R][A]);
				fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq_per[L][H], recordData[i].torq_per[L][K], recordData[i].torq_per[L][A]);
				fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq[R][H], recordData[i].torq[R][K], recordData[i].torq[R][A]);
				fprintf(fp, "%+12.6lf %+12.6lf %+12.6lf ", recordData[i].torq[L][H], recordData[i].torq[L][K], recordData[i].torq[L][A]);
				//simi Perception assist on
				for (int j = 0; j < EMG_CH; j++) {
					fprintf(fp, "%+6.7lf\t", recordData[i].emg.rms[R][j]); //check before exp //imp
				} //simi MPU
				//fprintf(fp, "%d\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\n", i - k + 1, recordData[i].jointPos.Foot[R].vel[X], recordData[i].jointPos.Foot[R].vel[Y], recordData[i].jointPos.Foot[R].acc[X], recordData[i].jointPos.Foot[R].acc[Y], recordData[i].jointPos.UppThigh[R].vel[X], recordData[i].jointPos.UppThigh[R].vel[Y], recordData[i].jointPos.XZmp);
				fprintf(fp, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %+12.6lf\t %+12.6lf\t %+12.6lf\t %+12.6lf\t %+12.6lf %+12.6lf\t %+12.6lf\t", //swing toe,heel pos,swing foot pos,vel,acc+swing hip loc,upp thigh vel+X_ZMP+hip angle L,R+knee angle L,R+ankle angle L,R
					recordData[i].jointPos.Toe[R][X], recordData[i].jointPos.Toe[R][Y], recordData[i].jointPos.Heel[R][X], recordData[i].jointPos.Heel[R][Y], recordData[i].jointPos.Foot[R].pos[X], recordData[i].jointPos.Foot[R].pos[Y], recordData[i].jointPos.Foot[R].vel[X], recordData[i].jointPos.Foot[R].vel[Y], recordData[i].jointPos.Foot[R].acc[X], recordData[i].jointPos.Foot[R].acc[Y], recordData[i].jointPos.Hip[L][X], recordData[i].jointPos.Hip[L][Y], recordData[i].jointPos.UppThigh[R].vel[X], recordData[i].jointPos.UppThigh[R].vel[Y], recordData[i].jointPos.XZmp, recordData[i].enc[L][H], recordData[i].enc[R][H], recordData[i].enc[L][K], recordData[i].enc[R][K], recordData[i].enc[L][A], recordData[i].enc[R][A]);
				fprintf(fp, "%f\t %f\t %f\t %f\t", recordData[i].yaw1,recordData[i].roll, recordData[i].pitch2, recordData[i].yaw2);//simi removed recordData[i].pitch1,
				for (int j = 0; j < 7; j++) {
					fprintf(fp, "%lf\t ", recordData[k].bump[j]);
				}
			}
			fprintf(fp, "\n");
		}
		fprintf(fp, "\n");
		fclose(fp);
		//return 0;
	}
	if (REC_NN == ON) { //simi new funcq
		FILE_WRITE_OPEN(fp, "output_data/NN7_sit_f8.dat");
		for (int i = 0; i < w; i++) {
			if (abs(recordData[i].bump[0]) <3){//just to make sure it is initialized
				for (int j = 0; j < NN_INPUTS; j++) {
					fprintf(fp, "%+6.7lf\t", recordData[i].inputs[j]);
				}
				for (int j = 0; j < NN_OUTPUTS; j++) {
					fprintf(fp, "%+6.7lf\t", recordData[i].outputs[j]);
				}
				fprintf(fp, "\n");
			}	
		}
		fprintf(fp, "\n");
		fclose(fp);
		
	}
	return 0;
}
//simi added a newline after data output after every trial

/********************************************************
関数名：ReadData
説明　：ファイルからデータを読み込むための関数
引数　：data_struct *data		データを確保するための構造体
		
********************************************************/
	void ReadData(data_struct *data) {


		//ファイルの読み込み
		FILE *fp;

		FILE_READ_OPEN(fp, "output_data/data_size.dat");
		fscanf(fp, "%d", &data->length);
		fclose(fp);


		data->element = new valueParam[data->length];
		data->joint = new draw_joint[data->length];


		/*FILE_READ_OPEN( fp, "output_data/Torq/torq.dat" );
		for(int i=0;i<data->length;i++){
			fscanf(fp,"%lf %lf %lf %lf %lf %lf", &data->element[i].Torq[R][H], &data->element[i].Torq[R][K], &data->element[i].Torq[R][A]
												, &data->element[i].Torq[L][H], &data->element[i].Torq[L][K], &data->element[i].Torq[L][A]);
		}
		fclose(fp);*/

		FILE_READ_OPEN(fp, "output_data/VirtualWall/bump.dat");
		for (int i = 0; i < data->length; i++) {
			fscanf(fp, "%lf %lf %lf", &data->element[i].Bump[DISTANCE], &data->element[i].Bump[HEIGHT], &data->element[i].Bump[DEPTH]);
				//, &data->element[i].Bump[VIR_START_X], &data->element[i].Bump[VIR_START_Y], &data->element[i].Bump[VIR_END_X], &data->element[i].Bump[VIR_END_Y]);
		}
		fclose(fp);
	
	/*FILE_READ_OPEN( fp, "output_data/Position/JointPosition_data.dat" );
	for(int i=0;i<data->length;i++){
		fscanf(fp,"%lf %lf", &data->element[i].Foot[X], &data->element[i].Foot[Y] );
	}
	fclose(fp);*/

	//****** 姿勢 *********
		//各関節座標
	FILE_READ_OPEN( fp, "output_data/Position/JointPos.dat" );
	for( int i=0 ; i<data->length ; i++ ){
		fscanf( fp, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t \n", 
			&data->joint[i].Hip[R][X], &data->joint[i].Hip[R][Y], &data->joint[i].Knee[R][X], &data->joint[i].Knee[R][Y], 
			&data->joint[i].Ankle[R][X], &data->joint[i].Ankle[R][Y], &data->joint[i].Toe[R][X], &data->joint[i].Toe[R][Y], 
			&data->joint[i].Heel[R][X], &data->joint[i].Heel[R][Y],

			&data->joint[i].Hip[L][X], &data->joint[i].Hip[L][Y], &data->joint[i].Knee[L][X], &data->joint[i].Knee[L][Y], 
			&data->joint[i].Ankle[L][X], &data->joint[i].Ankle[L][Y], &data->joint[i].Toe[L][X], &data->joint[i].Toe[L][Y], 
			&data->joint[i].Heel[L][X], &data->joint[i].Heel[L][Y], &data->joint[i].Body[X], &data->joint[i].Body[Y]);
	}
	fclose( fp );


	for(int i=0;i<data->length;i++){
		for(int j=0;j<2;j++){
			data->joint[i].Hip[j][Z] = -0.3*j;	data->joint[i].Knee[j][Z] = -0.3*j;	data->joint[i].Ankle[j][Z] = -0.3*j;	data->joint[i].Toe[j][Z] = -0.3*j; data->joint[i].Heel[j][Z] = -0.3*j;	
		}
		data->joint[i].Body[Z] = -0.15;
		data->element[i].Bump[DEPTH] = 0.2;
	}
	
	
}


/********************************************************
関数名：SetData_ToolBar
説明　：読み込んだデータをツールバーの変数に格納するための関数
引数　：int t					時間
		data_struct data		データを確保するための構造体
		tweakParam *barParam	ツールバーに設定する変数を格納した構造体

出力　：tweakParam  *barParam	
********************************************************/
void SetData_ToolBar(int t, data_struct data, tweakParam *barParam){

	barParam->valParam.Foot[X] = data.joint[t].Toe[R][X];	barParam->valParam.Foot[Y] = data.joint[t].Toe[R][Y];
	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			barParam->valParam.Torq.out[i][j] = data.torq.out[i][j];
			barParam->valParam.Torq.pow[i][j] = data.torq.pow[i][j];
			barParam->valParam.Torq.per[i][j] = data.torq.per[i][j];
		}
	}
	barParam->valParam.Bump[DISTANCE] = data.bump[DISTANCE];	barParam->valParam.Bump[HEIGHT] = data.bump[HEIGHT];	barParam->valParam.Bump[DEPTH] = data.bump[DEPTH];
	barParam->valParam.Bump[VIR_START_X] = data.bump[VIR_START_X];	barParam->valParam.Bump[VIR_START_Y] = data.bump[VIR_START_Y];	barParam->valParam.Bump[VIR_END_X] = data.bump[VIR_END_X]; barParam->valParam.Bump[VIR_END_Y] = data.bump[VIR_END_Y];
	barParam->valParam.ZmpX = 5.0;
	barParam->valParam.time = data.time;
	barParam->valParam.Freq = data.freq;

}



/********************************************************
関数名：Adjust_SamplingTime
説明　：サンプリングタイムを調整するための関数
Function for adjusting the sampling time
引数　：int t				時間
		
********************************************************/
int Adjust_SamplingTime(int t, double *timeAvg){
	static int i=0;
	static double time_ave;
	double time=0;
	static LARGE_INTEGER freq,start_time, end_time;
	static int flag = 0;	//サンプリング周波数チェックフラグ Sampling frequency check flag

	QueryPerformanceFrequency( &freq ); 
	QueryPerformanceCounter( &end_time );
	time = (double)(end_time.QuadPart - start_time.QuadPart)/(double)(freq.QuadPart);

	
	if(t!=0){
		//サンプリング周波数チェック
			if(time>SAMPLING_TIME){
				flag++;
			}
		//1ループが時間内ならビジーループで時間をつぶす
		while( time<=SAMPLING_TIME ) {
			QueryPerformanceCounter( &end_time );
			time = (double)(end_time.QuadPart - start_time.QuadPart)/(double)(freq.QuadPart);			
		}
	}
	time_ave += time;
	
	//　経過時間表示
	if(t==(SAMPLING_FREQ*i)){
		
		printf("%d [s]\n", i);
		time_ave /= SAMPLING_FREQ;			//1ループする平均時間
		*timeAvg = 1/time_ave;	
		printf("Sampling Average Freq = %f [Hz]\n",(1/time_ave));
		printf("Sampling Freq = %f [Hz]\n\n",(1/time));
		if(flag>=1)	printf("Sampling time is OVER!! \t 秒間%d回オーバーしました\n",(flag+1));
			
		time_ave =0;
		flag = 0;
		
		i++;
		
	}

	QueryPerformanceFrequency( &freq ); 
	QueryPerformanceCounter( &start_time );
	
	return 0; 
}