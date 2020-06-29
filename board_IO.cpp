/*****************************************************************************************************************************
******************************************************************************************************************************
JWボードの入出力に必要な関数をまとめたソースファイル
JWRIBPCIファイル各種，boardファイル，Parameterファイルが必要
またJWボードを載せた実験用PCで使う必要がある


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "board_IO.h"
//#include "myFilter.h"
//#include "myFilter.cpp"
/********************************************************
関数名：Setting
説明　：ボードデータの取得，チャンネル設定，エンコーダスタート
		ループ開始前に使用
引数　：int *noRib				ボードのハンドル
		
出力　：int *noRib
********************************************************/


/********************************************************
関数名：Calibrate_Sensor
説明　：力センサのキャリブレーション
		ループ開始前に使用
引数　：const int *noRib			ボードのハンドル
		double force_avg[2][3][2]	キャリブレーションした力センサの値
		
出力　：double force_avg[2][3][2]
********************************************************/
void Calibrate_Sensor(BOARD *_board, double force_avg[2][3][2]){

	printf("\n*** Start Calibration *** \n");
	printf("Program will start in 3[s]\n");
	printf("Please wait...\n");

	double preForce[2][3][2];
	static double stored_f[2][3][2][FILTER_LEN];

	static LARGE_INTEGER freq,start_time, end_time;
	double time=0;
	int count=0;
	
	for(int t=0; t<PRE_TASK; t++){
		
	//力センサデータ取得	
		/*_board->AD1.GetBoard_AD();
		preForce[R][H][X] = _board->AD1.getData(CH_FORCE_RH_X, 10.0);
		preForce[R][K][X] = _board->AD1.getData(CH_FORCE_RK_X, 10.0);
		preForce[R][A][X] = _board->AD1.getData(CH_FORCE_RA_X, 10.0);
		preForce[L][H][X] = _board->AD1.getData(CH_FORCE_LH_X, 10.0);
		preForce[L][K][X] = _board->AD1.getData(CH_FORCE_LK_X, 10.0);
		preForce[L][A][X] = _board->AD1.getData(CH_FORCE_LA_X, 10.0);
		*/
		force_avg[R][H][X] += Filtering( t, preForce[R][H][X], stored_f[R][H]);
		force_avg[R][K][X] += Filtering( t, preForce[R][K][X], stored_f[R][K]);
		force_avg[R][A][X] += Filtering( t, preForce[R][A][X], stored_f[R][A]);
		force_avg[L][H][X] += Filtering( t, preForce[L][H][X], stored_f[L][H]);
		force_avg[L][K][X] += Filtering( t, preForce[L][K][X], stored_f[L][K]);
		force_avg[L][A][X] += Filtering( t, preForce[L][A][X], stored_f[L][A]);

		
	
		// wait until sampling time 　サンプリングタイム調整　//
		QueryPerformanceFrequency( &freq ); 
		QueryPerformanceCounter( &end_time );
		time = (double)(end_time.QuadPart - start_time.QuadPart)/(double)(freq.QuadPart);

		if(t!=0){		
			//1ループが時間内ならビジーループで時間をつぶす
			while( time<=SAMPLING_TIME ) {
				QueryPerformanceCounter( &end_time );
				time = (double)(end_time.QuadPart - start_time.QuadPart)/(double)(freq.QuadPart);			
			}
		}

		//　経過時間表示
		if(t==(SAMPLING_FREQ*count)){
			printf("%d [s]\n", (PRE_TIME - count));
			count++;
		}
		QueryPerformanceCounter( &start_time );
	

	}

	force_avg[R][H][X] /= (double)(PRE_TASK);		force_avg[R][K][X] /= (double)(PRE_TASK);		force_avg[R][A][X] /= (double)(PRE_TASK);
	force_avg[L][H][X] /= (double)(PRE_TASK);		force_avg[L][K][X] /= (double)(PRE_TASK);		force_avg[L][A][X] /= (double)(PRE_TASK);
	
	
}


/********************************************************
関数名：Read_Sensor
説明　：ボードデータの取得，チャンネル設定，エンコーダスタート
引数　：const int count				経過時間
		const int *noRib			ボードのハンドル
		double force_cal[2][3][2]	力センサのキャリブレーションした値
		struct Sensor_data *data	読み取ったセンサデータ
		
出力　：struct Sensor_data *data
********************************************************/
void Read_Sensor(const int count,  BOARD *_board, double force_cal[2][3][2], struct Sensor_data *data,float *Yaw1){// t is count

	static double stored_f[2][3][2][FILTER_LEN];
	_board->ENC.GetBoard_ENC();
	//_board->AD1.GetBoard_AD(); //simi
	_board->AD2.GetBoard_AD();

	/******		エンコーダの読み取り		******/
	static float enc_pre[1][2][3];
	static float buf_enc[1][2][3][2][FILTER_LEN];
	double temp=0.0;

	//if ((count % (SAMPLING_FREQ / 2000)) == 1) { //2000Hz
		for (int i = 0; i < 3; i++) {//switched L R back
			data->enc_data[R][i] = _board->ENC.getData(i, GEAR_RATIO[i]); //- change dir
			data->enc_data[L][i] = -_board->ENC.getData(i + 4, GEAR_RATIO[i]);
			temp = data->enc_data[R][K]; //change at run time, only for sitting
			data->enc_data[L][K] = temp;
			data->enc_data[R][H] = *Yaw1*DtoR; //temp
			//printf("%lf", *Yaw1);
			//simi filter encoders
			data->enc_data[R][i] = LowPassFilter(2000.0f, 10.0f, data->enc_data[R][i], buf_enc[POS][R][i]);
			data->enc_data[L][i] = LowPassFilter(2000.0f, 10.0f, data->enc_data[L][i], buf_enc[POS][L][i]);

			enc_pre[POS][R][i] = data->enc_data[R][i];
			enc_pre[POS][L][i] = data->enc_data[L][i];
		}
	//}
	//else { //copies pre values to other encoder records
	//	for (int i = 0; i < 3; i++) {
	//		data->enc_data[R][i] = enc_pre[POS][R][i];
	//		data->enc_data[L][i] = enc_pre[POS][L][i];
	//	}
	//}
	//std::cout <<"\nL: H "<< data->enc_data[L][H] * 180 / PI << " K "<<data->enc_data[L][K] * 180 / PI << " A " << data->enc_data[L][A] * 180 / PI << "\n"; //simi previously R
	//std::cout <<"R: H "<< data->enc_data[R][H]*180/PI << " K "<<data->enc_data[R][K] * 180 / PI<< " A " << data->enc_data[R][A] * 180 / PI <<"\n"; //simi previously R
	//足首暫定的に逆にしてます

	
	/******		力センサの読み取り		******/
	/// 後ろ向きがx軸正，下向きがy軸正，外向きがz軸正 (方向要確認）///
/*
	data->force_data[R][H][X] = _board->AD1.getData(CH_FORCE_RH_X, 10.0);
	data->force_data[R][K][X] = _board->AD1.getData(CH_FORCE_RK_X, 10.0);
	data->force_data[R][A][X] = _board->AD1.getData(CH_FORCE_RA_X, 10.0);
	data->force_data[L][H][X] = _board->AD1.getData(CH_FORCE_LH_X, 10.0);
	data->force_data[L][K][X] = _board->AD1.getData(CH_FORCE_LK_X, 10.0);
	data->force_data[L][A][X] = _board->AD1.getData(CH_FORCE_LA_X, 10.0);
	

	//力センサのフィルタリング
	data->force_data[R][H][X] = Filtering( count, data->force_data[R][H][X], stored_f[R][H]) - force_cal[R][H][X];
	data->force_data[R][K][X] = Filtering( count, data->force_data[R][K][X], stored_f[R][K]) - force_cal[R][K][X];
	data->force_data[R][A][X] = Filtering( count, data->force_data[R][A][X], stored_f[R][A]) - force_cal[R][A][X];
	data->force_data[L][H][X] = Filtering( count, data->force_data[L][H][X], stored_f[L][H]) - force_cal[L][H][X];
	data->force_data[L][K][X] = Filtering( count, data->force_data[L][K][X], stored_f[L][K]) - force_cal[L][K][X];
	data->force_data[L][A][X] = Filtering( count, data->force_data[L][A][X], stored_f[L][A]) - force_cal[L][A][X];
	*/
	//printf("%lf	 %lf  %lf  %lf\n",data->force_data[R][H][X] ,data->force_data[R][K][X] ,data->force_data[L][H][X] ,data->force_data[L][K][X]);
	/*	sen->tact[R][0] = _board->AD1.getData(CH_FOOT_RH, 10.0);
	sen->tact[R][1] = _board->AD1.getData(CH_FOOT_RT, 10.0);
	sen->tact[L][0] = _board->AD1.getData(CH_FOOT_LH, 10.0);
	sen->tact[L][1] = _board->AD1.getData(CH_FOOT_LT, 10.0);*/
	//筋電データ取得
	for(int i=0;i<EMG_CH;i++){
		data->emg.raw[R][i]= _board->AD2.getData(i, 10); //simi previously R
		//Read_Data_From_AD( noRib[1], i+EMG_CH, &data->emg.raw[L][i]);
	}

	/*for(int i=0;i<EMG_CH;i++){
		data->emg.raw[R][i] = 0;
		data->emg.raw[L][i] = 0;
	}*/
	
		//data->emg.raw[L][0] = data->emg.raw[R][0];
}


/********************************************************
関数名：Calculate_RMS
説明　：筋電データのRMS値を計算(キャリブレーションは行っていない)
引数　：const int t				経過時間
　　　　struct EMG_data *emgData	読み取ったセンサデータ
		
出力　：struct EMG_data *emgData
********************************************************/
void Calculate_RMS(const int t, struct EMG_data *emgData){

	static double emg[EMG_CH][MAX_STOCK];

	double EMG_sum[EMG_CH];
	for(int i=0; i<EMG_CH; i++){
		EMG_sum[i]=0;
		emg[i][t] = emgData->raw[R][i]; //simi previously R
	}

	for(int i=0; i<EMG_CH; i++)
	{
		
		//Calculate RMS
		if(t<NUM_RMS){
			for(int j=0; j<t; j++){
				EMG_sum[i] += emg[i][t] * emg[i][t];	 
			}
		}
		else{
			for(int j=0; j<NUM_RMS; j++){
				EMG_sum[i] += emg[i][t-j] * emg[i][t-j];	 
			}
		}
		emgData->rms[R][i] = sqrt(EMG_sum[i] / (double)NUM_RMS);	//simi previously R
	}	
	
}

/********************************************************
関数名：Calculate_RMS
説明　：筋電データのRMS値を計算(キャリブレーションは行っていない)
引数　：const int t				経過時間
　　　　struct EMG_data *emgData	読み取ったセンサデータ
		
出力　：struct EMG_data *emgData
********************************************************/
void Calculate_RMS_L(const int t, struct EMG_data *emgData){

	static double emg[EMG_CH][MAX_STOCK];

	double EMG_sum[EMG_CH];
	for(int i=0; i<EMG_CH; i++){
		EMG_sum[i]=0;
		emg[i][t] = emgData->raw[L][i]; //simi previously L
	}

	for(int i=0; i<EMG_CH; i++)
	{
		
		//Calculate RMS
		if(t<NUM_RMS){
			for(int j=0; j<t; j++){
				EMG_sum[i] += emg[i][t] * emg[i][t];	 
			}
		}
		else{
			for(int j=0; j<NUM_RMS; j++){
				EMG_sum[i] += emg[i][t-j] * emg[i][t-j];	 
			}
		}
		emgData->rms[L][i] = sqrt(EMG_sum[i] / (double)NUM_RMS);	//simi previously L
	}	
	
}

