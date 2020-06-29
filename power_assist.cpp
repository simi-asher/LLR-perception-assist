/*****************************************************************************************************************************
******************************************************************************************************************************
パワーアシストの計算に必要な関数をまとめたヘッダファイル


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "power_assist.h"



/********************************************************
関数名：Calculate_Power_Assist
説明　：パワーアシスト計算用の関数
		メインループ内で使用
引数　：const int learnFlag					  ウエイト値を学習させるかどうかのフラグ
		double weight_emg[3][EMG_CH]		　各筋の筋電に対するウエイト値
		double emg[2][EMG_CH]				　各筋の筋電のrms値
		double forceData[2][3][2]			  力センサのデータ
　　　　double torq[2][3]　　　　　　　　　　 発生させる関節トルク
		
出力　：double torq[2][3]
********************************************************/
void Calculate_Power_Assist(const int learnFlag, _weight *weight_fuzzy, double encData[2][3], double weight_emg[3][EMG_CH], double emg[2][EMG_CH], double forceData[2][3][3], double torq[2][3]){

	double torq_forceSensor[2][3], torq_emg[2][3];
	double weight_assist[2];
	
	//double weight_p[3][EMG_CH];		//調整したウエイト値

	for(int i=0;i<2;i++){
		for(int j=0;j<3;j++){
			torq_forceSensor[i][j] = 0.0;
			torq_emg[i][j]		   = 0.0;
		}
	}

#ifdef USE_EMG
	//ファジィニューロ調整器の計算
	FuzzyNeuro(learnFlag, weight_fuzzy, encData[R], emg[R], forceData[R], weight_emg);
	
	//筋電ベースのパワーアシストの計算
	//EMG_BaseAssist(weight_emg, emg, torq_emg);
	EMG_BaseAssist(weight_fuzzy->ori, emg, torq_emg);
	
#endif

#ifdef USE_FORCESENSOR
	//力センサベースのパワーアシストの計算
	ForceSensorBaseAssist(forceData, torq_forceSensor);

#ifndef USE_EMG
	for(int i=0;i<EMG_CH;i++)	emg[R][i] = 0;
#endif

#endif

	//大腿筋膜張筋の筋電値から力センサベースと筋電ベースのウエイト値を計算
	weight_assist[R] = Weight_ForceSensorAndEMG(emg[R][0]);
	weight_assist[L] = Weight_ForceSensorAndEMG(emg[L][0]);

	if(emg[R][0]>emg[L][0])	 weight_assist[L]=0;
	else					 weight_assist[R]=0;

	//力センサベースと筋電ベースの比率を考慮したトルクの計算
	Sum_ForceSensorAndEMG_BaseAssist(weight_assist, torq_forceSensor, torq_emg, torq);
	
	/*for(int i=0;i<3;i++){
			printf("%lf\t",torq_forceSensor[R][i]);
	}
	printf("\n");*/
}


/********************************************************
関数名：ForceSensorBaseAssist
説明　：力センサベースのパワーアシスト
引数　：double forceData[2][3][2]			力センサのデータ
　　　　double torq[2][3]　　　　　　　　　　 発生させる関節トルク
		
出力　：double torq[2][3]
********************************************************/
void ForceSensorBaseAssist(double forceData[2][3][3], double torq[2][3]){

	double force[2][3];

	//力センサの値が±0.01以下なら0とする（力センサはX軸しか使用していないためY軸は計算しない）
	for(int i=0;i<2;i++)
		for(int j=0;j<3;j++)
			if(forceData[i][j][X]<0.01 && forceData[i][j][X]>-0.01)	forceData[i][j][X] = 0;
		
	//力センサの値を[V]から[kg]に変換
	Compensation_ForceSensor( forceData );


	//発生させる力の計算とトルクへの変換
	for(int i=0;i<2;i++){
		//人側のフレームとロボット側のフレームで力センサの値が反転するため(-1)をかける
		force[i][H] = P_Control(GAIN_P_H, -1*forceData[i][H][X], 0);
		force[i][K] = P_Control(GAIN_P_K, -1*-1*forceData[i][K][X], 0);		//膝のみ回転方向が違うため(-1)をかける
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
力センサ干渉補正行列
************************************************************/
//2018/9/26よりCOEF_COMPENSATION_FORCEの代わりに使用+

double CompensationMatrix_Force_RH[3][3] = { { 191.1, 3.902, 2.341 },
{ -39.58, 211.1, 24.26 },
{ -7.973, -6.904, 325.8 } };

double CompensationMatrix_Force_RK[3][3] = { { 209.0, 66.12, 0 },//Z軸が壊れているのでZの出力を無視（力センサ番号UL130301）
{ 6.767, 253.2, 0 },
{ 0, 0, 0 } };

double CompensationMatrix_Force_RA[3][3] = { { 0, 0, 0 },//足首は計測できていないので不使用（2018/1/30）
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
関数名：Compensation_ForceSensor
説明　：力センサの補正 [V]→[N]への変換
引数　：double forceData[2][3][2]			力センサのデータ
		
出力　：double forceData[2][3][2]
********************************************************/


void Compensation_ForceSensor(double forceData[2][3][3]) {

	//forceData[R][H][X] = COEFF_COMPENSATION_FORCE_RH*forceData[R][H][X];
	//forceData[R][K][X] = COEFF_COMPENSATION_FORCE_RK*forceData[R][K][X];
	//forceData[R][A][X] = COEFF_COMPENSATION_FORCE_RA*forceData[R][A][X];
	//forceData[L][H][X] = COEFF_COMPENSATION_FORCE_LH*forceData[L][H][X];
	//forceData[L][K][X] = COEFF_COMPENSATION_FORCE_LK*forceData[L][K][X];
	//forceData[L][A][X] = COEFF_COMPENSATION_FORCE_LA*forceData[L][A][X];

	//tempforce[RL][HKA][XYZ]
	double tempforce[2][3][3] = {};//それぞれの力センサ座標系における力センサ値[N]（後で下肢ロボットの座標系に変換する）

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

	for (int i = 0; i < 3; i++)//forceDataにtempforceを代入する時に、下肢ロボットの座標系にあわせる
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
関数名：EMG_BaseAssist
説明　：筋電ベースのパワーアシスト
引数　：double weight[3][EMG_CH]			各筋の筋電に対するウエイト値
　　　　double emg[2][EMG_CH]				rms処理した筋電値
		double torq[2][3]　　　　　　　　　 発生させる関節トルク
		
出力　：double torq[2][3]
********************************************************/
void EMG_BaseAssist(double weight[3][EMG_CH], double emg[2][EMG_CH], double torq[2][3]){

	//トルクの初期化
	for(int i=0;i<2;i++)
		for(int j=0;j<3;j++)
			torq[i][j] = 0;

	//各筋電からトルクの推定
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
関数名：Sum_ForceSensorAndEMG_BaseAssist
説明　：筋電ベースのパワーアシスト
引数　：const double weight     			力センサベースと筋電ベースのウエイト値
　　　　double torq_forceSensor[2][3]		力センサベースのトルク
		double torq_emg[2][3]				筋電ベースのトルク
		double torq[2][3]　　　　　　　　　 発生させる関節トルク
		
出力　：double torq[2][3]
********************************************************/
void Sum_ForceSensorAndEMG_BaseAssist(const double weight[2], double torq_forceSensor[2][3], double torq_emg[2][3], double torq[2][3]){

	//パワーアシストトルクの計算
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
関数名：Weight_ForceSensorAndEMG
説明　：EMGとForceSensorのウエイト率
		EMGとForcesensorベースの制御のウエイト率を決定 シグモイド関数使用
//		(筋電の値が1.5[V]で0.5,	2[V]くらいで1.0になるように調整)適当
引数　：double emg					大腿筋膜張筋の筋電をrms処理した値
		
戻り値：1.0/(1.0+exp(-1.0*(-12.0+8*emg)))	EMGのweight(ForceSensorは　1-weight )
********************************************************/
inline double Weight_ForceSensorAndEMG(double emg){
	
	return 1.0/(1.0+exp(-1.0*8*(-1.5+emg*1)));		//1.0から上昇し始め2.0で上昇か終わる
	//return 1.0/(1.0+exp(-1.0*(-7.5+9*emg*1.0)));		//0.5から上昇し始め1.25で上昇か終わる
	
}

/********************************************************
関数名：P_Controlr
説明　：P制御のインライン関数
引数　：gain		 弾性係数
　　　　currentPos　 現在の位置
　　　　targetPos    目標の位置
		
戻り値：(gain)*( (targetPos) - (currentPos) )
********************************************************/
inline double P_Control(double gain, double currentPos, double targetPos){
	
	return	(  (gain)*( (targetPos) - (currentPos) )  );

}


/********************************************************
関数名：PD_Control
説明　：PD制御の関数
引数　：gain		 弾性係数
　　　　currentPos　 現在の位置
　　　　targetPos    目標の位置
		
戻り値：(gain)*( (targetPos) - (currentPos) )
********************************************************/
double PD_Control(double gain[2], double current[2], double target[2]){
	const int P=0, D=1;

	return	(  gain[P]*(target[P] - current[P]) + gain[D]*(target[D] - current[D]) );

}