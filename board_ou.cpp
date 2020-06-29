#include "board.h"

//ALL BOARDS
void BOARD::SetAll() {
	ENC.SetBoard_ENC();
	EMG.SetBoard_AD();
	AD_R.SetBoard_AD();
	AD_L.SetBoard_AD();
	DA_R.SetBoard_DA();
	DA_L.SetBoard_DA();
}
void BOARD::ReadAllSenosrs(Robot *robot, int t, float force_avg[2][3][3], float sampletime) {

	static float enc_pre[3][2][3]; //[ANG VEL ACC][RL][HKA] Readを使わないとき用
	static float buf_enc[3][2][3][2][FILTER_LEN];	//[ANG VEL ACC][RL][HKA][][]エンコーダのフィルタリング用
	static float time_sum;

	time_sum += sampletime;

	if ((t % (SAMPLING_FREQ / 100)) == 1) {	//100Hz
		
		for (int i = 0; i < 3; i++) {
			//100Hz
			(*robot).sensorData.enc[ANG][R][i] = ENC.Read_ENC(i, GEAR_RATIO[i]);
			(*robot).sensorData.enc[ANG][L][i] = -ENC.Read_ENC(i + 3, GEAR_RATIO[i]);
			(*robot).sensorData.enc[ANG][R][i] = LowPassFilter(100.0f, 10.0f, (*robot).sensorData.enc[ANG][R][i], buf_enc[ANG][R][i]);
			(*robot).sensorData.enc[ANG][L][i] = LowPassFilter(100.0f, 10.0f, (*robot).sensorData.enc[ANG][L][i], buf_enc[ANG][L][i]);
			
			(*robot).sensorData.enc[VEL][R][i] = ((*robot).sensorData.enc[ANG][R][i] - enc_pre[ANG][R][i]) / time_sum;// time;
			(*robot).sensorData.enc[VEL][L][i] = ((*robot).sensorData.enc[ANG][L][i] - enc_pre[ANG][L][i]) / time_sum;// time;
			(*robot).sensorData.enc[VEL][R][i] = LowPassFilter(100.0f, 2.0f,(*robot).sensorData.enc[VEL][R][i], buf_enc[VEL][R][i]);
			(*robot).sensorData.enc[VEL][L][i] = LowPassFilter(100.0f, 2.0f, (*robot).sensorData.enc[VEL][L][i], buf_enc[VEL][L][i]);
			
			(*robot).sensorData.enc[ACC][R][i] = ((*robot).sensorData.enc[VEL][R][i] - enc_pre[VEL][R][i]) / time_sum;// time;
			(*robot).sensorData.enc[ACC][L][i] = ((*robot).sensorData.enc[VEL][L][i] - enc_pre[VEL][L][i]) / time_sum;// time;
			//(*robot).sensorData.enc[ACC][R][i] = LowPassFilter(100, 2, (*robot).sensorData.enc[ACC][R][i], buf_enc[ACC][R][i]);
			//(*robot).sensorData.enc[ACC][L][i] = LowPassFilter(100, 2, (*robot).sensorData.enc[ACC][R][i], buf_enc[ACC][L][i]);

			enc_pre[ANG][R][i] = (*robot).sensorData.enc[ANG][R][i];
			enc_pre[ANG][L][i] = (*robot).sensorData.enc[ANG][L][i];
			enc_pre[VEL][R][i] = (*robot).sensorData.enc[VEL][R][i];
			enc_pre[VEL][L][i] = (*robot).sensorData.enc[VEL][L][i];
			enc_pre[ACC][R][i] = (*robot).sensorData.enc[ACC][R][i];
			enc_pre[ACC][L][i] = (*robot).sensorData.enc[ACC][L][i];
		}

		(*robot).sensorData.enc[ANG][R][H] -= 0.0;// 0.2;
		(*robot).sensorData.enc[ANG][L][H] -= 0.0;// 0.1;
		(*robot).sensorData.enc[ANG][R][K] += 0.0;
		(*robot).sensorData.enc[ANG][L][K] -= 0.0;// 0.1;
		enc_pre[ANG][R][H] = (*robot).sensorData.enc[ANG][R][H];
		enc_pre[ANG][L][H] = (*robot).sensorData.enc[ANG][L][H];
		enc_pre[ANG][R][K] = (*robot).sensorData.enc[ANG][R][K];
		enc_pre[ANG][L][K] = (*robot).sensorData.enc[ANG][L][K];

		time_sum = 0.0f;

	}
	else {
		//エンコーダの値にenc_preを入れる
		for (int i = 0; i < 3; i++) {
			(*robot).sensorData.enc[ANG][R][i] = enc_pre[ANG][R][i];
			(*robot).sensorData.enc[ANG][L][i] = enc_pre[ANG][L][i];
			(*robot).sensorData.enc[VEL][R][i] = enc_pre[VEL][R][i];
			(*robot).sensorData.enc[VEL][L][i] = enc_pre[VEL][L][i];
			(*robot).sensorData.enc[ACC][R][i] = enc_pre[ACC][R][i];
			(*robot).sensorData.enc[ACC][L][i] = enc_pre[ACC][L][i];
		}
	}

	EMG.Read_AD();
	for (int i = 0; i < EMG_CH; i++) {
		(*robot).sensorData.emg[0][R][i] = EMG.getData(i);
		(*robot).sensorData.emg[0][L][i] = EMG.getData(i + EMG_CH);
	}
	//rms
	calcRMS(t, &(*robot).sensorData);				//8chずつ筋電のRMSを計算

	for (int i = 0;i < EMG_CH;i++)(*robot).sensorData.emg[rms][L][i] = (*robot).sensorData.emg[rms][R][i];

	(RLswitch == 1) ? AD_R.Read_AD() : AD_L.Read_AD();	//実質1000Hzになる

	int AD_count = 0;
	for (int i = 0; i < 2; i++) {
		(*robot).sensorData.tact[R][i] = AD_R.getData(AD_count);
		(*robot).sensorData.tact[L][i] = AD_L.getData(AD_count);
		AD_count++;
	}//ここでAD_count =2になる

	static float force_pre[2][3][3]; //AD_Readを使わないとき用
	static float buf_fsensor[2][3][3][2][FILTER_LEN] = {};	//力センサのフィルタリング用

	for (int i = 0; i < 3; i++) {	//HKA
		for (int j = 0; j < 3; j++) {	//XYZ
			if (RLswitch == 1) {	//1000Hz
				(*robot).sensorData.forceV[R][i][j] = AD_R.getData(AD_count);
				(*robot).sensorData.forceV[R][i][j] = LowPassFilter(1000.0f, 10.0f, (*robot).sensorData.forceV[R][i][j], buf_fsensor[R][i][j]) - force_avg[R][i][j];
				(*robot).sensorData.forceV[L][i][j] = force_pre[L][i][j];
				force_pre[R][i][j] = (*robot).sensorData.forceV[R][i][j];
			}
			else {
				(*robot).sensorData.forceV[L][i][j] = AD_L.getData(AD_count);
				(*robot).sensorData.forceV[L][i][j] = LowPassFilter(1000.0f, 10.0f, (*robot).sensorData.forceV[L][i][j], buf_fsensor[L][i][j]) - force_avg[L][i][j];
				(*robot).sensorData.forceV[R][i][j] = force_pre[R][i][j];
				force_pre[L][i][j] = (*robot).sensorData.forceV[L][i][j];
			}
			AD_count++;

		}
		//printf("%d,%f,%f,%f\n", t, (*robot).sensorData.forceV[L][H][X], (*robot).sensorData.forceV[L][H][Y], (*robot).sensorData.forceV[L][H][Z]);
		Matrix3331_Product(FORCE_ROT_MAT[R][i], (*robot).sensorData.forceV[R][i], (*robot).sensorData.forceV[R][i]);	//回転行列をかけて、座標系をロボットに合わせる
		Matrix3331_Product(FORCE_ROT_MAT[L][i], (*robot).sensorData.forceV[L][i], (*robot).sensorData.forceV[L][i]);
		Matrix3331_Product(FORCE_COMP_MAT[R][i], (*robot).sensorData.forceV[R][i], (*robot).sensorData.forceN[R][i]);	//干渉補正行列をかけて、電圧(V)を力(N)にする
		Matrix3331_Product(FORCE_COMP_MAT[L][i], (*robot).sensorData.forceV[L][i], (*robot).sensorData.forceN[L][i]);
		//printf("%d,%f,%f,%f\n", t, (*robot).sensorData.forceN[R][H][X], (*robot).sensorData.forceN[R][H][Y], (*robot).sensorData.forceN[R][H][Z]);
	}
	//printf("%d,%f,%f,%f\n", t, (*robot).sensorData.forceN[R][H][X], force_pre[R][H][Y], force_avg[R][H][Z]);
	//printf("%d,%f,%f,%f\n", t, (*robot).sensorData.forceN[R][H][X], (*robot).sensorData.forceN[R][H][Y], (*robot).sensorData.forceN[R][H][Z]);
	//printf("%d,%f\n", t, (*robot).sensorData.forceN[R][H][X]);
	RLswitch = 1 - RLswitch;
	//AD_count 2~10

}

void BOARD::calibrateForceSensor(Robot *robot, float force_avg[2][3][3]) {
	cout<<"\n*** Start Calibration *** \n";
	cout<<"Program will start in 3[s]\n";
	cout<<"Please wait...\n";

	float preForceV[2][3][3] = {};	//[RL][HKA][XYZ]、電圧(V)
	static float stored_f[2][3][3][2][FILTER_LEN];// = {0};	//フィルタリングに使用、[RL][HKA][XYZ][][]

	static LARGE_INTEGER freq, start_time, end_time;
	float time = 0;
	int count = 0;
	
	int AD_count = 2;
	int t;
	for (t = 0; t < PRE_TASK/2; t++)	//2000Hzの半分の1000Hz
	{
		/******		力センサの読み取り		******/
		AD_count = 2;		//AD_count 2~10、力センサの値が入るチャンネル
		AD_R.Read_AD();
		AD_L.Read_AD();
		for (int i = 0; i < 3; i++) {	//HKA
			for (int j = 0; j < 3; j++) {	//XYZ
				preForceV[R][i][j] = AD_R.getData(AD_count);
				preForceV[L][i][j] = AD_L.getData(AD_count);
				force_avg[R][i][j] += preForceV[R][i][j];
				force_avg[L][i][j] += preForceV[L][i][j];
				AD_count++;
			}
		}

		// wait until sampling time 　サンプリングタイム調整　1000Hz//
		QueryPerformanceFrequency(&freq);
		QueryPerformanceCounter(&end_time);
		time = (float)(end_time.QuadPart - start_time.QuadPart) / (float)(freq.QuadPart);
		if (t != 0) { 									//1ループが時間内ならビジーループで時間をつぶす
			while (time <= SAMPLING_TIME*2) {	//2000Hzの半分の1000Hz
				QueryPerformanceCounter(&end_time);
				time = (float)(end_time.QuadPart - start_time.QuadPart) / (float)(freq.QuadPart);
			}
		}
		if (t == (SAMPLING_FREQ/2*count)) {	//2000Hzの半分の1000Hz
			printf("%d [s]\n", (PRE_TIME - count));		//　経過時間表示
			count++;
		}
		QueryPerformanceCounter(&start_time);
	}
	//printf("%f\n", force_avg[R][H][X]);
	for (int i = 0; i<2; i++) for (int j = 0; j<3; j++) for (int k = 0; k<3; k++) force_avg[i][j][k] /= t;	//2000Hzの半分の1000Hz
	printf("Roop : %d times\n",t);
}
void BOARD::motorOutput(int judge, Robot *robot) {
	if (judge == ON) {	
		DA_R.convert(0, (*robot).torq.fin[R][H]);	DA_R.convert(1, (*robot).torq.sw[R][H]);
		DA_R.convert(2, (*robot).torq.fin[R][K]);	DA_R.convert(3, (*robot).torq.sw[R][K]);
		DA_R.convert(4, (*robot).torq.fin[R][A]);	DA_R.convert(5, (*robot).torq.sw[R][A]);
		DA_R.OutBoard_DA();

		DA_L.convert(0, (*robot).torq.fin[L][H]);	DA_L.convert(1, (*robot).torq.sw[L][H]);
		DA_L.convert(2, (*robot).torq.fin[L][K]);	DA_L.convert(3, (*robot).torq.sw[L][K]);
		DA_L.convert(4, (*robot).torq.fin[L][A]);	DA_L.convert(5, (*robot).torq.sw[L][A]);
		DA_L.OutBoard_DA();
	}
	else {
		DA_R.convert(0, 0);	DA_R.convert(1, 0);
		DA_R.convert(2, 0);	DA_R.convert(3, 0);
		DA_R.convert(4, 0);	DA_R.convert(5, 0);
		DA_R.OutBoard_DA();

		DA_L.convert(0, 0);	DA_L.convert(1, 0);
		DA_L.convert(2, 0);	DA_L.convert(3, 0);
		DA_L.convert(4, 0);	DA_L.convert(5, 0);
		DA_L.OutBoard_DA();
	}	
}
void BOARD::CloseAll() {
	ENC.CloseENC();
	EMG.CloseAD();
	AD_R.CloseAD();
	AD_L.CloseAD();
	DA_R.CloseDA();
	DA_L.CloseDA();
}

//ENCODER
ENC_BOARD::ENC_BOARD(board_t ID, int ch) : m_ID(ID), m_ch(ch) {};
void ENC_BOARD::SetBoard_ENC() {
	m_hBoard_ENC = PencOpen(m_ID, PENC_FLAG_SHARE);
	if (m_hBoard_ENC == INVALID_HANDLE_VALUE) {
		printf("Failed to open the specified device.");
		exit(-1);
	}
	// R			  // L
	m_ChNo[0] = 1;	m_ChNo[3] = 5; // H
	m_ChNo[1] = 2;	m_ChNo[4] = 6; // K
	m_ChNo[2] = 3;	m_ChNo[5] = 7; // A
	for (int i = 0; i<m_ch; i++) {
		m_mode[i] = 0x06;
		m_nRet = PencSetMode(m_hBoard_ENC, m_ChNo[i], m_mode[i], 0, 1, 0);
		if (m_nRet != PENC_ERROR_SUCCESS) 	
			exit(-1);
	}
}
float ENC_BOARD::Read_ENC(int ch, int coef) {
	float data;
	unsigned long   raw;
	m_nRet = PencGetCounter(m_hBoard_ENC, m_ChNo[ch], &raw);
	if (m_nRet != PENC_ERROR_SUCCESS) {
		return(-1);
	}
	if (raw>0xAFFFF) {
		raw = 0xFFFFFF - raw;
		data = -(float)raw;
	}
	else	data = (float)raw;
	data *= 2 * M_PI / (coef * 1024 * 4);
	return data;
}
void ENC_BOARD::CloseENC() {
	PencClose(m_hBoard_ENC);
}

//AD board
AD_BOARD::AD_BOARD(board_t ID, int ch) : m_ID(ID) ,m_ch(ch) {};
void AD_BOARD::SetBoard_AD() {
	m_hBoard_AD = AdOpen(m_ID);
	if (m_hBoard_AD == INVALID_HANDLE_VALUE) {
		printf("Failed to open the specified device.");
		exit(-1);
	}
	m_nRet = AdBmGetSamplingConfig(m_hBoard_AD, &m_paramAD_BMS);
	if (m_nRet != AD_ERROR_SUCCESS) {
		printf("AdBmGetSamplingConfig errr(%lx)", m_nRet);
		AdClose(m_hBoard_AD);
		exit(-1);
	}
	m_paramAD_BMS.ulChCount = m_ch;
	for (int i = 0; i < m_ch; i++) {
		m_paramAD_BMS.SmplChReq[i].ulChNo = i + 1;
		m_paramAD_BMS.SmplChReq[i].ulRange = AD_10V;
	}
	ULONG smpl_numb = 1;
	m_paramAD_BMS.ulSmplNum = smpl_numb;
	m_paramAD_BMS.fSmplFreq = 0.0f;
	m_paramAD_BMS.fScanFreq = 206999.0f;

	m_nRet = AdBmSetSamplingConfig(m_hBoard_AD, &m_paramAD_BMS);
	if (m_nRet != AD_ERROR_SUCCESS) {
		printf("AdBmSetSamplingConfig errr(%lx)", m_nRet);
		AdClose(m_hBoard_AD);
		exit(-1);
	}
}
void AD_BOARD::Read_AD() {
		m_nRet = AdInputAD(m_hBoard_AD, m_ch, AD_INPUT_SINGLE, m_paramAD_BMS.SmplChReq, m_raw);
		if (m_nRet != AD_ERROR_SUCCESS) {
			printf("EMG AdInputAD errr(%lx)", m_nRet);
			AdClose(m_hBoard_AD);
			exit(-1);
		}
}

float AD_BOARD::getData(int ch) {
	float val = (float(m_raw[ch])*20.0f / 65535.0f) - 10.0f;
	return val;
}
void AD_BOARD::CloseAD() {
	AdClose(m_hBoard_AD);
}

//DA board
DA_BOARD::DA_BOARD(board_t ID, int ch) : m_ID(ID), m_ch(ch) {};
void DA_BOARD::SetBoard_DA() {
	m_hBoard_DA = DaOpen(m_ID);
	for (int i = 0; i<m_ch; i++) {
		m_paramDA[i].ulChNo = i + 1;
		m_paramDA[i].ulRange = DA_0_5V;
	}
}
void DA_BOARD::convert(int ch, float val) {
	m_data[ch] = static_cast<WORD>(val * 4095 / 5.0);
}
void DA_BOARD::OutBoard_DA() {
	m_nRet = DaOutputDA(m_hBoard_DA, m_ch, &m_paramDA[0], m_data);
	if (m_nRet != DA_ERROR_SUCCESS) {
		printf("DaOutputDA errr(%lx)\n", m_nRet);
		DaClose(m_hBoard_DA);
		return;
	}
}
void DA_BOARD::CloseDA() {
	DaClose(m_hBoard_DA); 
}