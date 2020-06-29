#include "board.h"

/////ALL BOARDS
void BOARD::SetAll() {
	ENC.SetBoard_ENC();
	//AD1.SetBoard_AD();
	AD2.SetBoard_AD();
	DA1.SetBoard_DA();
	DA2.SetBoard_DA();
}
/*
void BOARD::ReadAllSenosrs(Robot *robot) {
#ifdef USE_ENC_BOARD
ENC.GetBoard_ENC();
for (int i = 0; i < 3; i++) {
(*robot).sensorData.enc[R][i] = ENC.getData(i, GEAR_RATIO[i]);
(*robot).sensorData.enc[L][i] = -ENC.getData(i + 4, GEAR_RATIO[i]);
}
#endif //USE_ENC_BOARD

#ifdef USE_AD1_BOARD

#ifndef USE_MULTHREAD
AD1.readSensorNoThread();
#endif
int AD1channel = 0;
for (int i = 0; i < 2; i++)
for (int j = 0; j < 2; j++)
(*robot).sensorData.tact[i][j] = AD1.getData(AD1channel++, 10.0);
for (int i = 0; i < 2; i++)
for (int j = 0; j < 3; j++)
for (int k = 0; k < 3; k++)
(*robot).sensorData.force.V[i][j][k] = AD1.getData(AD1channel++, 10.0);
#endif //USE_AD1_BOARD

#ifdef USE_EMG_BOARD
AD2.GetBoard_AD();
for (int i = 0; i < CH_EMG; i++) {
(*robot).sensorData.emg.raw[R][i] = AD2.getData(i, 10.0);
}
#endif //USE_EMG_BOARD
}*/
void BOARD::motorOutput(int judge, const double torq[2][3], const double rotation[2][3]) {

	if (judge == ON) {

		DA2.setData(0, torq[R][H]);	DA2.setData(1, rotation[R][H]);
		DA2.setData(2, torq[R][K]);	DA2.setData(3, rotation[R][K]);
		DA2.setData(4, torq[R][A]);	DA2.setData(5, rotation[R][A]);
		
		DA2.OutBoard_DA();

		DA1.setData(0, torq[L][H]);	DA1.setData(1, rotation[L][H]);
		DA1.setData(2, torq[L][K]);	DA1.setData(3, rotation[L][K]);
		DA1.setData(4, torq[L][A]);	DA1.setData(5, rotation[L][A]);

		DA1.OutBoard_DA();

	}
	else {

		DA2.setData(0, 0);	DA2.setData(1, 0);
		DA2.setData(2, 0);	DA2.setData(3, 0);
		DA2.setData(4, 0);	DA2.setData(5, 0);

		DA2.OutBoard_DA();

		DA1.setData(0, 0);	DA1.setData(1, 0);
		DA1.setData(2, 0);	DA1.setData(3, 0);
		DA1.setData(4, 0);	DA1.setData(5, 0);

		DA1.OutBoard_DA();

	}



}
void BOARD::CloseAll() {
	ENC.CloseENC();
	//AD1.CloseAD();
	AD2.CloseAD();
	DA1.CloseDA();
	DA2.CloseDA();
}
//////////////////////

/////////ENCODER
ENC_BOARD::ENC_BOARD(board_t ID) : m_ID(ID) {};
void ENC_BOARD::SetBoard_ENC() {
	hBoard_ENC = PencOpen(m_ID, PENC_FLAG_SHARE);
	if (hBoard_ENC == INVALID_HANDLE_VALUE) {
		printf("Failed to open the specified device.");
		exit(-1);
	}
	for (int i = 0; i<CH_ENC; i++) {
		ChNo[i] = i + 1;
		mode[i] = 0x06;
		nRet = PencSetMode(hBoard_ENC, ChNo[i], mode[i], 0, 1, 0);
		if (nRet != PENC_ERROR_SUCCESS) 	exit(-1);
	}
}
void ENC_BOARD::GetBoard_ENC() {
	for (int i = 0; i<CH_ENC; i++) {
		nRet = PencGetCounter(hBoard_ENC, ChNo[i], &raw[i]);
		if (nRet != PENC_ERROR_SUCCESS) {
			exit(-1);
		}
		else {
			if (raw[i]>0xAFFFF) {
				raw[i] = 0xFFFFFF - raw[i];
				data[i] = -(float)raw[i];
			}
			else	data[i] = (float)raw[i];
		}
	}
}

void ENC_BOARD::CloseENC() {
	PencClose(hBoard_ENC);
}
double ENC_BOARD::getData(int ch, int coef) {
	double val = double(data[ch]) * 2 * M_PI / (coef * 1024 * 4);
	return val;
}
///////////////////////

//////AD board
AD_BOARD::AD_BOARD(board_t ID) : m_ID(ID) {};
void AD_BOARD::SetBoard_AD() {
	hBoard_AD = AdOpen(m_ID);
	if (hBoard_AD == INVALID_HANDLE_VALUE) {
		printf("Failed to open the specified device.");
		exit(-1);
	}
	for (int i = 0; i<CH_AD1; i++) {
		paramAD[i].ulChNo = i + 1;
		paramAD[i].ulRange = AD_5V;
	}

}
void AD_BOARD::GetBoard_AD() {
	nRet = AdInputAD(hBoard_AD, CH_AD1, AD_INPUT_SINGLE, paramAD, raw);
	if (nRet != AD_ERROR_SUCCESS) {
		printf("AD11 AdInputAD errr(%lx)", nRet);
		AdClose(hBoard_AD);
		exit(-1);
	}
}
void AD_BOARD::readSensor() {
	int t = 0;
	while (programRunning) {
		//m.lock();
		GetBoard_AD();
		//m.unlock();
		t++;
		Sleep(1);
		//std::cout << t << "\n";
		//Adjust_SamplingTime(t, 800, 0.00125, OFF);
	}
}
void AD_BOARD::readSensorNoThread() {
	GetBoard_AD();
	for (int i = 0; i < CH_AD1; i++) {
		data[i] = float(raw[i]);
	}
}
void AD_BOARD::CloseAD() {
	AdClose(hBoard_AD);
}
double AD_BOARD::getData(int ch, double voltage) {
	double val = (raw[ch]) * voltage / 4095 - 5.0;
	return val;
}
///////////////////////

////EMG_AD board
EMG_AD_BOARD::EMG_AD_BOARD(board_t ID) : m_ID(ID) {};
void EMG_AD_BOARD::SetBoard_AD() {
	hBoard_AD = AdOpen(m_ID);
	if (hBoard_AD == INVALID_HANDLE_VALUE) {
		printf("Failed to open the specified device.");
		exit(-1);
	}
	nRet = AdBmGetSamplingConfig(hBoard_AD, &paramAD);
	if (nRet != AD_ERROR_SUCCESS) {
		printf("AdBmGetSamplingConfig errr(%lx)", nRet);
		AdClose(hBoard_AD);
		exit(-1);
	}
	paramAD.ulChCount = CH_EMG;
	for (int i = 0; i < CH_EMG; i++) {
		paramAD.SmplChReq[i].ulChNo = i + 1;
		paramAd[i].ulChNo = i + 1;
		paramAD.SmplChReq[i].ulRange = AD_10V;
		paramAd[i].ulRange = AD_10V;
	}
	paramAD.ulSmplNum = SMPL_NUM;
	paramAD.fSmplFreq = 1000.0f;
	paramAD.fScanFreq = 100000.0f;

	nRet = AdBmSetSamplingConfig(hBoard_AD, &paramAD);
	if (nRet != AD_ERROR_SUCCESS) {
		printf("AdBmSetSamplingConfig errr(%lx)", nRet);
		AdClose(hBoard_AD);
		exit(-1);
	}
}
void EMG_AD_BOARD::GetBoard_AD()
{
	nRet = AdInputAD(hBoard_AD, CH_EMG, AD_INPUT_SINGLE, paramAd, raw);
	if (nRet != AD_ERROR_SUCCESS) {
		printf("EMG AdInputAD errr(%lx)", nRet);
		AdClose(hBoard_AD);
		exit(-1);
	}
	for (int i = 0; i < CH_EMG; i++) {
		data[i] = float(raw[i]);
	}
}
double EMG_AD_BOARD::getData(int ch, double voltage) {
	double val = (double(data[ch])*20.0 / 65535.0) - 10.0;
	return val;
}
void EMG_AD_BOARD::CloseAD() {
	AdClose(hBoard_AD);
}
//////DA board
DA_BOARD::DA_BOARD(board_t ID) : m_ID(ID) {};
void DA_BOARD::SetBoard_DA() {
	hBoard_DA = DaOpen(m_ID);
	for (int i = 0; i<CH_DA1; i++) {
		paramDA[i].ulChNo = i + 1;
		paramDA[i].ulRange = DA_0_5V;
	}
}
void DA_BOARD::OutBoard_DA() {
	nRet = DaOutputDA(hBoard_DA, CH_DA1, &paramDA[0], Data);
	if (nRet != DA_ERROR_SUCCESS) {
		printf("DaOutputDA errr(%lx)\n", nRet);
		DaClose(hBoard_DA);
		return;
	}
}
void DA_BOARD::CloseDA() {
	DaClose(hBoard_DA);
}
void DA_BOARD::setData(int ch, double val) {
	Data[ch] = static_cast<WORD>(val * 4095 / 5.0);
}
/////////////////