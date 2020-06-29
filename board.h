#pragma once
#ifndef BOARD_H
#define BOARD_H


#include <windows.h>
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>

#include <FbiAd.h>
#include <FbiDa.h>
#include <fbipenc.h>
#pragma comment(lib,"FbiAd.lib")
#pragma comment(lib,"FbiDa.lib")
#pragma comment(lib,"FbiPenc.lib")
#pragma comment(lib,"FbiAdDC.lib")

#include "Parameter.h"
#include "struct.h"
using namespace std;


#ifdef _WIN64					//X64
typedef LPCSTR board_t;
#else							//X86
typedef LPCTSTR board_t;
#endif

#define USE_ENC_BOARD
#define ENC_ID (board_t)"FBIPENC17"	//encoders
#define CH_ENC (8)

#define USE_AD1_BOARD
//#define USE_MULTHREAD
#define AD1_ID (board_t)"FBIAD1"	//foot sensors, force sensors
#define CH_AD1 (22)

#define USE_EMG_BOARD
#define EMG_ID (board_t)"FBIAD2"	//EMG
#define CH_EMG (8)
#define SMPL_NUM (1)

#define USE_DA1_BOARD
#define DA1_ID (board_t)"FBIDA1"	//LEFT leg motors
#define CH_DA1 (6)

#define USE_DA2_BOARD
#define DA2_ID (board_t)"FBIDA2"	//RIGHT leg motors
#define CH_DA2 (6)

#ifndef M_PI
#define M_PI (3.1415926535)
#endif


class ENC_BOARD {
private:
	HANDLE		hBoard_ENC;		// Device handle
	float		*data = new float[CH_ENC];	// Output data storage area
	unsigned long   raw[CH_ENC];
	int         ChNo[CH_ENC];
	int         mode[CH_ENC];
	board_t		m_ID;
	int			nRet;
public:
	ENC_BOARD(board_t ID);
	void SetBoard_ENC();
	void GetBoard_ENC();
	void CloseENC();
	double getData(int ch, int coef);
};

class AD_BOARD {
private:
	int			nRet;
	HANDLE		hBoard_AD;		// Device handle
	float       *data = new float[CH_AD1];	// Output data storage area
	WORD		*raw = new WORD[CH_AD1];
	ADSMPLCHREQ	paramAD[CH_AD1];		// Output conditions setting structure
	board_t		m_ID;
	int			programRunning = ON;
	thread		*thr = new thread[1];
	mutex m;
public:
	AD_BOARD(board_t ID);
	void SetBoard_AD();
	void GetBoard_AD();
	void CloseAD();
	double getData(int ch, double voltage);
	void readSensor();
	void readSensorNoThread();
};

class DA_BOARD {
private:
	int			nRet;
	HANDLE		hBoard_DA;		// Device handle
	WORD        *Data = new WORD[CH_DA1];	// Output data storage area
	DASMPLCHREQ	paramDA[CH_DA1];		// Output conditions setting structure
	board_t		m_ID;
public:
	DA_BOARD(board_t ID);
	void SetBoard_DA();
	void OutBoard_DA();
	void CloseDA();
	void setData(int ch, double val);
};

class EMG_AD_BOARD {
private:
	int			nRet;
	HANDLE		hBoard_AD;		// Device handle
	float       *data = new float[CH_EMG];	// Output data storage area
	float		*pre_data = new float[CH_EMG];	// previous Output data storage area
	ADBMSMPLREQ	paramAD;		// Output conditions setting structure
	ADSMPLCHREQ	paramAd[CH_EMG];
	board_t		m_ID;
	WORD        raw[CH_EMG];
public:
	EMG_AD_BOARD(board_t ID);
	void SetBoard_AD();
	void GetBoard_AD();
	double getData(int ch, double voltage);
	void CloseAD();
};

class BOARD {

public:
	ENC_BOARD			ENC{ ENC_ID };
	AD_BOARD			AD1{ AD1_ID };
	EMG_AD_BOARD			AD2{ EMG_ID };
	DA_BOARD			DA1{ DA1_ID };
	DA_BOARD			DA2{ DA2_ID };
	void SetAll();
	//void ReadAllSenosrs(Robot *robot);
	void motorOutput(int judge, const double torq[2][3], const double rotation[2][3]);
	void CloseAll();
};
#endif // !BOARD_H