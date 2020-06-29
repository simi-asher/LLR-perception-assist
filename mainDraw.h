/*******************************************************************************
	

		下肢パワーアシストロボットを描画するプログラム
		
		製作者　hora
	　　製作日　2016/11/25
		更新日　2016/11/25

		※環境構築　AntTweakBarのdll,lib,hファイルを所定のフォルダに入れる必要あり
					openGLの拡張機能としてglutを入れる必要あり
********************************************************************************/

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <AntTweakBar.h>
#include <process.h>
#include <Windows.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#pragma comment(lib,"opencv_world343.lib")
#pragma comment(lib,"opencv_world343d.lib")

#include "calculateDraw.h"
#include "setToolBar.h"
#include "Draw.h"
#include "dataSet.h"


#pragma comment(lib,"glew32.lib")
#pragma comment(lib,"glew32s.lib")



int draw_main();
void InitThread(CRITICAL_SECTION *cs, HANDLE *hThread, unsigned __stdcall threadFunc(void *p));
void EndThread(CRITICAL_SECTION *cs, HANDLE *hThread);