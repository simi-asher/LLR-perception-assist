/*******************************************************************************
	

		�����p���[�A�V�X�g���{�b�g��`�悷��v���O����
		
		����ҁ@hora
	�@�@������@2016/11/25
		�X�V���@2016/11/25

		�����\�z�@AntTweakBar��dll,lib,h�t�@�C��������̃t�H���_�ɓ����K�v����
					openGL�̊g���@�\�Ƃ���glut������K�v����
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