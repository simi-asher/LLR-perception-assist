#pragma once
//#include <stdio.h>
//#include <conio.h>
//#include <string>
#include <atlstr.h>

int Init_MPU(HANDLE *file, char *port_name);
int Read_MPU(HANDLE *file, float *Y1,float *R, float *P2,float *Y);//simi removed float *P1,