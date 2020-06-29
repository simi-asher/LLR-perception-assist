/*****************************************************************************************************************************
******************************************************************************************************************************
フィルターに必要な関数をまとめたソースファイル
力センサのフィルタリングなどに使用


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "myFilter.h"
#include <math.h>

/********************************************************
関数名：Low_Pass_Filter
説明　：ローパスフィルタの設計
		バタワースフィルタに使用
引数　：double signal[]				ボードのハンドル
		double signal_f[]
		
戻り値：Sum_Signal-Sum_Signal_f
********************************************************/
double Low_Pass_Filter( double signal[], double signal_f[] ){

	double cut_frec[2][FILTER_LEN];
	double Sum_Signal;
	double Sum_Signal_f;

	// sampling frec = 2000Hz, cutoff freq = 2Hz, order = 2 //
	cut_frec[1][0] =  0.246193004643791;
	cut_frec[1][1] =  0.492386009287582;
	cut_frec[1][2] =  0.246193004643791;
	cut_frec[0][0] =  1.0;
	cut_frec[0][1] = -1.99555712434579;
	cut_frec[0][2] =  0.995566972065975;
	for(int i=0 ; i<FILTER_LEN ; i++ ) cut_frec[1][i] *= 0.00001;

	//// sampling frec = 5Hz, cutoff freq = 2Hz, order = 2 //
	//cut_frec[1][0] =  0.6389;
	//cut_frec[1][1] =  1.2779;
	//cut_frec[1][2] =  0.6389;
	//cut_frec[0][0] =  1.0;
	//cut_frec[0][1] =  1.1430;
	//cut_frec[0][2] =  0.4128;
	////for(int i=0 ; i<FILTER_LEN ; i++ ) cut_frec[1][i] *= 0.00001;


	Sum_Signal   = 0.0;		Sum_Signal_f = 0.0;
	for(int i=0 ; i<FILTER_LEN ; i++ ) Sum_Signal   += ( cut_frec[1][i]*signal[i]   );
	for(int i=1 ; i<FILTER_LEN ; i++ ) Sum_Signal_f += ( cut_frec[0][i]*signal_f[i] );


	return ( Sum_Signal-Sum_Signal_f );
}

/********************************************************
関数名：LowPassFilter (from ou san)
説明　：ローパスフィルタの設計
		バタワースフィルタに使用
引数　：
		

戻り値：float buf[1][0]
********************************************************/
//このサイトを参照http://vstcpp.wpblog.jp/?page_id=523
float LowPassFilter(float samplefreq, float cutfreq, float input, float buf[2][FILTER_LEN]) {

	//bufの0行目には入力、bufの1行目には出力がため込まれる
	//0列目には現在、1列目には1つ前、2列目には2つ前の値が入る
	//float output;

	//サンプリング周波数 //cutfreqはカットオフ周波数 The cut-off frequency is the frequency at which no further frequencies pass
	double q = 1.0f / 1.41421356f;	//Q値 1/root(2)
	float omega = 2.0f * 3.14159265f * cutfreq / samplefreq;
	float alpha = sin(omega) / (2.0f * q);

	float a0 = 1.0f + alpha;
	float a1 = -2.0f * cos(omega);
	float a2 = 1.0f - alpha;
	float b0 = (1.0f - cos(omega)) / 2.0f;
	float b1 = 1.0f - cos(omega);
	float b2 = (1.0f - cos(omega)) / 2.0f;
	//printf("%f\n", sin(omega));
	//printf("omega=%f, alpha=%f, a0=%f a1=%f a2=%f b0=%f b1=%f b2=%f\n", omega, alpha, a0, a1, a2, b0, b1, b2);
	// Initialize Input-Buffer for Filtering Signals //
	//if (count == 0) for (int i = 0; i<FILTER_LEN; i++)for (int j = 0; j < 2; j++) buf[j][i] = 0;

	buf[1][0] = b0 / a0 * input + b1 / a0 * buf[0][1] + b2 / a0 * buf[0][2] - a1 / a0 * buf[1][1] - a2 / a0 * buf[1][2];

	//Update buffer
	buf[0][2] = buf[0][1];
	buf[0][1] = input;
	buf[1][2] = buf[1][1];
	buf[1][1] = buf[1][0];
	// change Buffer for Filtering Signals //
	//for (int i = FILTER_LEN - 1; i>0; i--) for (int j = 0; j<2; j++) buf[j][i] = buf[j][i - 1];

	return buf[1][0];
}

/********************************************************
関数名：Low_Pass_Filter
説明　：ローパスフィルタの設計
		バタワースフィルタに使用
引数　：double signal[]				ボードのハンドル
		double signal_f[]
		
戻り値：Sum_Signal-Sum_Signal_f
********************************************************/
double Filtering( int count, double input, double data[2][FILTER_LEN]){

	double filter_data[2][FILTER_LEN];


	// Initialize Input-Buffer for Filtering Signals //
	if( count==0 ){
		for(int i=0 ; i<FILTER_LEN ; i++ ){
			for(int j=0 ; j<2 ; j++ ) data[j][i] = input;
		}
	}

	data[0][0] = input;

	for(int i=0 ; i<FILTER_LEN ; i++ ){
		for(int j=0 ; j<2 ; j++ ) filter_data[j][i] = data[j][i];
	}

	filter_data[1][0] = Low_Pass_Filter( filter_data[0], filter_data[1] );

	for(int i=0 ; i<FILTER_LEN ; i++ ){
		for(int j=0 ; j<2 ; j++ ) data[j][i] = filter_data[j][i];
	}

	// change Buffer for Filtering Signals //
	for(int i=FILTER_LEN-1 ; i>0 ; i-- ){
		for(int j=0 ; j<2 ; j++ ) data[j][i] = data[j][i-1];
	}


	return data[1][0];
}