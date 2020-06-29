/*****************************************************************************************************************************
******************************************************************************************************************************

�j���[�����l�b�g�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��


******************************************************************************************************************************
*****************************************************************************************************************************/

#include "NeuralNetwork.h"


/********************************************************
�֐����FNeuralNet
�����@�F�j���[�����l�b�g���[�N�̌v�Z
�����@�Fdouble *input			���͑w�̏o�͒l
		_weights *weights 		���͑w�Əo�͑w�̊Ԃ̃E�G�C�g�l
		int funcMode			�o�͑w�Ōv�Z����֐��̐ݒ�
		const int layer1_num	���͑w�̐�
		const int layer2_num	�o�͑w�̐�
		double *output			�o�͑w�̏o�͒l
		int func                �������֐��w��

�o�́@�Fdouble *output
********************************************************/
void NeuralNet(double *input, _weights *weights, int num, const int layer1_num, const int layer2_num, double *output, double *disOutput, int func){
	
	double *val = new double[layer2_num];
	for(int i=0; i<layer2_num; i++){
		val[i] = 0;
	}
	/*if(num==LAYER_NUM-2){
		for(int i=0; i<layer1_num; i++){
			printf("%10.8lf\t",input[i]);
		}
		printf("\n");
	}*/


	for(int j=0; j<layer2_num; j++){
		for(int i=0; i<layer1_num; i++){
			val[j] += (weights[num].layer[i][j] * input[i]);
		}
		val[j] += weights[num].bias[j]*1.0;
		//if(num==LAYER_NUM-2) printf("%10.8lf\t",val[j]);
	}
	//if(num==LAYER_NUM-2)	printf("\n");	


	for(int j=0; j<layer2_num; j++){
		if(func==0) output[j] = Sigmoid_Func(val[j]);              //sigmoid
		if(func==1) output[j] = Tanh_Func(val[j]);                       //tanh 
		if(func==2) output[j] = ReLU(val[j], 1.0);                       //ReLU
		//if(func=3) output[j] = Sigmoid_Func(val[j], 0.0, 0.5);             //leakly_ReLU
		if(func==4) output[j] = Softmax(val,layer2_num,j);                //Softmax
		//if(func=5) output[j] = Linear(val[j], 0.0, 0.5);               //Linear
		//if(num==LAYER_NUM-2) printf("%10.8lf\t",output[j]);
	}
	//if(num==LAYER_NUM-2)	printf("\n");
		
	for(int j=0; j<layer2_num; j++){
		if(func==0) disOutput[j] = dif_Sig_Func(val[j]);              //sigmoid
		if(func==1) disOutput[j] = dif_Tanh_Func(val[j]);                //tanh 
		if(func==2) disOutput[j] = dif_ReLU(val[j], 1.0);                       //ReLU
		//if(func=3) output[j] = Sigmoid_Func(val[j], 0.0, 0.5);            //leakly_ReLU
		if(func==4) disOutput[j] = dif_Softmax(val,layer2_num,j);               //Softmax
		//if(func=5) output[j] = Linear(val[j], 0.0, 0.5);              //Linear
		//printf("Tanh_Func=%lf\n",Tanh_Func(val[j]) );
		//printf("dif_Tanh_Func=%lf\n",dif_Tanh_Func(val[j]) );
		//printf("disOutput[%d] = %lf\n",j,disOutput[j]);
	}


	delete[] val;

}


/********************************************************
�֐����FTest_NeuralNet
�����@�F�j���[�����l�b�g���[�N�̌v�Z
�����@�Fdouble *input			���͑w�̏o�͒l
		_weights *weights 		���͑w�Əo�͑w�̊Ԃ̃E�G�C�g�l
		int funcMode			�o�͑w�Ōv�Z����֐��̐ݒ�
		const int layer1_num	���͑w�̐�
		const int layer2_num	�o�͑w�̐�
		double *output			�o�͑w�̏o�͒l
		int func                �������֐��w��

�o�́@�Fdouble *output
********************************************************/
void Test_NeuralNet(double *input, _weights *weights, int num, const int layer1_num, const int layer2_num, double *output, int func){
	
	double *val = new double[layer2_num];
	for(int i=0; i<layer2_num; i++){
		val[i] = 0;
	}
	
	//printf("val\n");
	for(int j=0; j<layer2_num; j++){
		for(int i=0; i<layer1_num; i++){
			val[j] += (weights[num].layer[i][j] * input[i]);
		}
		val[j] += weights[num].bias[j]*1.0;
		//printf("%+6.7lf\t", val[j]);
	}
	//printf("\n");
	for(int j=0; j<layer2_num; j++){
		if(func==0) output[j] = Sigmoid_Func(val[j]);              //sigmoid
		if(func==1) output[j] = Tanh_Func(val[j]);                       //tanh 
		if(func==2) output[j] = ReLU(val[j], 1.0);                       //ReLU
		//if(func=3) output[j] = Sigmoid_Func(val[j], 0.0, 0.5);             //leakly_ReLU
		if(func==4) output[j] = Softmax(val,layer2_num,j);                //Softmax
		//if(func=5) output[j] = Linear(val[j], 0.0, 0.5);               //Linear
	}

		
	delete[] val;

}



/********************************************************
�֐����FCalculateError
�����@�F�ڕW�l�Ƃ̌덷�̌v�Z
�����@�Fconst int OUTPUT_NUM	�o�͑w�̐�
		double *layer			�o�͑w�̏o�͒l
		double *output			�ڕW�l
		double *error			�ڕW�l�Ƃ̍�
		
�o�́@�Fdouble *error
********************************************************/
void CalculateError(const int OUTPUT_NUM, double *layer, double *teach, double *error){
	
	for(int i=0; i<OUTPUT_NUM; i++){
		error[i] = teach[i] - layer[i];
	}

}



/********************************************************
�֐����FBP
�����@�F���̑��̑w�̃E�G�C�g�l�̊w�K
�����@�Fint num					���݂̑w�̔ԍ�
		int *NEURON_NUM			�e�w�̐�
		double error_sum		�ڕW�l�Ƃ̍��̍��v
		double **disLayer		�e�w�̏o�͒l�̔���
		double **layer			�e�w�̏o�͒l
		_weights *weights			�e�w�̊Ԃ̃E�G�C�g�l

�o�́@�F_weights *weights
********************************************************/
void BP(int num, int layer_num, int *NEURON_NUM, double *error, double **disLayer, double **layer, _weights *weights,double ***keep0,double ****keep1){
	
	

	//5�w��backpropagation
	if((layer_num-num)==5){
		for(int i=0;i<NEURON_NUM[num];i++){
			for(int j=0;j<NEURON_NUM[num+1];j++){
				for(int k=0;k<NEURON_NUM[num+2];k++){
					for(int l=0;l<NEURON_NUM[num+3];l++){
						for(int m=0;m<NEURON_NUM[num+4];m++){
							weights[num].delta[i][j] -= error[m] * layer[num][i]  * weights[num+1].layer[j][k] * disLayer[num+1][j] * disLayer[num+2][k] * weights[num+2].layer[k][l] * disLayer[num+3][l] * weights[num+3].layer[l][m];
							weights[num].delta_bias[j] -= error[m] * 1.0  * weights[num+1].layer[j][k] * disLayer[num+1][j] * disLayer[num+2][k] * weights[num+2].layer[k][l] * disLayer[num+3][l] * weights[num+3].layer[l][m];
						}
					}
				}
				//printf("weightsdelta=%lf\n",weights[num].delta[i][j]);
			}
		}
	}

	//4�w��backpropagation
	if((layer_num-num)==4){
		for(int i=0;i<NEURON_NUM[num];i++){
			for(int j=0;j<NEURON_NUM[num+1];j++){
				for(int k=0;k<NEURON_NUM[num+2];k++){
					for(int l=0;l<NEURON_NUM[num+3];l++){
						weights[num].delta[i][j] -= error[l] * layer[num][i]  * weights[num+1].layer[j][k] * disLayer[num+1][j] * disLayer[num+2][k] * weights[num+2].layer[k][l];
						weights[num].delta_bias[j] -= error[l] * 1.0  * weights[num+1].layer[j][k] * disLayer[num+1][j] * disLayer[num+2][k] * weights[num+2].layer[k][l];
					}
				}
			}
		}
	}

	//3�w��backpropagation
	if((layer_num-num)==3){
		for(int i=0;i<NEURON_NUM[num];i++){
			for(int j=0;j<NEURON_NUM[num+1];j++){
				for(int k=0;k<NEURON_NUM[num+2];k++){
					weights[num].delta[i][j] -= error[k] * layer[num][i]  * weights[num+1].layer[j][k] * disLayer[num+1][j];
					weights[num].delta_bias[j] -= error[k] * 1.0  * weights[num+1].layer[j][k] * disLayer[num+1][j];
				}
			}
		}
	}

	//2�w��backpropagation
	if((layer_num-num)==2){
		for(int i=0;i<NEURON_NUM[num];i++){
			for(int j=0;j<NEURON_NUM[num+1];j++){
				weights[num].delta[i][j] -= error[j] * layer[num][i];
				weights[num].delta_bias[j] -= error[j] * 1.0;
			}
		}
	}

	


	/*for(a=0;a<NEURON_NUM[num];a++){

		for(b=0;b<NEURON_NUM[num+1];b++){
		if((layer_num-num)>2){

			for(c=0;c<NEURON_NUM[num+2];c++){
			if((layer_num-num)>3){


				for(d=0;d<NEURON_NUM[num+3];d++){
				if((layer_num-num)>4){


					for(e=0;e<NEURON_NUM[num+4];e++){
					if((layer_num-num)>5){

						for(f=0;f<NEURON_NUM[num+5];f++){
						if((layer_num-num)>6){

							for(g=0;g<NEURON_NUM[num+6];g++){
							if((layer_num-num)>7){
								
								for(h=0;h<NEURON_NUM[num+7];h++){
								if((layer_num-num)>8){

									for(i=0;i<NEURON_NUM[num+8];i++){
									if((layer_num-num)>9){

										for(j=0;j<NEURON_NUM[num+9];j++){
										if((layer_num-num)>10){
										delta[9] += error_sum*weights[num+8].layer[i][j];
										}
										}

									delta[8] += delta[9] * disLayer[num+8][i]*weights[num+7].layer[h][i];
									}
									else delta[8] += error_sum*weights[num+7].layer[h][i];
									}

								delta[7] += delta[8] * disLayer[num+7][h]*weights[num+6].layer[g][h];	
								}
								else delta[7] += error_sum*weights[num+6].layer[g][h];
								}
							
							delta[6] += delta[7] * disLayer[num+6][g]*weights[num+5].layer[f][g];
							}
							else delta[6] += error_sum*weights[num+5].layer[f][g];
							}

						delta[5] += delta[6] * disLayer[num+5][f]*weights[num+4].layer[e][f];
						}
						else delta[5] += error_sum*weights[num+4].layer[e][f];
						}

					delta[4] += delta[5] * disLayer[num+4][e]*weights[num+3].layer[d][e];
					}
					else delta[4] += error_sum*weights[num+3].layer[d][e];
					}

				delta[3] += delta[4] * disLayer[num+3][d]*weights[num+2].layer[c][d];
				}
				else delta[3] += error_sum*weights[num+2].layer[c][d];
				printf(" disLayer[%d][%d] = %lf\n",num+3,d , disLayer[num+3][d]);
				}
				printf("delta[3]=%lf\n",delta[3]);
			delta[2] += delta[3] * disLayer[num+2][c]*weights[num+1].layer[b][c];
			}
			else delta[2] += error_sum*weights[num+1].layer[b][c];
			printf(" disLayer[%d][%d] = %lf\n",num+2,c , disLayer[num+2][c]);
			}
        delta[1] = delta[2] * disLayer[num+1][b] * layer[num][a] ;
		delta[0] = delta[2] * disLayer[num+1][b] * 1.0 ;
		}else{
			delta[1] = error_sum * disLayer[num+1][b] * layer[num][a];
			delta[0] = error_sum * disLayer[num+1][b] * 1.0 ;
		}
		printf(" disLayer[%d][%d] = %lf\n",num+1,b , disLayer[num+1][b]);
		keep0[0][num][b] = delta[0];
		keep1[0][num][a][b] = delta[1];
		printf(" delta[1] = %20.17lf\n", delta[1]);
		weights[num].layer[a][b] += ETA2 * delta[1] - ETA2 * 0.001 * weights[num].layer[a][b] + 0.7 * keep1[1][num][a][b];
		weights[num].bias[b] += ETA2 * delta[0] + 0.7 * keep0[1][num][b];
		keep0[1][num][b] = keep0[0][num][b];
		keep1[1][num][a][b] = keep1[0][num][a][b];
		}	
	}
	*/

}


void BP_weights_Change(int layer_num, int *NEURON_NUM,  _weights *weights,int q){
	for(int i=0;i<layer_num-1;i++){
		for(int j=0;j<NEURON_NUM[i];j++){
			for(int k=0;k<NEURON_NUM[i+1];k++){
				weights[i].layer[j][k] += ETA * exp(-((double)(q/1000000))) * weights[i].delta[j][k] ;
				weights[i].delta[j][k] = 0;
			}
		}
	}

	for(int i=0;i<layer_num-1;i++){
		for(int k=0;k<NEURON_NUM[i+1];k++){
			weights[i].bias[k] += ETA * exp(-((double)(q/1000000))) * weights[i].delta_bias[k] ;
			weights[i].delta_bias[k] = 0;
		}
	}


}



/********************************************************
�֐����FSigmoid_Func
�����@�F�V�O���C�h�֐���p�����v�Z
�����@�Fdouble input		����
		double w_o			�֐��𕽍s�ړ�������ϐ�
		double w_i			�֐��̌��z��ω�������ϐ�
		double w_o Variable that translates the function
		double w_i variable that changes the slope of the function

�߂�l�Fdouble	( 1.0/( 1.0+exp( -theta*w_i-w_o ) ) );
********************************************************/
double Sigmoid_Func( double input ){

	return ( 1.0/( 1.0+exp( -input) ) );
}


/********************************************************
�֐����Fdif_Sig_Func
�����@�F�V�O���C�h�֐��̔������v�Z
�����@�Fdouble input		����
		double w_o			�֐��𕽍s�ړ�������ϐ�
		double w_i			�֐��̌��z��ω�������ϐ�

�߂�l�Fdouble	( 1.0/( 1.0+exp( -theta*w_i-w_o ) ) );
********************************************************/
double dif_Sig_Func(double input){

	return(exp(-input)/((1.0+exp( -input))* (1.0 + exp(-input))));
}



/********************************************************
�֐����FTanh_Func
�����@�F�n�C�p�{���b�N�^���W�F���g���v�Z
�����@�Fdouble input		����
		

�߂�l�Fdouble	(LU);
********************************************************/
double Tanh_Func( double input ){
	//printf("exp(input)+exp(-input)=%lf\n",exp(input)+exp(-input));
	return ( (exp(input)-exp(-input))/(exp(input)+exp(-input)) );
}

/********************************************************
�֐����Fdif_Tanh_Func
�����@�F�n�C�p�{���b�N�^���W�F���g�̔������v�Z
�����@�Fdouble input		����
		

�߂�l�Fdouble	(LU);
********************************************************/
double dif_Tanh_Func( double input ){
	//printf("dif = %lf",4/((exp(input*0.05)+exp(-input*0.05))*(exp(input*0.05)+exp(-input*0.05))));
	return ( 4/((exp(input)+exp(-input))*(exp(input)+exp(-input))) );
}




/********************************************************
�֐����FReLU
�����@�F0�ȏ�Œ����Ȋ֐����v�Z
�����@�Fdouble input		����
		double slope		�֐��̌X����ω�������ϐ�

�߂�l�Fdouble	(LU);
********************************************************/

double ReLU(double input, double slope){  //slope�ˌX��
	double LU;
	if (input < 0.0) LU = 0; //for prelu: slope* input * 0.5;
	else LU = slope * input;
	return(LU);
}


/********************************************************
�֐����Fdif_ReLU
�����@�F0�ȏ�Œ����Ȋ֐��̔������v�Z
�����@�Fdouble input		����
		double slope		�֐��̌X����ω�������ϐ�

�߂�l�Fdouble	(LU);
********************************************************/

double dif_ReLU(double input, double slope){  //slope�ˌX��
	double dif_LU;
	if (input < 0.0) dif_LU = 0; //for prelu slope * 0.5;
	else dif_LU = slope;
	return(dif_LU);
}





/********************************************************
�֐����FSoftmax
�����@�FSoftmax function���v�Z
�����@�Fdouble input		����
		double slope		�֐��̌X����ω�������ϐ�

�߂�l�Fdouble	(LU);
********************************************************/

double Softmax(double *val,double layer_num, int j){
	double total = 0;
	for(int i=0;i<layer_num;i++){
		total += exp(val[i]);
	}

	return(exp(val[j])/total);
}


/********************************************************
�֐����Fdif_Softmax
�����@�FSoftmax function�̔������v�Z
�����@�Fdouble input		����
		double slope		�֐��̌X����ω�������ϐ�

�߂�l�Fdouble	(LU);
********************************************************/

double dif_Softmax(double *val,double layer_num, int j){
	double total = 0;
	for(int i=0;i<layer_num;i++){
		total += exp(val[i]);
	}

	return(((total-exp(val[j]))*exp(val[j]))/(total*total));
}