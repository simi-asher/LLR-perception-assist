/*****************************************************************************************************************************
******************************************************************************************************************************
�}���`�X���b�h�ɕK�v�Ȋ֐����܂Ƃ߂��\�[�X�t�@�C��
���[�U�[�����W�t�@�C���_�̃f�[�^�X�V�╨�̌��o�v���O�����̃}���`�X���b�h���̂��߂Ɏg�p


******************************************************************************************************************************
*****************************************************************************************************************************/
#include "thread.h"



///********************************************************
//�֐����FInitThread
//�����@�F�N���e�B�J���Z�N�V�����̏������ƃX���b�h�̏����ݒ�
//�����@�FCRITICAL_SECTION *cs					 �N���e�B�J���Z�N�V�����ɕK�v�ȍ\���́i�ҏW���͕s�j
//		HANDLE *hThread				             �X���b�h�̃n���h��
//		unsigned __stdcall threadFunc(void *p)	 �X���b�h���ŏ�������֐�
//		
//�o�́@�FHANDLE *hThread
//********************************************************/
//void InitThread(CRITICAL_SECTION *cs, HANDLE *hThread, unsigned __stdcall threadFunc(void *p)){
//
//	InitializeCriticalSection(cs);/* �N���e�B�J���Z�N�V�����������I */
//
//	*hThread = (HANDLE)_beginthreadex(NULL, 0, threadFunc, 0, CREATE_SUSPENDED, NULL);
//	ResumeThread(*hThread);
//
//}
//
//
///********************************************************
//�֐����FInitThread
//�����@�F�N���e�B�J���Z�N�V�����̏������ƃX���b�h�̏����ݒ�
//�����@�FCRITICAL_SECTION *cs					 �N���e�B�J���Z�N�V�����ɕK�v�ȍ\���́i�ҏW���͕s�j
//		HANDLE *hThread				             �X���b�h�̃n���h��
//		
//********************************************************/
//void EndThread(CRITICAL_SECTION *cs, HANDLE *hThread){
//
//	WaitForSingleObject(*hThread,INFINITE);
//	CloseHandle(*hThread);
//   
//    DeleteCriticalSection(cs);/* �N���e�B�J���Z�N�V�����I��� */
//
//}
