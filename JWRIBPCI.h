#ifndef	__JWRIBPCI_H__
#define	__JWRIBPCI_H__

#include <windows.h>

#ifdef ROBOTDLL_EXPORTS
#define ROBOTDLL_API __declspec(dllexport)
#else
#define ROBOTDLL_API __declspec(dllimport)
#endif

#ifndef	_RIBVXDCALLBACK_DEFINED
#define	_RIBVXDCALLBACK_DEFINED
typedef int (RIBVXDCALLBACK)(WORD wIntNo, BOOL* bEnableMessage, void* pUserData, DWORD nUserDataSize);
#endif	//_RIBVXDCALLBACK_DEFINED

extern "C"{
ROBOTDLL_API int WINAPI GetRegAdrEx(int iRibNo);
ROBOTDLL_API int WINAPI GetIrqNoEx(int iRibNo);
ROBOTDLL_API int WINAPI GetRibNoEx(int RibID);
ROBOTDLL_API int WINAPI GetRibIDEx(int iRibNo);

ROBOTDLL_API int WINAPI SetWndMsgEx(int iRibNo, HANDLE hwnd, UINT uiMsg);
ROBOTDLL_API int WINAPI EnableIntMsgEx(int iRibNo, BOOL bEnable);
ROBOTDLL_API int WINAPI SetVxdCallBackEx(int iRibNo, RIBVXDCALLBACK* pUserProc, DWORD dwUserProcSize, PVOID pUserData, DWORD dwUserDataSize, DWORD* pdwProcAdrRet, DWORD* pdwDataAdrRet);
//ROBOTDLL_API int WINAPI SetVxdCallBackEx2(int iRibNo, RIBVXDCALLBACK* pUserProc, DWORD dwUserProcSize, void* pUserData, DWORD dwUserDataSize, DWORD* pdwProcAdrRet = NULL, DWORD* pdwDataAdrRet = NULL);
ROBOTDLL_API int WINAPI SetVxdCallBackEx2(int iRibNo, RIBVXDCALLBACK* pUserProc, DWORD dwUserProcSize, PVOID pUserData, DWORD dwUserDataSize, DWORD* pdwProcAdrRet, DWORD* pdwDataAdrRet);

ROBOTDLL_API BOOL WINAPI SendCmdEx(int iRibNo, int iChNo, WORD wCmd);
ROBOTDLL_API BOOL WINAPI SendCmdDataEx(int iRibNo, int iChNo, WORD wCmd, WORD wData);
ROBOTDLL_API BOOL WINAPI SendCmdDWDataEx(int iRibNo, int iChNo, WORD wCmd, DWORD dwData);
ROBOTDLL_API BOOL WINAPI ReadDataEx(int iRibNo, WORD *pwData, int nNum);
ROBOTDLL_API BOOL WINAPI ReadDWDataEx(int iRibNo, DWORD *pdwData);		//!!


}//extern "C"

#endif	//__JWRIBPCI_H__
