#ifndef __CUST_PANTECH_MMP_H__
#define __CUST_PANTECH_MMP_H__

/*
  2011/03/02 권오윤
  PANTECH multimedia 개발 관련 공통 최상위 feature.
  세부 feature를 정의하기 어려운 부분 또는 multimedia관련 소스가 아닌 부분을
  수정하는 경우 사용. (make file 등인 경우 #주석 부분에 추가)
*/
#define FEATURE_PANTECH_MMP

/*
  2011/03/02 권오윤
  VisuialOn VOME engine관련 /external/vome/... 외의 부분을 수정하거나
  VisualOn 소스를 일부 수정해서 사용하는 부분에 대해 feature작업
*/
#define FEATURE_PANTECH_MMP_VOME

/* 
  2011/03/10 최병주
  QualComm Patch 적용한 부분에 대한 frature 작업
*/
#define FEATURE_PANTECH_MMP_QCOM_CR

/* 
  2011/03/11 권오윤
  StageFright 관련 수정하는 부분들을 찾기 쉽게 하기 위해 사용
*/
#define FEATURE_PANTECH_MMP_STAGEFRIGHT
 
/* 
  2011/03/11 권오윤
  OpenCORE 관련 수정하는 부분들을 찾기 쉽게 하기 위해 사용
*/
#define FEATURE_PANTECH_MMP_OPENCORE

/* 
  2011/04/16 이선중
  MP4 QuickTime(ftypqt) 포맷이 SF에서 지원되지 않아 PV로 변경 사용(KakaoTok issue)
*/
#define FEATURE_PANTECH_MMP_QUICKTIME

/*
  2011/05/13 Heekyoung Seo
  Add Qualcomm VC-1 Patch.
  Fis timeStamp order is up so down with a few WMV8 streams.
  */
#define FEATURE_PANTECH_MMP_VC1_DEC_PATCH

/*
  2011/mm/dd who
  ...description...
*/
#define FEATURE_PANTECH_MMP_xxx

/*
  2011/mm/dd who
  ...description...
*/
#define FEATURE_PANTECH_MMP_zzz


#endif/* __CUST_PANTECH_MMP_H__ */
