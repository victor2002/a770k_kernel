#ifndef __CUST_SKYCAM_H__
#define __CUST_SKYCAM_H__

/*
Modified files which have "SKYCAM" tag:
	android/build/core/Makefile
	android/kernel/arch/arm/configs/qsd8650_defconfig
	android/kernel/drivers/media/video/msm/Kconfig
	android/kernel/drivers/media/video/msm/Makefile
	android/vendor/qcom/android-open/libcamera2/Android.mk
	android/vendor/qcom-proprietary/mm-camera/Android.mk
	android/vendor/qcom-proprietary/mm-camera/camera.mk
	android/vendor/qcom-proprietary/mm-camera/targets/Android.mk

Modified files which don't have "SKYCAM" tag:
	android/kernel/drivers/media/video/msm/mt9p111.h
	android/kernel/drivers/media/video/msm/mt9p111_reg.c
	android/vendor/qcom-proprietary/mm-camera/targets/tgtcommon/sensor/mt9p111/mt9p111.h
	android/kernel/drivers/media/video/msm/yacbac1sddas.h
	android/kernel/drivers/media/video/msm/yacbac1sddas_reg.c
	android/vendor/qcom-proprietary/mm-camera/targets/tgtcommon/sensor/yacbac1sddas/yacbac1sddas.h

Local features:
	CONFIG_MSM_CAMERA_DEBUG (MSM_CAMERA_DEBUG)
	CONFIG_SKYCAM_MT9P111 (SKYCAM_MT9P111)
	F_SKYCAM_SENSOR_MT9P111
	CONFIG_SKYCAM_YACBAC1SDDAS (SKYCAM_YACBAC1SDDAS)
	F_SKYCAM_SENSOR_YACBAC1SDDAS
	F_SKYCAM_LOG_VERBOSE (enable LOGV/LOGD/LOGI in userspace)
	F_SKYCAM_LOG_CDBG (enable CDBG in userspace)
	F_SKYCAM_LOG_OEM (enable SKYCDBG/SKYCERR in userspace)

How to turn off all camera logs (kernel/userspace):
	android/kernel/arch/arm/config/qsd8650-defconfig
		- CONFIG_MSM_CAMERA_DEBUG -> no (default)
	Disable F_SKYCAM_LOG_PRINTK (default enabled)
	Find all F_SKYCAM_LOG_OEM, 
		- comment out (default)
	Find all F_SKYCAM_LOG_CDBG
		- comment out (default)
	Find all F_SKYCAM_LOG_VERBOSE
		- comment out (default)

How to exclude module "MT9P111":
	android/kernel/arch/arm/config/msm7630_defconfig
		- disable CONFIG_MT9P111
	android/vendor/qcom-proprietary/mm-camera/Android.mk
	android/vendor/qcom-proprietary/mm-camera/camera.mk
		- CONFIG_MT9P111=no (default yes)

How to exclude module "OLAWORKS":



(2)  카메라 관련 모든 kernel/userspace/android 로그를 제거

kernel/arch/arm/config/qsd8650-perf_defconfig 를 수정한다.

	# CONFIG_MSM_CAMERA_DEBUG is not set (default)

CUST_SKYCAM.h 에서 F_SKYCAM_LOG_PRINTK 을 #undef 한다. 

	#define F_SKYCAM_LOG_PRINTK (default)

모든 소스 파일에서 F_SKYCAM_LOG_OEM 을 찾아 주석 처리한다. 
	선언 된 경우, 해당 파일에 사용된 SKYCDBG/SKYCERR 매크로를 이용한 
	메시지들을 활성화 시킨다. (user-space only)

모든 소스 파일에서 F_SKYCAM_LOG_CDBG 를 찾아 주석 처리한다. 
	선언 된 경우, 해당 파일에 사용된 CDBG 매크로를 이용한 메시지들을 
	활성화 시킨다. (user-space only)

모든 소스 파일에서 F_SKYCAM_LOG_VERBOSE 를 찾아 주석 처리한다.
	선언 된 경우, 해당 파일에 사용된 LOGV/LOGD/LOGI 매크로를 이용한 
	메시지들을 활성화 시킨다. (user-space only)


(4)  안면인식 관련 기능 빌드 환경

vendor/qcom/android-open/libcamera2/Android.mk 를 수정한다.
	3rd PARTY 솔루션 사용 여부를 결정한다.

	SKYCAM_FD_ENGINE := 0		미포함
	SKYCAM_FD_ENGINE := 1		올라웍스 솔루션 사용
	SKYCAM_FD_ENGINE := 2		기타 솔루션 사용

CUST_SKYCAM.h 에서 F_SKYCAM_ADD_CFG_FACE_FILTER 를 #undef 한다.
	안면인식 기능 관련 인터페이스 포함 여부를 결정한다.

libOlaEngine.so 를 기존 libcamera.so 에 링크하므로 기존 대비 libcamera.so 의
크기가 증가하여 링크 오류가 발생 가능하며, 이 경우 아래 파일들에서 
liboemcamera.so 의 영역을 줄여 libcamera.so 의 영역을 확보할 수 있다.

build/core/prelink-linux-arm-2G.map (for 2G/2G)
build/core/prelink-linux-arm.map (for 1G/3G)	

*/	


/* 내수 CS 부서에서는 소비자 시료 분석을 위해 별도 PC 프로그램을 사용하여 
 * 카메라 구동 시간 정보를 PC 로 추출한다. 
 *
 * 구현 방법은 공정 커맨드 사양서에 명시되어 있으므로 관련 코드들은 공정 커맨드 
 * 관련 모듈에 포함되어 있으나, 공정 커맨드 용 PC 프로그램을 사용하지 않고 별도
 * 프로그램을 사용하여, 시료의 DIAG 포트로부터 구동 시간 정보를 확인할 수 있다.
 *
 * 공정 커맨드 사양서 v10.35 기반 구현
 * PhoneInfoDisplay v4.0 프로그램으로 확인
 * 사양서와 프로그램은 DS2팀 박경호 선임에게 문의 */
#define F_SKYCAM_FACTORY_PROC_CMD

#define F_SKYCAM_LOG_DEBUG

/*----------------------------------------------------------------------------*/
/*  MODEL-SPECIFIC                                                            */
/*  EF18K 에만 적용되는 또는 EF13S 에서만 검증된 FEATURE 목록                 */
/*----------------------------------------------------------------------------*/
#if (MODEL_ID == MODEL_EF33S)
#define F_SKYCAM_TARGET_EF33S
#endif
#if (MODEL_ID == MODEL_EF34K)
#define F_SKYCAM_TARGET_EF34K
#endif
#if (MODEL_ID == MODEL_EF35L)
#define F_SKYCAM_TARGET_EF35L
#endif
#if (MODEL_ID == MODEL_EF40K)/*yjw*/
#define F_SKYCAM_TARGET_EF40K
#endif


/*  듀얼 카메라 작업을 위해서 추가된 피쳐
 * 전후면 카메라를 내장한 안드로이드 단말에서 카메라 전환을 위해 작업된 부분
 * 새로운 API 추가하여 CamID를 전달 할 수 있음
 */
#define F_SKYCAM_ADD_DUAL_CAM



/* SKYCAM Lee Jitae 2010-09-25
 * VPE의사용으로 인해서 비디오 프레임이 올라오지 않는 문제점이 발생
 * mDisEnabled의 미사용으로 문제 해결 
 */
 #define F_SKYCAM_VIDEO_FRAME_VPE



#define F_SKYCAM_LOG_PRINTK	//커널 영역의 SKYCDBG/SKYCERR 메세지 on off
#define F_SKYCAM_LOG_OEM		//유저 영역의 SKYCDBG/SKYCERR 메세지 on off

/* MSM7230 is used VFE31 
   QSD8250 is used VFE41 
   Sensor Definition 
 */



/* 다운 사이즈를 하기 위한 함수를 사용하기 위해서,  캡쳐 화면을 보여주는 곳과 캡쳐 화면이  틀린 것을 수정하기 위해서
 * 썸네일을 가지고 스냅샷프리뷰를 만든다.
 * IPL 라이브러리에 있는 downsize를 이용해서 줄인다.
 * 현재 SR진행 중 -> 퀄컴 패치에 적용 될 수 있도록 수정 받기 위해서 진행 중
 */

/* #define F_SKYCAM_SNAPSHOT_PREVIEW  */


/* SKYCAM_PSJ_100513
 * 전면 카메라의 프리뷰를 90도 회전시키기 위한 작업
 * Surface만 돌려서 구현
 * cameraservice.cpp의 handlesutter함수에 스냅샷 후 post view rotation도 해줘야 함
 * MDP 3.1 이하에서만 사용된다. 
 * MSM7230이후부터는 MDP4.0이 사용되기 때문에 의미는 두지 않는다.
 */
/*#define F_SKYCAM_PREVIEW_ROTATION_FOR_SUBCAM*/

/* 
 * 전면 카메라의 JPEG 인코딩시 Rotation적용
 */
#define F_SKYCAM_JPEG_ENCODE_ROTATION_FOR_SUBCAM

/* 
 * 일반 카메라 진입, 종료 후 VT카메라 실행시 start preview fail나는 상황 수정
 * picture size설정이 안되면 mDimension의 값들이 모두 0으로 변해 set dimension fail나는 현상
 */
/* SKYCAM Lee Jitae 2010-09-01
 *  VT카메라를 시작할 때 디폴트 preview사이즈가 800*480이 된다.
 *  이것을 176*144로 세팅하기 위한 작업
 */
#define F_SKYCAM_VT_FIX_CAMERA_START_ERROR

/* 
 * VT 메인카메라 실행시 preview size를 176, 176으로 설정하므로 인해
 * 내 화면 176, 144로 보여지는 것이 깨짐
 * preview size를 176, 176으로 하나 receivePreviewFrame함수에서 176, 144로 crop하여 90 rotate하게 수정 되었음
 ******	
 *  - SKYCAM Lee Jitae 2010-08-30 추가사항MSM7x30,8x55 계열 문제-
 * MDP4.0 을 사용하면서 문제가 발생하기 시작한다.
 * EF13과는 많은 차이를 보여서 수정을 가함 
 * 아직 테스트는 하지 못함(2010-09-01) 테스트 필요
 */
 #define F_SKYCAM_VT_FIX_MAIN_CAM_PREVIEW_SIZE

/*
 * pantech VT는 호가 연결되면 전송 버퍼를 video 버퍼로부터 얻기 위해 start recording
 * 을 시작하며 따라서 connect/disconnect 시에 촬영음이 발생한다.
 * pantech VT에서 촬영음이 발생하는 것을 막기 위해 CameraService에 
 * CAMERA_CMD_SET_SHUTTER_DISABLE commad를 추가 하였다.
*/
 #define F_SKYCAM_VT_SHUTTER_DISABLE

 /* 
 * VT카메라 On/Off 설정
*/
 #define F_SKYCAM_CFG_VT

/* SKYCAM_PSJ_110302
 * 기존 FEATURE폰에서 사용하던 퀄컴의 IPL함수를 사용하기 위해 IPL LIB로드
 * ipl_reflect 테스트 완료
*/
#define F_SKYCAM_USE_IPLLIB

/* SKYCAM Lee Jitae 2010-08-30
 * VT preview에 문제가 있어서 임시적으로 SetOverlay를 하지 못하도록 함
 * VT이외에는 정상인 루틴을 탐
 */
#define F_SKYCAM_VT_OK

/* 
 * MEDIA_RECORDER_INFO_FILESIZE_PROGRESS CallBack, Only Use KT Phone 
 * KT폰에서는 비디오 파일 사이즈를 기록하는데 파일 사이즈를 APP에 알려주기 위해서
 * 추가 
 */
#define F_SKYCAM_ADD_EVT_RECSIZE

/* 이곳이외에 다른곳에 정의 하지 않음 */

#define F_SKYCAM

/* EF18K 에서 사용되는 카메라 정보 테이블 (최대 출력 가능 해상도, 최대 스냅샷 
 * 해상도, AF 지원 여부) 을 수정한다. */
#define F_SKYCAM_CUST_SENSOR_TYPE

/* 카메라 장치 파일 OPEN 에 실패한 경우 (단순 I2C 버스 R/W 오류, 카메라 미장착) 
 * 오류 처리를 위해 수정한다. 
 *
 * 장치 파일을 OPEN 하는 과정에서 VFE 초기화 이후 카메라 HW 초기화가 이루어 
 * 지는데, HW 초기화에 실패할 경우 VFE 는 초기화 된 상태로 남게되고 다음
 * OPEN 시도 시 HW 초기화에 성공한다 하더라도 이미 VFE 가 초기화된 상태이므로 
 * VFE 초기화 시 오류가 발생한다.
 * 
 * 호출순서 : vfefn.vfe_init() -> sctrl.s_init()
 *
 * HW 초기화에 실패할 경우, 이미 초기화된 VFE 를 RELEASE (vfe_release) 시켜 
 * 다음 열기 시도 시 정상 동작하도록 수정한다. 
 *
 * ECLAIR 버전에서는 위와 같은 에러 처리에도 불구하고 센서가 연결되어 있지
 * 않거나 센서 하드웨어에 이상이 발생한 경우 카메라 응용이 ANR 오류로 인해 
 * 비정상 종료되고 이후 재부팅 전까지는 지속하여 재진입이 불가능하다.
 *
 * 센서가 비 정상인 경우, ISP 초기화 시 ISP 와 센서를 프리뷰 모드로 설정하는 
 * 과정에서 3초 간 POLLING 수행하며, 이로 인해 타임아웃 발생하고 ANR 오류로 
 * 이어진다. 비 정상 종료 이후 카메라 재진입 시 센서가 정상이라 하더라도 ANR 
 * 오류 이후 응용이 비 정상적으로 종료되었으므로 FRAMEWORK 내부는 비 정상인 
 * 상태로 유지되고, 이로 인해 재부팅 전까지는 카메라 응용 진입 시 "Unable to 
 * connect camera device" 팝업과 함께 무조건 진입에 실패한다.
 *
 * ISP 초기화 시 프리뷰 모드 설정 이전에, ISP 를 통해 센서의 특정 레지스터를 
 * 1회 READ 하고 비 정상일 경우, 이를 FRAMWORK 을 통해 응용으로 전달하여 
 * 정상적으로 종료되도록 수정한다. 
 *
 * 또한 ISP 자체에 이상이 발생한 경우에도, PROBE 시에 오류 발생하여 해당 
 * 디바이스 파일들을 생성할 수 없으므로 FRAMWORK 내부에서 함께 처리 가능하다. 
 *
 * EF10S 의 경우, BAYER 센서만 커넥터로 연결되어 있고, MV9337 은 ON-BOARD
 * 되어 있으므로, BAYER 센서가 연결되어 있지 않아도, MV9337 만 정상이라면,
 * PROBE 시 정상 동작하였으나, EF12S 의 경우, 카메라 모듈에 MV9335 가 함께
 * 인스톨되어 있어, 커넥터에 모듈이 연결되지 않으면 PROBE 시 I2C R/W 실패가
 * 지속 발생, RETRY 수행하면서 부팅 시간이 10초 이상 지연되고, 이로 인해
 * 다른 모듈들의 초기화에 직접적인 영향을 미친다. */
#define F_SKYCAM_INVALIDATE_CAMERA_CLIENT

/* Service단에서 이전 connect가 남아있을  경우 종료 해주는 부분 추가 */
#define F_SKYCAM_DEVICE_CONNECT_FAIL_FIXED

/* MSM단에서 이전 control, config가 남아있을  경우 종료 해주는 부분 추가 */
#define F_SKYCAM_FIX_MSM_OPEN_FAIL

/*
*camera id별로 검색하여 각각 app에서 후면 카메라, 전면 카메라 따로 동작시 진입 가능하게 되어
*진입시 open이 비슷한 시기에 되거나(홈키 long 키, 전환), setparameter가 셋팅되는 현상등이 발생하여,
*전혀 의도하지 않은 값이 써져 오동작 하는 문제로
*froyo와 마찬가지로 전 후면 모든 카메라가 이전 카메라 release 이전에는 진입 불가하도록 수정
*
*HW, QCH 모두 개별의 카메라 동작을 지원한다면 아래를 제거한 후 테스트 할 것.
*/
#define F_SKYCAM_GB_ORIGIN_CONNECT


/* 카메라 IOCTL 커맨드 MSM_CAM_IOCTL_SENSOR_IO_CFG 를 확장한다. 
 *
 * EF10S 에서 추가한 기능 및 SOC 카메라를 감안하지 않고 미구현된 부분들을 
 * 수정 및 추가 구현한다. */
#define F_SKYCAM_CUST_MSM_CAMERA_CFG


/* SKY캠코더 녹화파일이 Qparser로 확인시 에러발생.(deocde thumnail할수없음)
 * 캠코더 레코딩시 구글캠코더와 SKY캠코더의 차이중 하나가 
 * app에서 내려오는 stagefrightrecorder의 mMaxFileDurationUs 값이다.
 * (SKY캠코더: 3600000000(us)=1시간 / 구글캠코더: 600000000(us)=10분.)
 * mMaxFileDurationUs의 차이로인해 Mpeg4write에서 
 * SKY캠코더는 64bitfileoffset / 구글캠코더는 32bitfileoffset를 사용하게 된다.
 * 이를 32bitfileoffset으로 동일하게 설정하기 위해서 해당부분을 수정한다.
 * 임시로 수정되는 부분이므로 추가 검토 및 지속적인 모니터링이 필요함.
*/
#define F_SKYCAM_VIDEO_REC_FILEOFFSET


/*
 * 카메라 드라이버가 어플이 종료되지 않았을 때, suspend 되는 것을 막는다.
 * power control 에 의해 커널 드라이버가 suspend 되는 것을 막는다.
 * 일반적인 경우 카메라 어플이 카메라 드라이버를 종료 시키며, 이 때 커널 드라이버도 내려간다.
 * HD 영상통화의 경우 조도 센서의 control이 불가능해 LCD가 OFF 되는 상황에서 suspend가 발생한다.
 * 이 때 커널 드라이버가 suspend 되지 않도록 한다.
*/
#define F_SKYCAM_FIX_SUSPENDLOCK_ADD

/* F_SKYCAM_TODO, SKT FOTA DCMO (Device Control Management Object)
 * SKT 향에만 적용되며, UI VOB에서만 define을 연다.
 * "pantech/development/sky_fota/sky_fota.h" 파일이 있어야 한다.
*/
/*#define F_SKYCAM_FOTA_DCMO_CAMERA_LOCK*/

 /* SKYCAM_PSJ_110401
 *  VT의 프리뷰 관련 수정 된 사항에 대한 Feature
 *  VT main카메라의 90도 rotation적용 및 8x60 부터 필요한 2K align버퍼에 대한 처리가 있음
 */
#define F_SKYCAM_FIX_VT_PREVIEW

/* SKYCAM_PSJ_110401
 * 기존 모델들도 Video buffer에 대한 align을 퀄컴 소스에서 처리를  했었으나 8x60에서는 2K(2047) align이 적용 되어있다.
 * 이 버퍼를 얻어가기 위해서는 2K align된 버퍼를 다시 정상으로 돌려야 하는데 이에 대한 작업을 위하여 버퍼 변환하는 함수 추가
 */
#define F_SKYCAM_ADD_ALIGN_VIDEO_BUFFER

/* SKYCAM_PSJ_110401
 * GB부터 전면카메라 구동시 좌우가 반전이 되어있으며 Surface만 돌아가 있는 것이므로 
 * Layer1에서 다시 돌려주는 부분이 필요하다
 * 이 부분은 GB Framework에 정의된 부분이므로 어떤것이 맞는 것인지는 향 후 타모델과 비교하여 정립하여야 할 것이다.
 */
/*#define F_SKYCAM_ADD_REFLECT_FOR_SUBCAM*/

#ifdef F_SKYCAM_TARGET_EF35L
/* LGT의 경우 MMS전송용 동영상촬영시 bitrate를 128bps로 사용해야 함.
 * 그러나 mediaprofiles에서 MPEG4인코딩시 bitrate의 최소값인 192000으로 설정함.
 * 이보다 작은 128000를 설정하기 위해 수정된 부분. 
 */
#define F_SKYCAM_CFG_MMS_REC_FOR_LGT

/* LGT의 경우 VT엔진인 GIPS에서 frame buf를 가져다 사용하게 된다. 
 * GIPS엔진은 encoding시 buf회전을 할 수 없기에 HAL에서 frame buf를 회전시켜 올려주며,
 * frame buf회전은 LGT-VT에만 해당되는 내용이므로 어플에서 .set("pantech-vt", "lgtvt-on"); 호출하여, HAL에서 구분할 수 잇도록 한다.
 * 전면,후면 카메라가 모두 LCD방향으로 변경된 것을 감안하여, 
 * GIPS에서 setPreviewSize는 (144,176)으로, .setDisplayOrientation(0);으로 한다. <-frame buf를 회전시켯으므로 0.
 */
#define F_SKYCAM_VT_FOR_LGT

/* EF35L 김포 공장 요청사항. - 공정중 발생 
 * 문제: 카메라 플래쉬 모드 동작시 Power Reset 발생됨. (무비모드일때는 정상동작)
 * 원인: R1371 오삽. 
 * 이를 검출하기 위해서 #4648#의 카메라 플래쉬모드와 무비모드에 대한 테스트 메뉴를 수정/추가함.
 * 이를 위해 FLASH_MODE_FLASH , LED_MODE_FLASH 를 추가하여 플래쉬 모드 설정 가능하도록 함.
 * 셋팅값인 4는 ICP_HD_CFG_LED_MODE_SNAP 와 매칭되도록 함.
 * 수정사항은 EF35L에만 해당되는 내용으로 EF35L UI vob에만 적용.
 * Function flag 28(Flash mode), 33(Movie mode) 추가하여 적용함 EF35L UI vob에만 적용.
 */
#define F_SKYCAM_FACTORY_REQ_FLASH
#endif

#ifdef F_SKYCAM_TARGET_EF34K
/* SKYCAM_PSJ_110531
 *  stagefright로 인코딩 된 동영상이 KT에서 전송이 안되는 현상
 *  동영상 트랙 헤더의 "pasp" 부분을 KT서버에서 파싱을 못하는 것으로 보여지며 SKT나 LG향에서는 정상적으로 동작이 되어짐
 *  KT향의 경우 서버 수정이 불가한 상황을 대비하여 헤더의 해당 부분을 넣지 않도록 함. 향 후 KT에서 수정 될 수 있는 사항
 */
#define F_SKYCAM_FIX_MMS_CAMCORDER_FOR_KT
#endif

 /* SKYCAM_PSJ_110607
 *  VT카메라 시작시 setPreviewDisplay이후에 preview size의 파라미터 설정을 하여 
 *  setPreviewDisplay에서 만들어졌던 overlay0가 free되고 다시 만들어지는 현상이 있음
 *  다시 정상적으로 만들어져야 하나 destroy(free)와 create사이에 적절한 delay가 없으면 overlay1이 생기고 overlay0이 free되는 경우가 있음
 *  이런 상황을 방지하기 위하여 setPreviewDisplay를 파라미터 설정 이후로 수정
 */
#define F_SKYCAM_FIX_VT_OVERLAY_FAIL

/*----------------------------------------------------------------------------*/
/*  SENSOR CONFIGURATION                                                      */
/*  모델 별 사용 센서(ISP)를 위해 수정/추가된 FEATURE 목록                    */
/*----------------------------------------------------------------------------*/
/* If you use YUV sensor only, define this feature. If you use two cameras, 
 * and one is YUV and another is BAYER, don't define this. If you define this,
 * some BAYER-codes will not be processed.
 */
#define F_SKYCAM_YUV_SENSOR_ONLY

#ifdef F_SKYCAM_TARGET_EF18K
/* EF18K에서사용되는 센서*/
#define F_SKYCAM_MT9P111
#define F_SKYCAM_YACBAC1SDDAS
#endif

#ifdef F_SKYCAM
/* ISP backend camera ISP */
#define F_SKYCAM_ICP_HD
/* 1.3M front camera sensor */
#define F_SKYCAM_S5K6AAFX13
/*#define F_SKYCAM_MT9V113 // vga front camera sensor*/
#endif

#define F_SKYCAM_CUST_ORIENTATION


#ifdef F_SKYCAM_YUV_SENSOR_ONLY
#define F_SKYCAM_ADD_CFG_ANTISHAKE
#define F_SKYCAM_FIX_CFG_EXPOSURE
#define F_SKYCAM_FIX_CFG_SCENE_MODE
#define F_SKYCAM_FIX_CFG_FOCUS_RECT
#define F_SKYCAM_FIX_CFG_REFLECT
#define F_SKYCAM_FIX_CFG_FOCUS_STEP
#define F_SKYCAM_FIX_CFG_ROTATION
#define F_SKYCAM_ADD_CFG_MULTISHOT
#define F_SKYCAM_CUST_PICTURE_SIZES
#define F_SKYCAM_FIX_CFG_BRIGHTNESS
#define F_SKYCAM_FIX_CFG_EFFECT
#define F_SKYCAM_FIX_CFG_PREVIEW_FPS
#define F_SKYCAM_FIX_CFG_ANTIBANDING
#define F_SKYCAM_FIX_CFG_AF
#define F_SKYCAM_CUST_PREVIEW_SIZES
#define F_SKYCAM_FIX_CFG_WB
#define F_SKYCAM_ADD_CFG_DIMENSION
#define F_SKYCAM_FIX_CFG_CAF //ICP_HD Continuous AF추가 (동영상 촬영 시작시 사용) psj 110518
#endif

/* 
 * 올라웍스 얼굴 인식 솔루션 사용하기 위한 FEATURE 
 */
#define F_SKYCAM_ADD_CFG_FACE_FILTER

#ifdef F_SKYCAM_ADD_CFG_FACE_FILTER
#define F_SKYCAM_ADD_CFG_FACE_FILTER_RECT
#endif


/* 플래쉬 LED 설정을 위한 인터페이스를 수정한다.
 *
 * QUALCOMM 에서는 별도의 IOCTL (MSM_CAM_IOCTL_FLASH_LED_CFG) 커맨드를 
 * 사용하여 구현되어 있으며, PMIC 전원을 사용하는 LED 드라이버를 제공한다.
 * MAXIM칩과 ICP_HD의 컨트롤로 FLASH구현
 *
 * AUTO 모드로 설정할 경우, 저조도 일 경우에만 AF 수행 중 AF/AE 를 위해
 * 잠시 ON 되고, 실제 스냅샷 시점에서 한 번 더 ON 된다. */
#define F_SKYCAM_FIX_CFG_LED_MODE

/* SKYCAM PSJ 110224
 * 1080P의 세팅을 dynamic하게 바꿀 수 있도록 드라이버 및 퀄컴 관련 코드 추가 및 수정
 * ISP의 출력이 1080P로 세팅되며 MSM 에서 해당 사이즈를 받을 수 있도록 set dimension을 함
 * 기존의 set dimension 함수에 구현이 되어 있어 F_SKYCAM_ADD_CFG_DIMENSION feature에 
 * dependency를 가지고 있음
 */
#ifdef F_SKYCAM_ADD_CFG_DIMENSION
#define F_SKYCAM_1080P_PREVIEW
#endif
/* Continous AF Feature */
#define F_SKYCAM_CAF

/* 
 * Gingerbread의 CameraService에서 lockIfMessageWanted(int32_t msgType) 함수가 추가 되었다.
 * CameraService의 callback 처리 함수에서 mLock을 check하여 LOCK 상태이면, UNLOCK까지 waiting 한다.
 * capture 수행 도중 UI 로부터 command가 내려오면 callback 함수에서 이로 인해 지연이 발생한다.
 * capture 수행 중 카메라 종료시 이로 인해 CameraHAL보다 UI가 먼저 종료 되는 경우가 발생한다.
 * UI가 먼저 종료되고 CameraHAL 종료전에 다시 Camera가 실행되면 정상적으로 Open 하지 못한다.
 * lockIfMessageWanted 함수를 사용 하지 않도록 수정하였다.
*/
#define F_SKYCAM_FIX_CS_TRYLOCK

/* 동영상 녹화 시작/종료를 빠르게 반복하거나, 이어잭을 장착한 상태에서 연속촬영
 * 모드로 촬영할 경우, MediaPlayer 가 오동작하면서 HALT 발생한다.
 *
 * MediaPlayer 의 경우, 동일한 음원을 재생 중에 또 다시 재생 시도할 경우 100%
 * 오동작하므로 동일 음원을 연속하여 재생해야 할 경우, 반드시 이전 재생이 완료
 * 되었는지 여부를 확인 후 재생해야 한다. */
#define F_SKYCAM_QBUG_SKIP_CAMERA_SOUND

/*
 * 비디오 레코딩 시작과 종료 음이 끊기는 현상에 대한 디버깅
 * 효과음 재생 중 시스템 부하로 인해 소리가 끊기는 경우 발생
 * 레코딩 시작음 start 이 후 시작음 종료 될 때까지 wating
 * EF31S/EF32K 에서는 Sound쪽 kernel message가 나오는 경우 소리가 끊기며,
 * kernel message를 막거나 release build에서는 현상 발생 안함.
*/
#define F_SKYCAM_QBUG_REC_BEEP_SOUND

/* 촬영음/녹화음 재생 중에 응용이 종료될 경우, CLIENT 소멸 시에 해당 촬영음/
 * 녹화음 오브젝트가 강제로 disconnect/clear 되면서 MEDIA SERVER 가 죽는 것을
 * 방지한다. */
#define F_SKYCAM_QBUG_STOP_CAMERA_SOUND

/* 1600x1200, 1600x960 해상도에서 "zoom" 파라미터를 21 로 설정 후 스냅샷 시
 * 썸네일 YUV 이미지를 CROP 하는 과정에서 발생하는 BUFFER OVERRUN 문제를 
 * 임시 수정한다. 
 *
 * QualcommCameraHardware::receiveRawPicture() 에서 crop_yuv420(thumbnail) 
 * 호출 시 파라미터는 width=512, height=384, cropped_width=504, 
 * cropped_height=380 이며 memcpy 도중에 SOURCE 주소보다 DESTINATION 주소가 
 * 더 커지는 상황이 발생한다.
 *
 * R5040 에서 QUALCOMM 수정 확인 후 삭제한다. (SR#308616) 
 * - R407705 FIXED ISSUE 6 번 참조 */
/* #define F_SKYCAM_QBUG_ZOOM_CAUSE_BUFFER_OVERRUN */


/* 모든 해상도에서 ZOOM 을 특정 레벨 이상으로 설정할 경우, 
 * EXIFTAGID_EXIF_PIXEL_X_DIMENSION, EXIFTAGID_EXIF_PIXEL_Y_DIMENSION 태그
 * 정보에 잘못된 값이 저장되는 문제를 임시 수정한다.
 *
 * R5040 에서 QUALCOMM 수정 확인 후 삭제한다. (SR#307343) 
 * - R4077 FIXED ISSUE 12 번 참조 
 * - vendor/qcom/proprietary/mm-still/jpeg/src/jpeg_writer.c 의 기존 임시 수정
 *   코드는 구조가 변경되어 삭제 */
/* #define F_SKYCAM_QBUG_EXIF_IMAGE_WIDTH_HEIGHT */


/* SKYCAM_PSJ_100610
 * 촬영이 종료되지 않은 상황에서 Stop preview를 하여 메모리가 해제되는 상황 방지
*/
#define F_SKYCAM_QBUG_SNAPSHOT_RELEASE_INTERRUPT

/* 8x60에서 녹화된 동영상(MMS/MPEG4/320x240)이 기존 출시모델들에서 decoding시 깨짐현상 발생함.
 * 퀄컴SR을 통해 관련내용 workaround로 적용함.
*/
#define F_SKYCAM_QBUG_DECODING_OTHER_CHIPSET

/* oen exif tag 수정 */
#define F_SKYCAM_OEM_EXIF_TAG

/* CTS qualcomm bug 수정 
 */
#define F_SKYCAM_QBUG_CTS

/* SKYCAM_PSJ_110520
 * 캠코더 녹화시 AV sync안맞는 문제 수정. qcom에서 해결 못해 이창원 과장님이 workaround로 적용한 내용
 * 수정 후 Gom, DaumPot, Km, Quicktime의 player에서 sync맞으며 stagefright에서는 비디오가 약간 빠름
*/
#define F_SKYCAM_QBUG_ENCODING_AV_SYNC

/*----------------------------------------------------------------------------*/
/*  MODEL-SPECIFIC CONSTANTS                                                  */
/*  모델 관련 상수 선언                                       */
/*----------------------------------------------------------------------------*/



#ifdef F_SKYCAM_FACTORY_PROC_CMD
#define C_SKYCAM_FNAME_FACTORY_PROC_CMD	"/data/.factorycamera.dat"
#endif



/* 설정 가능한 최소/최대 포커스 레벨이다. */
#ifdef F_SKYCAM_FIX_CFG_FOCUS_STEP
#define C_SKYCAM_MIN_FOCUS_STEP 0	/* 무한대 (default) */
#define C_SKYCAM_MAX_FOCUS_STEP 9	/* 매크로 */
#endif


/*  FPS 세팅*/
#ifdef F_SKYCAM_FIX_CFG_PREVIEW_FPS
#define C_SKYCAM_MIN_PREVIEW_FPS	0
#define C_SKYCAM_MAX_PREVIEW_FPS	31
#define C_SKYCAM_VT_PREVIEW_FPS		7
#endif


/* Brightness 세팅 */
#ifdef F_SKYCAM_FIX_CFG_BRIGHTNESS
#define C_SKYCAM_MIN_BRIGHTNESS 0
#define C_SKYCAM_MAX_BRIGHTNESS 8
#endif

/* VT camera preview size */
#define VT_CAMERA_PREVIEW_WIDTH		176
#define VT_CAMERA_PREVIEW_HEIGHT 	144

#define C_SKYCAM_SELECT_MAIN_CAM 		0
#define C_SKYCAM_SELECT_SUB_CAM 		1
#define C_SKYCAM_SELECT_VT_SUB_CAM 		2
#define C_SKYCAM_SELECT_VT_MAIN_CAM		3

#ifdef F_SKYCAM_OEM_EXIF_TAG
#define C_SKYCAM_EXIF_MAKE		"PANTECH"
#define C_SKYCAM_EXIF_MAKE_LEN		8		/* including NULL */
#ifdef F_SKYCAM_TARGET_EF33S
#define C_SKYCAM_EXIF_MODEL		"IM-A760S"
#endif
#ifdef F_SKYCAM_TARGET_EF34K
#define C_SKYCAM_EXIF_MODEL		"IM-A770K"
#endif
#ifdef F_SKYCAM_TARGET_EF35L
#define C_SKYCAM_EXIF_MODEL		"IM-A780L"
#endif
#ifdef F_SKYCAM_TARGET_EF40K /*yjw*/
#define C_SKYCAM_EXIF_MODEL		"IM-EF40K"
#endif
#define C_SKYCAM_EXIF_MODEL_LEN		9		/* including NULL */
#endif



#endif /* CUST_SKYCAM.h */
