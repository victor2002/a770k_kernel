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



(2)  ī�޶� ���� ��� kernel/userspace/android �α׸� ����

kernel/arch/arm/config/qsd8650-perf_defconfig �� �����Ѵ�.

	# CONFIG_MSM_CAMERA_DEBUG is not set (default)

CUST_SKYCAM.h ���� F_SKYCAM_LOG_PRINTK �� #undef �Ѵ�. 

	#define F_SKYCAM_LOG_PRINTK (default)

��� �ҽ� ���Ͽ��� F_SKYCAM_LOG_OEM �� ã�� �ּ� ó���Ѵ�. 
	���� �� ���, �ش� ���Ͽ� ���� SKYCDBG/SKYCERR ��ũ�θ� �̿��� 
	�޽������� Ȱ��ȭ ��Ų��. (user-space only)

��� �ҽ� ���Ͽ��� F_SKYCAM_LOG_CDBG �� ã�� �ּ� ó���Ѵ�. 
	���� �� ���, �ش� ���Ͽ� ���� CDBG ��ũ�θ� �̿��� �޽������� 
	Ȱ��ȭ ��Ų��. (user-space only)

��� �ҽ� ���Ͽ��� F_SKYCAM_LOG_VERBOSE �� ã�� �ּ� ó���Ѵ�.
	���� �� ���, �ش� ���Ͽ� ���� LOGV/LOGD/LOGI ��ũ�θ� �̿��� 
	�޽������� Ȱ��ȭ ��Ų��. (user-space only)


(4)  �ȸ��ν� ���� ��� ���� ȯ��

vendor/qcom/android-open/libcamera2/Android.mk �� �����Ѵ�.
	3rd PARTY �ַ�� ��� ���θ� �����Ѵ�.

	SKYCAM_FD_ENGINE := 0		������
	SKYCAM_FD_ENGINE := 1		�ö���� �ַ�� ���
	SKYCAM_FD_ENGINE := 2		��Ÿ �ַ�� ���

CUST_SKYCAM.h ���� F_SKYCAM_ADD_CFG_FACE_FILTER �� #undef �Ѵ�.
	�ȸ��ν� ��� ���� �������̽� ���� ���θ� �����Ѵ�.

libOlaEngine.so �� ���� libcamera.so �� ��ũ�ϹǷ� ���� ��� libcamera.so ��
ũ�Ⱑ �����Ͽ� ��ũ ������ �߻� �����ϸ�, �� ��� �Ʒ� ���ϵ鿡�� 
liboemcamera.so �� ������ �ٿ� libcamera.so �� ������ Ȯ���� �� �ִ�.

build/core/prelink-linux-arm-2G.map (for 2G/2G)
build/core/prelink-linux-arm.map (for 1G/3G)	

*/	


/* ���� CS �μ������� �Һ��� �÷� �м��� ���� ���� PC ���α׷��� ����Ͽ� 
 * ī�޶� ���� �ð� ������ PC �� �����Ѵ�. 
 *
 * ���� ����� ���� Ŀ�ǵ� ��缭�� ��õǾ� �����Ƿ� ���� �ڵ���� ���� Ŀ�ǵ� 
 * ���� ��⿡ ���ԵǾ� ������, ���� Ŀ�ǵ� �� PC ���α׷��� ������� �ʰ� ����
 * ���α׷��� ����Ͽ�, �÷��� DIAG ��Ʈ�κ��� ���� �ð� ������ Ȯ���� �� �ִ�.
 *
 * ���� Ŀ�ǵ� ��缭 v10.35 ��� ����
 * PhoneInfoDisplay v4.0 ���α׷����� Ȯ��
 * ��缭�� ���α׷��� DS2�� �ڰ�ȣ ���ӿ��� ���� */
#define F_SKYCAM_FACTORY_PROC_CMD

#define F_SKYCAM_LOG_DEBUG

/*----------------------------------------------------------------------------*/
/*  MODEL-SPECIFIC                                                            */
/*  EF18K ���� ����Ǵ� �Ǵ� EF13S ������ ������ FEATURE ���                 */
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


/*  ��� ī�޶� �۾��� ���ؼ� �߰��� ����
 * ���ĸ� ī�޶� ������ �ȵ���̵� �ܸ����� ī�޶� ��ȯ�� ���� �۾��� �κ�
 * ���ο� API �߰��Ͽ� CamID�� ���� �� �� ����
 */
#define F_SKYCAM_ADD_DUAL_CAM



/* SKYCAM Lee Jitae 2010-09-25
 * VPE�ǻ������ ���ؼ� ���� �������� �ö���� �ʴ� �������� �߻�
 * mDisEnabled�� �̻������ ���� �ذ� 
 */
 #define F_SKYCAM_VIDEO_FRAME_VPE



#define F_SKYCAM_LOG_PRINTK	//Ŀ�� ������ SKYCDBG/SKYCERR �޼��� on off
#define F_SKYCAM_LOG_OEM		//���� ������ SKYCDBG/SKYCERR �޼��� on off

/* MSM7230 is used VFE31 
   QSD8250 is used VFE41 
   Sensor Definition 
 */



/* �ٿ� ����� �ϱ� ���� �Լ��� ����ϱ� ���ؼ�,  ĸ�� ȭ���� �����ִ� ���� ĸ�� ȭ����  Ʋ�� ���� �����ϱ� ���ؼ�
 * ������� ������ �����������並 �����.
 * IPL ���̺귯���� �ִ� downsize�� �̿��ؼ� ���δ�.
 * ���� SR���� �� -> ���� ��ġ�� ���� �� �� �ֵ��� ���� �ޱ� ���ؼ� ���� ��
 */

/* #define F_SKYCAM_SNAPSHOT_PREVIEW  */


/* SKYCAM_PSJ_100513
 * ���� ī�޶��� �����並 90�� ȸ����Ű�� ���� �۾�
 * Surface�� ������ ����
 * cameraservice.cpp�� handlesutter�Լ��� ������ �� post view rotation�� ����� ��
 * MDP 3.1 ���Ͽ����� ���ȴ�. 
 * MSM7230���ĺ��ʹ� MDP4.0�� ���Ǳ� ������ �ǹ̴� ���� �ʴ´�.
 */
/*#define F_SKYCAM_PREVIEW_ROTATION_FOR_SUBCAM*/

/* 
 * ���� ī�޶��� JPEG ���ڵ��� Rotation����
 */
#define F_SKYCAM_JPEG_ENCODE_ROTATION_FOR_SUBCAM

/* 
 * �Ϲ� ī�޶� ����, ���� �� VTī�޶� ����� start preview fail���� ��Ȳ ����
 * picture size������ �ȵǸ� mDimension�� ������ ��� 0���� ���� set dimension fail���� ����
 */
/* SKYCAM Lee Jitae 2010-09-01
 *  VTī�޶� ������ �� ����Ʈ preview����� 800*480�� �ȴ�.
 *  �̰��� 176*144�� �����ϱ� ���� �۾�
 */
#define F_SKYCAM_VT_FIX_CAMERA_START_ERROR

/* 
 * VT ����ī�޶� ����� preview size�� 176, 176���� �����ϹǷ� ����
 * �� ȭ�� 176, 144�� �������� ���� ����
 * preview size�� 176, 176���� �ϳ� receivePreviewFrame�Լ����� 176, 144�� crop�Ͽ� 90 rotate�ϰ� ���� �Ǿ���
 ******	
 *  - SKYCAM Lee Jitae 2010-08-30 �߰�����MSM7x30,8x55 �迭 ����-
 * MDP4.0 �� ����ϸ鼭 ������ �߻��ϱ� �����Ѵ�.
 * EF13���� ���� ���̸� ������ ������ ���� 
 * ���� �׽�Ʈ�� ���� ����(2010-09-01) �׽�Ʈ �ʿ�
 */
 #define F_SKYCAM_VT_FIX_MAIN_CAM_PREVIEW_SIZE

/*
 * pantech VT�� ȣ�� ����Ǹ� ���� ���۸� video ���۷κ��� ��� ���� start recording
 * �� �����ϸ� ���� connect/disconnect �ÿ� �Կ����� �߻��Ѵ�.
 * pantech VT���� �Կ����� �߻��ϴ� ���� ���� ���� CameraService�� 
 * CAMERA_CMD_SET_SHUTTER_DISABLE commad�� �߰� �Ͽ���.
*/
 #define F_SKYCAM_VT_SHUTTER_DISABLE

 /* 
 * VTī�޶� On/Off ����
*/
 #define F_SKYCAM_CFG_VT

/* SKYCAM_PSJ_110302
 * ���� FEATURE������ ����ϴ� ������ IPL�Լ��� ����ϱ� ���� IPL LIB�ε�
 * ipl_reflect �׽�Ʈ �Ϸ�
*/
#define F_SKYCAM_USE_IPLLIB

/* SKYCAM Lee Jitae 2010-08-30
 * VT preview�� ������ �־ �ӽ������� SetOverlay�� ���� ���ϵ��� ��
 * VT�̿ܿ��� ������ ��ƾ�� Ž
 */
#define F_SKYCAM_VT_OK

/* 
 * MEDIA_RECORDER_INFO_FILESIZE_PROGRESS CallBack, Only Use KT Phone 
 * KT�������� ���� ���� ����� ����ϴµ� ���� ����� APP�� �˷��ֱ� ���ؼ�
 * �߰� 
 */
#define F_SKYCAM_ADD_EVT_RECSIZE

/* �̰��̿ܿ� �ٸ����� ���� ���� ���� */

#define F_SKYCAM

/* EF18K ���� ���Ǵ� ī�޶� ���� ���̺� (�ִ� ��� ���� �ػ�, �ִ� ������ 
 * �ػ�, AF ���� ����) �� �����Ѵ�. */
#define F_SKYCAM_CUST_SENSOR_TYPE

/* ī�޶� ��ġ ���� OPEN �� ������ ��� (�ܼ� I2C ���� R/W ����, ī�޶� ������) 
 * ���� ó���� ���� �����Ѵ�. 
 *
 * ��ġ ������ OPEN �ϴ� �������� VFE �ʱ�ȭ ���� ī�޶� HW �ʱ�ȭ�� �̷�� 
 * ���µ�, HW �ʱ�ȭ�� ������ ��� VFE �� �ʱ�ȭ �� ���·� ���Եǰ� ����
 * OPEN �õ� �� HW �ʱ�ȭ�� �����Ѵ� �ϴ��� �̹� VFE �� �ʱ�ȭ�� �����̹Ƿ� 
 * VFE �ʱ�ȭ �� ������ �߻��Ѵ�.
 * 
 * ȣ����� : vfefn.vfe_init() -> sctrl.s_init()
 *
 * HW �ʱ�ȭ�� ������ ���, �̹� �ʱ�ȭ�� VFE �� RELEASE (vfe_release) ���� 
 * ���� ���� �õ� �� ���� �����ϵ��� �����Ѵ�. 
 *
 * ECLAIR ���������� ���� ���� ���� ó������ �ұ��ϰ� ������ ����Ǿ� ����
 * �ʰų� ���� �ϵ��� �̻��� �߻��� ��� ī�޶� ������ ANR ������ ���� 
 * ������ ����ǰ� ���� ����� �������� �����Ͽ� �������� �Ұ����ϴ�.
 *
 * ������ �� ������ ���, ISP �ʱ�ȭ �� ISP �� ������ ������ ���� �����ϴ� 
 * �������� 3�� �� POLLING �����ϸ�, �̷� ���� Ÿ�Ӿƿ� �߻��ϰ� ANR ������ 
 * �̾�����. �� ���� ���� ���� ī�޶� ������ �� ������ �����̶� �ϴ��� ANR 
 * ���� ���� ������ �� ���������� ����Ǿ����Ƿ� FRAMEWORK ���δ� �� ������ 
 * ���·� �����ǰ�, �̷� ���� ����� �������� ī�޶� ���� ���� �� "Unable to 
 * connect camera device" �˾��� �Բ� ������ ���Կ� �����Ѵ�.
 *
 * ISP �ʱ�ȭ �� ������ ��� ���� ������, ISP �� ���� ������ Ư�� �������͸� 
 * 1ȸ READ �ϰ� �� ������ ���, �̸� FRAMWORK �� ���� �������� �����Ͽ� 
 * ���������� ����ǵ��� �����Ѵ�. 
 *
 * ���� ISP ��ü�� �̻��� �߻��� ��쿡��, PROBE �ÿ� ���� �߻��Ͽ� �ش� 
 * ����̽� ���ϵ��� ������ �� �����Ƿ� FRAMWORK ���ο��� �Բ� ó�� �����ϴ�. 
 *
 * EF10S �� ���, BAYER ������ Ŀ���ͷ� ����Ǿ� �ְ�, MV9337 �� ON-BOARD
 * �Ǿ� �����Ƿ�, BAYER ������ ����Ǿ� ���� �ʾƵ�, MV9337 �� �����̶��,
 * PROBE �� ���� �����Ͽ�����, EF12S �� ���, ī�޶� ��⿡ MV9335 �� �Բ�
 * �ν���Ǿ� �־�, Ŀ���Ϳ� ����� ������� ������ PROBE �� I2C R/W ���а�
 * ���� �߻�, RETRY �����ϸ鼭 ���� �ð��� 10�� �̻� �����ǰ�, �̷� ����
 * �ٸ� ������ �ʱ�ȭ�� �������� ������ ��ģ��. */
#define F_SKYCAM_INVALIDATE_CAMERA_CLIENT

/* Service�ܿ��� ���� connect�� ��������  ��� ���� ���ִ� �κ� �߰� */
#define F_SKYCAM_DEVICE_CONNECT_FAIL_FIXED

/* MSM�ܿ��� ���� control, config�� ��������  ��� ���� ���ִ� �κ� �߰� */
#define F_SKYCAM_FIX_MSM_OPEN_FAIL

/*
*camera id���� �˻��Ͽ� ���� app���� �ĸ� ī�޶�, ���� ī�޶� ���� ���۽� ���� �����ϰ� �Ǿ�
*���Խ� open�� ����� �ñ⿡ �ǰų�(ȨŰ long Ű, ��ȯ), setparameter�� ���õǴ� ������� �߻��Ͽ�,
*���� �ǵ����� ���� ���� ���� ������ �ϴ� ������
*froyo�� ���������� �� �ĸ� ��� ī�޶� ���� ī�޶� release �������� ���� �Ұ��ϵ��� ����
*
*HW, QCH ��� ������ ī�޶� ������ �����Ѵٸ� �Ʒ��� ������ �� �׽�Ʈ �� ��.
*/
#define F_SKYCAM_GB_ORIGIN_CONNECT


/* ī�޶� IOCTL Ŀ�ǵ� MSM_CAM_IOCTL_SENSOR_IO_CFG �� Ȯ���Ѵ�. 
 *
 * EF10S ���� �߰��� ��� �� SOC ī�޶� �������� �ʰ� �̱����� �κе��� 
 * ���� �� �߰� �����Ѵ�. */
#define F_SKYCAM_CUST_MSM_CAMERA_CFG


/* SKYķ�ڴ� ��ȭ������ Qparser�� Ȯ�ν� �����߻�.(deocde thumnail�Ҽ�����)
 * ķ�ڴ� ���ڵ��� ����ķ�ڴ��� SKYķ�ڴ��� ������ �ϳ��� 
 * app���� �������� stagefrightrecorder�� mMaxFileDurationUs ���̴�.
 * (SKYķ�ڴ�: 3600000000(us)=1�ð� / ����ķ�ڴ�: 600000000(us)=10��.)
 * mMaxFileDurationUs�� ���̷����� Mpeg4write���� 
 * SKYķ�ڴ��� 64bitfileoffset / ����ķ�ڴ��� 32bitfileoffset�� ����ϰ� �ȴ�.
 * �̸� 32bitfileoffset���� �����ϰ� �����ϱ� ���ؼ� �ش�κ��� �����Ѵ�.
 * �ӽ÷� �����Ǵ� �κ��̹Ƿ� �߰� ���� �� �������� ����͸��� �ʿ���.
*/
#define F_SKYCAM_VIDEO_REC_FILEOFFSET


/*
 * ī�޶� ����̹��� ������ ������� �ʾ��� ��, suspend �Ǵ� ���� ���´�.
 * power control �� ���� Ŀ�� ����̹��� suspend �Ǵ� ���� ���´�.
 * �Ϲ����� ��� ī�޶� ������ ī�޶� ����̹��� ���� ��Ű��, �� �� Ŀ�� ����̹��� ��������.
 * HD ������ȭ�� ��� ���� ������ control�� �Ұ����� LCD�� OFF �Ǵ� ��Ȳ���� suspend�� �߻��Ѵ�.
 * �� �� Ŀ�� ����̹��� suspend ���� �ʵ��� �Ѵ�.
*/
#define F_SKYCAM_FIX_SUSPENDLOCK_ADD

/* F_SKYCAM_TODO, SKT FOTA DCMO (Device Control Management Object)
 * SKT �⿡�� ����Ǹ�, UI VOB������ define�� ����.
 * "pantech/development/sky_fota/sky_fota.h" ������ �־�� �Ѵ�.
*/
/*#define F_SKYCAM_FOTA_DCMO_CAMERA_LOCK*/

 /* SKYCAM_PSJ_110401
 *  VT�� ������ ���� ���� �� ���׿� ���� Feature
 *  VT mainī�޶��� 90�� rotation���� �� 8x60 ���� �ʿ��� 2K align���ۿ� ���� ó���� ����
 */
#define F_SKYCAM_FIX_VT_PREVIEW

/* SKYCAM_PSJ_110401
 * ���� �𵨵鵵 Video buffer�� ���� align�� ���� �ҽ����� ó����  �߾����� 8x60������ 2K(2047) align�� ���� �Ǿ��ִ�.
 * �� ���۸� ���� ���ؼ��� 2K align�� ���۸� �ٽ� �������� ������ �ϴµ� �̿� ���� �۾��� ���Ͽ� ���� ��ȯ�ϴ� �Լ� �߰�
 */
#define F_SKYCAM_ADD_ALIGN_VIDEO_BUFFER

/* SKYCAM_PSJ_110401
 * GB���� ����ī�޶� ������ �¿찡 ������ �Ǿ������� Surface�� ���ư� �ִ� ���̹Ƿ� 
 * Layer1���� �ٽ� �����ִ� �κ��� �ʿ��ϴ�
 * �� �κ��� GB Framework�� ���ǵ� �κ��̹Ƿ� ����� �´� �������� �� �� Ÿ�𵨰� ���Ͽ� �����Ͽ��� �� ���̴�.
 */
/*#define F_SKYCAM_ADD_REFLECT_FOR_SUBCAM*/

#ifdef F_SKYCAM_TARGET_EF35L
/* LGT�� ��� MMS���ۿ� �������Կ��� bitrate�� 128bps�� ����ؾ� ��.
 * �׷��� mediaprofiles���� MPEG4���ڵ��� bitrate�� �ּҰ��� 192000���� ������.
 * �̺��� ���� 128000�� �����ϱ� ���� ������ �κ�. 
 */
#define F_SKYCAM_CFG_MMS_REC_FOR_LGT

/* LGT�� ��� VT������ GIPS���� frame buf�� ������ ����ϰ� �ȴ�. 
 * GIPS������ encoding�� bufȸ���� �� �� ���⿡ HAL���� frame buf�� ȸ������ �÷��ָ�,
 * frame bufȸ���� LGT-VT���� �ش�Ǵ� �����̹Ƿ� ���ÿ��� .set("pantech-vt", "lgtvt-on"); ȣ���Ͽ�, HAL���� ������ �� �յ��� �Ѵ�.
 * ����,�ĸ� ī�޶� ��� LCD�������� ����� ���� �����Ͽ�, 
 * GIPS���� setPreviewSize�� (144,176)����, .setDisplayOrientation(0);���� �Ѵ�. <-frame buf�� ȸ���������Ƿ� 0.
 */
#define F_SKYCAM_VT_FOR_LGT

/* EF35L ���� ���� ��û����. - ������ �߻� 
 * ����: ī�޶� �÷��� ��� ���۽� Power Reset �߻���. (�������϶��� ������)
 * ����: R1371 ����. 
 * �̸� �����ϱ� ���ؼ� #4648#�� ī�޶� �÷������� �����忡 ���� �׽�Ʈ �޴��� ����/�߰���.
 * �̸� ���� FLASH_MODE_FLASH , LED_MODE_FLASH �� �߰��Ͽ� �÷��� ��� ���� �����ϵ��� ��.
 * ���ð��� 4�� ICP_HD_CFG_LED_MODE_SNAP �� ��Ī�ǵ��� ��.
 * ���������� EF35L���� �ش�Ǵ� �������� EF35L UI vob���� ����.
 * Function flag 28(Flash mode), 33(Movie mode) �߰��Ͽ� ������ EF35L UI vob���� ����.
 */
#define F_SKYCAM_FACTORY_REQ_FLASH
#endif

#ifdef F_SKYCAM_TARGET_EF34K
/* SKYCAM_PSJ_110531
 *  stagefright�� ���ڵ� �� �������� KT���� ������ �ȵǴ� ����
 *  ������ Ʈ�� ����� "pasp" �κ��� KT�������� �Ľ��� ���ϴ� ������ �������� SKT�� LG�⿡���� ���������� ������ �Ǿ���
 *  KT���� ��� ���� ������ �Ұ��� ��Ȳ�� ����Ͽ� ����� �ش� �κ��� ���� �ʵ��� ��. �� �� KT���� ���� �� �� �ִ� ����
 */
#define F_SKYCAM_FIX_MMS_CAMCORDER_FOR_KT
#endif

 /* SKYCAM_PSJ_110607
 *  VTī�޶� ���۽� setPreviewDisplay���Ŀ� preview size�� �Ķ���� ������ �Ͽ� 
 *  setPreviewDisplay���� ��������� overlay0�� free�ǰ� �ٽ� ��������� ������ ����
 *  �ٽ� ���������� ��������� �ϳ� destroy(free)�� create���̿� ������ delay�� ������ overlay1�� ����� overlay0�� free�Ǵ� ��찡 ����
 *  �̷� ��Ȳ�� �����ϱ� ���Ͽ� setPreviewDisplay�� �Ķ���� ���� ���ķ� ����
 */
#define F_SKYCAM_FIX_VT_OVERLAY_FAIL

/*----------------------------------------------------------------------------*/
/*  SENSOR CONFIGURATION                                                      */
/*  �� �� ��� ����(ISP)�� ���� ����/�߰��� FEATURE ���                    */
/*----------------------------------------------------------------------------*/
/* If you use YUV sensor only, define this feature. If you use two cameras, 
 * and one is YUV and another is BAYER, don't define this. If you define this,
 * some BAYER-codes will not be processed.
 */
#define F_SKYCAM_YUV_SENSOR_ONLY

#ifdef F_SKYCAM_TARGET_EF18K
/* EF18K�������Ǵ� ����*/
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
#define F_SKYCAM_FIX_CFG_CAF //ICP_HD Continuous AF�߰� (������ �Կ� ���۽� ���) psj 110518
#endif

/* 
 * �ö���� �� �ν� �ַ�� ����ϱ� ���� FEATURE 
 */
#define F_SKYCAM_ADD_CFG_FACE_FILTER

#ifdef F_SKYCAM_ADD_CFG_FACE_FILTER
#define F_SKYCAM_ADD_CFG_FACE_FILTER_RECT
#endif


/* �÷��� LED ������ ���� �������̽��� �����Ѵ�.
 *
 * QUALCOMM ������ ������ IOCTL (MSM_CAM_IOCTL_FLASH_LED_CFG) Ŀ�ǵ带 
 * ����Ͽ� �����Ǿ� ������, PMIC ������ ����ϴ� LED ����̹��� �����Ѵ�.
 * MAXIMĨ�� ICP_HD�� ��Ʈ�ѷ� FLASH����
 *
 * AUTO ���� ������ ���, ������ �� ��쿡�� AF ���� �� AF/AE �� ����
 * ��� ON �ǰ�, ���� ������ �������� �� �� �� ON �ȴ�. */
#define F_SKYCAM_FIX_CFG_LED_MODE

/* SKYCAM PSJ 110224
 * 1080P�� ������ dynamic�ϰ� �ٲ� �� �ֵ��� ����̹� �� ���� ���� �ڵ� �߰� �� ����
 * ISP�� ����� 1080P�� ���õǸ� MSM ���� �ش� ����� ���� �� �ֵ��� set dimension�� ��
 * ������ set dimension �Լ��� ������ �Ǿ� �־� F_SKYCAM_ADD_CFG_DIMENSION feature�� 
 * dependency�� ������ ����
 */
#ifdef F_SKYCAM_ADD_CFG_DIMENSION
#define F_SKYCAM_1080P_PREVIEW
#endif
/* Continous AF Feature */
#define F_SKYCAM_CAF

/* 
 * Gingerbread�� CameraService���� lockIfMessageWanted(int32_t msgType) �Լ��� �߰� �Ǿ���.
 * CameraService�� callback ó�� �Լ����� mLock�� check�Ͽ� LOCK �����̸�, UNLOCK���� waiting �Ѵ�.
 * capture ���� ���� UI �κ��� command�� �������� callback �Լ����� �̷� ���� ������ �߻��Ѵ�.
 * capture ���� �� ī�޶� ����� �̷� ���� CameraHAL���� UI�� ���� ���� �Ǵ� ��찡 �߻��Ѵ�.
 * UI�� ���� ����ǰ� CameraHAL �������� �ٽ� Camera�� ����Ǹ� ���������� Open ���� ���Ѵ�.
 * lockIfMessageWanted �Լ��� ��� ���� �ʵ��� �����Ͽ���.
*/
#define F_SKYCAM_FIX_CS_TRYLOCK

/* ������ ��ȭ ����/���Ḧ ������ �ݺ��ϰų�, �̾����� ������ ���¿��� �����Կ�
 * ���� �Կ��� ���, MediaPlayer �� �������ϸ鼭 HALT �߻��Ѵ�.
 *
 * MediaPlayer �� ���, ������ ������ ��� �߿� �� �ٽ� ��� �õ��� ��� 100%
 * �������ϹǷ� ���� ������ �����Ͽ� ����ؾ� �� ���, �ݵ�� ���� ����� �Ϸ�
 * �Ǿ����� ���θ� Ȯ�� �� ����ؾ� �Ѵ�. */
#define F_SKYCAM_QBUG_SKIP_CAMERA_SOUND

/*
 * ���� ���ڵ� ���۰� ���� ���� ����� ���� ���� �����
 * ȿ���� ��� �� �ý��� ���Ϸ� ���� �Ҹ��� ����� ��� �߻�
 * ���ڵ� ������ start �� �� ������ ���� �� ������ wating
 * EF31S/EF32K ������ Sound�� kernel message�� ������ ��� �Ҹ��� �����,
 * kernel message�� ���ų� release build������ ���� �߻� ����.
*/
#define F_SKYCAM_QBUG_REC_BEEP_SOUND

/* �Կ���/��ȭ�� ��� �߿� ������ ����� ���, CLIENT �Ҹ� �ÿ� �ش� �Կ���/
 * ��ȭ�� ������Ʈ�� ������ disconnect/clear �Ǹ鼭 MEDIA SERVER �� �״� ����
 * �����Ѵ�. */
#define F_SKYCAM_QBUG_STOP_CAMERA_SOUND

/* 1600x1200, 1600x960 �ػ󵵿��� "zoom" �Ķ���͸� 21 �� ���� �� ������ ��
 * ����� YUV �̹����� CROP �ϴ� �������� �߻��ϴ� BUFFER OVERRUN ������ 
 * �ӽ� �����Ѵ�. 
 *
 * QualcommCameraHardware::receiveRawPicture() ���� crop_yuv420(thumbnail) 
 * ȣ�� �� �Ķ���ʹ� width=512, height=384, cropped_width=504, 
 * cropped_height=380 �̸� memcpy ���߿� SOURCE �ּҺ��� DESTINATION �ּҰ� 
 * �� Ŀ���� ��Ȳ�� �߻��Ѵ�.
 *
 * R5040 ���� QUALCOMM ���� Ȯ�� �� �����Ѵ�. (SR#308616) 
 * - R407705 FIXED ISSUE 6 �� ���� */
/* #define F_SKYCAM_QBUG_ZOOM_CAUSE_BUFFER_OVERRUN */


/* ��� �ػ󵵿��� ZOOM �� Ư�� ���� �̻����� ������ ���, 
 * EXIFTAGID_EXIF_PIXEL_X_DIMENSION, EXIFTAGID_EXIF_PIXEL_Y_DIMENSION �±�
 * ������ �߸��� ���� ����Ǵ� ������ �ӽ� �����Ѵ�.
 *
 * R5040 ���� QUALCOMM ���� Ȯ�� �� �����Ѵ�. (SR#307343) 
 * - R4077 FIXED ISSUE 12 �� ���� 
 * - vendor/qcom/proprietary/mm-still/jpeg/src/jpeg_writer.c �� ���� �ӽ� ����
 *   �ڵ�� ������ ����Ǿ� ���� */
/* #define F_SKYCAM_QBUG_EXIF_IMAGE_WIDTH_HEIGHT */


/* SKYCAM_PSJ_100610
 * �Կ��� ������� ���� ��Ȳ���� Stop preview�� �Ͽ� �޸𸮰� �����Ǵ� ��Ȳ ����
*/
#define F_SKYCAM_QBUG_SNAPSHOT_RELEASE_INTERRUPT

/* 8x60���� ��ȭ�� ������(MMS/MPEG4/320x240)�� ���� ��ø𵨵鿡�� decoding�� �������� �߻���.
 * ����SR�� ���� ���ó��� workaround�� ������.
*/
#define F_SKYCAM_QBUG_DECODING_OTHER_CHIPSET

/* oen exif tag ���� */
#define F_SKYCAM_OEM_EXIF_TAG

/* CTS qualcomm bug ���� 
 */
#define F_SKYCAM_QBUG_CTS

/* SKYCAM_PSJ_110520
 * ķ�ڴ� ��ȭ�� AV sync�ȸ´� ���� ����. qcom���� �ذ� ���� ��â�� ������� workaround�� ������ ����
 * ���� �� Gom, DaumPot, Km, Quicktime�� player���� sync������ stagefright������ ������ �ణ ����
*/
#define F_SKYCAM_QBUG_ENCODING_AV_SYNC

/*----------------------------------------------------------------------------*/
/*  MODEL-SPECIFIC CONSTANTS                                                  */
/*  �� ���� ��� ����                                       */
/*----------------------------------------------------------------------------*/



#ifdef F_SKYCAM_FACTORY_PROC_CMD
#define C_SKYCAM_FNAME_FACTORY_PROC_CMD	"/data/.factorycamera.dat"
#endif



/* ���� ������ �ּ�/�ִ� ��Ŀ�� �����̴�. */
#ifdef F_SKYCAM_FIX_CFG_FOCUS_STEP
#define C_SKYCAM_MIN_FOCUS_STEP 0	/* ���Ѵ� (default) */
#define C_SKYCAM_MAX_FOCUS_STEP 9	/* ��ũ�� */
#endif


/*  FPS ����*/
#ifdef F_SKYCAM_FIX_CFG_PREVIEW_FPS
#define C_SKYCAM_MIN_PREVIEW_FPS	0
#define C_SKYCAM_MAX_PREVIEW_FPS	31
#define C_SKYCAM_VT_PREVIEW_FPS		7
#endif


/* Brightness ���� */
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
