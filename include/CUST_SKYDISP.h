/*
 lived 2009.11.05
 FEATURE define
*/

#ifndef F_SKYDISP_FRAMEWORK_FEATURE
#define F_SKYDISP_FRAMEWORK_FEATURE

/* Debug Msg */
#if defined(FEATURE_AARM_RELEASE_MODE)
#define SKYDISP_MSG(...)
#else
#define SKYDISP_MSG(...)   ((void)LOG(LOG_WARN, LOG_TAG, __VA_ARGS__))
#endif

/* PMEM ���� ����ȭ
 * MSM_FB_SIZE �� MSM_PMEM_SF_SIZE ũ�� ����
 */
#define F_SKYDISP_PMEM_OPTIMIZE

/* 32bpp ����� ������ �����Ǿ���� �κе� */
#define F_SKYDISP_FRAMEBUFFER_32

/* init ���μ������� �Ѹ��� �� Cursor ���ֱ� */
#define F_SKYDISP_NO_CURSOR_IN_BOOT

/* Android Boot Animation �߿� ��Ⱑ 6���� User Set Value��
 * ���� �Ǵ� ������ ���� -> ������ Feature */
#define F_SKYDISP_SET_BACKLIGHT_BEFORE_BOOTANIM

/* gralloc ��⿡ refresh rate ��� �߸��� �κ� ���� */
#define F_SKYDISP_FIX_REFRESH_RATE

/* Backlight ���� ���� ���� */
#define F_SKYDISP_QBUG_FIX_BACKLIGHT

/* Qualcomm�� �۾��� HDMI ���� DUAL Display�� �۾� ����
 * ����, HDMI�� ����ϴ� ���� �Ʒ� Feature�� undefine�ϰ�
 * Qualcomm�� HDMI_DUAL_DISPLAY �� define�ؾ� �Ѵ�.
 * android/device/qcom/msm8660_surf/ ���� 
 * BoardConfig.mk -> TARGET_HAVE_HDMI_OUT := false
 * system.prop -> ro.hdmi.enable=false
 * �� �� ������ ���� Config/Feature �̴�. ���� �� ��
 */
#define F_SKYDISP_REMOVE_HDMI

/*
 * SHARP MIPI ���� ���� ���� ���� Feature
 */
#define F_SKYDISP_QBUG_FIX_MIPI_ERROR

/*
 * SHARP LCD Veil View ���� Feature
 */
#define F_SKYDISP_VEIL_VIEW

/*
 * SKY Boot Logo in Kernel Feature
 */
#define F_SKYDISP_BOOT_LOGO_IN_KERNEL

/*
 * for Mirror Flip Effect for Camera
 */
#define F_SKYDISP_GBUG_OVERLAY_FLIP

/*
 * Overlay�� ���� SurfaceView���� Landscape<->Portrait ��ȯ ��
 * ȭ���� �ϱ׷����� ������ ����
 * 1080 ���Ŀ��� �ʿ��Ѱ�?
#define F_SKYDISP_GBUG_FIX_OVERLAY_ORIENTATION
 */

/*
 * LCD Module Reset ##1199 Test Menu
 */
#define F_SKYDISP_LCD_RESET

/*
 * LCD ���� on/off ���, sleep������ �Դ´�
 * �ʿ��� �� ���� �ǰڴ�.
 * EF33/34/35���� Battery Charging�ÿ� �־���
 */
#define F_SKYDISP_LCD_FORCE_ONOFF

/*
 * Overlay 2�� ��� �����ϵ��� �Ѵ�.
 * 35L�� GIPS �������� GLSurfaceView�� ���� ������ ���ʿ�
 */
#if ( IS_EF33S || IS_EF34K )
#define F_SKYDISP_DUAL_OVERLAY
#endif

/*
 * Surface Information �߸��� ��� ����
 */
#define F_SKYDISP_FIX_INVALID_SURFACE

/*
 * Overlay ��� ��, MDP_OV_PLAY_NOWAIT�� ���� ó�� �߰�
 */
/*#define F_SKYDISP_OV_PLAY_NOWAIT*/

/*
 * LCD Gamma Table Test
 */
#define F_SKYDISP_LCD_GAMMA_TEST

/*
 * 8660 MIPI Video mode LCD display
 * by Qualcomm SR fix
 */
#define F_SKYDISP_MIPI_VIDEO_LK_DISPLAY

#endif  /* SKY_FRAMEWORK_FEATURE */
