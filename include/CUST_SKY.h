#ifndef _CUST_SKY_FILE_
#define _CUST_SKY_FILE_

/* MODEL_ID  */
#define MODEL_EF33S      0x3004
#define MODEL_EF34K      0x3005
#define MODEL_EF35L      0x3006
#define MODEL_EF40K      0x3007 /*yjw*/

/* BOARD_VER */
#define EV10	((MODEL_EF33S<<8)+0x02)
#define WS10	((MODEL_EF33S<<8)+0x03)
#define WS20	((MODEL_EF33S<<8)+0x04)
#define TP10	((MODEL_EF33S<<8)+0x05)
#define TP15	((MODEL_EF33S<<8)+0x06)
#define TP20	((MODEL_EF33S<<8)+0x07)

#define BOARD_VER_L(a)                  (BOARD_VER < a)
#define BOARD_VER_LE(a)                 (BOARD_VER <= a)
#define BOARD_VER_E(a)                  (BOARD_VER == a)
#define BOARD_VER_GE(a)                 (BOARD_VER >= a)
#define BOARD_VER_G(a)                  (BOARD_VER > a)

/* EF33S MACRO */
#define IS_EF33S                        (MODEL_ID == MODEL_EF33S)
#define NOT_EF33S                       (MODEL_ID != MODEL_EF33S)
#define EF33S_BDVER_L(a)                ((MODEL_ID == MODEL_EF33S) && (BOARD_VER < a))
#define EF33S_BDVER_LE(a)               ((MODEL_ID == MODEL_EF33S) && (BOARD_VER <= a))
#define EF33S_BDVER_E(a)                ((MODEL_ID == MODEL_EF33S) && (BOARD_VER == a))
#define EF33S_BDVER_GE(a)               ((MODEL_ID == MODEL_EF33S) && (BOARD_VER >= a))
#define EF33S_BDVER_G(a)                ((MODEL_ID == MODEL_EF33S) && (BOARD_VER > a))

/* EF34K MACRO */
#define IS_EF34K                        (MODEL_ID == MODEL_EF34K)
#define NOT_EF34K                       (MODEL_ID != MODEL_EF34K)
#define EF34K_BDVER_L(a)                ((MODEL_ID == MODEL_EF34K) && (BOARD_VER < a))
#define EF34K_BDVER_LE(a)               ((MODEL_ID == MODEL_EF34K) && (BOARD_VER <= a))
#define EF34K_BDVER_E(a)                ((MODEL_ID == MODEL_EF34K) && (BOARD_VER == a))
#define EF34K_BDVER_GE(a)               ((MODEL_ID == MODEL_EF34K) && (BOARD_VER >= a))
#define EF34K_BDVER_G(a)                ((MODEL_ID == MODEL_EF34K) && (BOARD_VER > a))

/* EF35L MACRO */
#define IS_EF35L                        (MODEL_ID == MODEL_EF35L)
#define NOT_EF35L                       (MODEL_ID != MODEL_EF35L)
#define EF35L_BDVER_L(a)                ((MODEL_ID == MODEL_EF35L) && (BOARD_VER < a))
#define EF35L_BDVER_LE(a)               ((MODEL_ID == MODEL_EF35L) && (BOARD_VER <= a))
#define EF35L_BDVER_E(a)                ((MODEL_ID == MODEL_EF35L) && (BOARD_VER == a))
#define EF35L_BDVER_GE(a)               ((MODEL_ID == MODEL_EF35L) && (BOARD_VER >= a))
#define EF35L_BDVER_G(a)                ((MODEL_ID == MODEL_EF35L) && (BOARD_VER > a))

/* EF40K MACRO *//*yjw*/
#define IS_EF40K                        (MODEL_ID == MODEL_EF40K)
#define NOT_EF40K                       (MODEL_ID != MODEL_EF40K)
#define EF40K_BDVER_L(a)                ((MODEL_ID == MODEL_EF40K) && (BOARD_VER < a))
#define EF40K_BDVER_LE(a)               ((MODEL_ID == MODEL_EF40K) && (BOARD_VER <= a))
#define EF40K_BDVER_E(a)                ((MODEL_ID == MODEL_EF40K) && (BOARD_VER == a))
#define EF40K_BDVER_GE(a)               ((MODEL_ID == MODEL_EF40K) && (BOARD_VER >= a))
#define EF40K_BDVER_G(a)                ((MODEL_ID == MODEL_EF40K) && (BOARD_VER > a))

#if (MODEL_ID == MODEL_EF35L)
#define FEATURE_EF35L_FACTORY
#endif  //p13120

#if (MODEL_ID == MODEL_EF33S)
    #ifndef CONFIG_EF33_BOARD
    #define CONFIG_EF33_BOARD
    #endif
#elif (MODEL_ID == MODEL_EF34K)
    #ifndef CONFIG_EF34_BOARD
    #define CONFIG_EF34_BOARD
    #endif
#elif (MODEL_ID == MODEL_EF35L)
    #ifndef CONFIG_EF35_BOARD
    #define CONFIG_EF35_BOARD
    #endif
#elif (MODEL_ID == MODEL_EF40K) /*yjw*/
    #ifndef CONFIG_EF40_BOARD
    #define CONFIG_EF40_BOARD
    #endif	
#endif    

/*#define CONFIG_HSUSB_PANTECH_OBEX*/
#define FEATURE_ANDROID_PANTECH_USB_SKY
#define FEATURE_ANDROID_PANTECH_USB_QC_FIX_BUG
#define FEATURE_ANDROID_PANTECH_USB_UEVENT_PATCH //2011.04.11
#define FEATURE_ANDROID_PANTECH_USB_SERIAL_SET_at_8X60 //2011.06.15

#if defined(CONFIG_HSUSB_PANTECH_OBEX)
#define CONFIG_HSUSB_PANTECH_USB_TEST
#endif

#define FEATURE_PANTECH_MDS_MTC   /* MTC */

#if defined(FEATURE_PANTECH_MDS_MTC)
#define FEATURE_PANTECH_MAT      /* MAT */
#endif

#if defined(FEATURE_PANTECH_MDS_MTC)
#define FEATURE_DIAG_LARGE_PACKET
#endif

#define FEATURE_PANTECH_STABILITY  /* STABILITY */

#if defined(FEATURE_AARM_RELEASE_MODE)
#define FEATURE_SKY_DM_MSG_VIEW_DISABLE
#define FEATURE_SW_RESET_RELEASE_MODE // use in release mode
#endif

#define FEATURE_PANTECH_VOLUME_CTL

#define FEATURE_PANTECH_BOOTING_EVENT_DROP

/*******************************************************************************
**  PDL
*******************************************************************************/
#define FEATURE_SKY_PDL_DLOADINFO
#define FEATURE_SKY_PDL_DLOAD
#define FEATURE_SKY_FLASH_ACCESS
#define FEATURE_SKY_DLOAD_USB
#define FEATURE_SKY_REBOOT_FOR_IDLE_DL


/*******************************************************************************
**  RAWDATA PARTITION ACCESS, FOR BACKUP
*******************************************************************************/
#define FEATURE_SKY_RAWDATA_ACCESS

/*******************************************************************************
**  SKY STATION SDCARD UPGRADE
*******************************************************************************/
#define FEATURE_SKY_SDCARD_UPGRADE
#define FEATURE_SKY_SELF_UPGRADE_SDCARD

/*******************************************************************************
**  UIM
*******************************************************************************/
#define FEATURE_SKY_UIM_TESTMENU

/*******************************************************************************
**  NFC
*******************************************************************************/
#if ( IS_EF33S || IS_EF34K )
#if (BOARD_VER <= TP10)
#define CONFIG_ST21NFCA
#else
#define CONFIG_PN544
#endif /* (BOARD_VER <= WS20) */
#endif /* ( IS_EF33S || IS_EF34K ) */


/*******************************************************************************
**  RF
*******************************************************************************/
#define FEATURE_SKY_RF_TESTMENU

/*******************************************************************************
**  BOOTLOADER UART LOG for debugging
*******************************************************************************/
/*#define FEATURE_SKY_BOOTLOADER_UART_LOG*/

/*******************************************************************************
  **  SW reset changed to HW reset.
  *******************************************************************************/
#define FEATURE_SW_RESET_CONVERT_HW_RESET
#define FEATURE_SKY_ANDROID_PANIC_DEBUG_VERSION
#define FEATURE_SKY_SAVE_LOG_SDCARD 			//p13156 lks pantech 20110512

/*******************************************************************************
**  USER DATA REBUILDING VERSION
*******************************************************************************/
#define FEATURE_SKY_USER_DATA_VER


/*******************************************************************************
**  Android Pattern rest Feature
*******************************************************************************/
#define FEATURE_SKY_PATTERN_RESET

/* Global features for SKY camera framework. */
#include "CUST_SKYCAM.h"
/* [PS3] Kang Seong-Goo framework features about SurfaceFlinger */
#include "CUST_SKYDISP.h"

#define FEATURE_SKYSND

/****************************************************
** MMC(eMMC, MicroSD)
****************************************************/
#define FEATURE_SKY_MMC

/****************************************************
** EFS_ERASE
****************************************************/
#define FEATURE_SKY_EFS_ERASE

/* paiksun... */
/****************************************************
** USB CHARGING
****************************************************/
#define FEATURE_SKY_CHARGING
#define FEATURE_SKY_CHG_LOGO

/****************************************************
** POWER ON/OFF REASON COUNT
****************************************************/
#define FEATURE_SKY_PWR_ONOFF_REASON_CNT

/****************************************************
** PMIC
****************************************************/
#define FEATURE_SKY_PMIC

/****************************************************
** PDIP
****************************************************/
#define FEATURE_SKY_PDIP_COMMAND


/* 2010-11-14 octopusy added  [PS1 Team Feature] */
#ifdef T_SKY_MODEL_TARGET_COMMON
#include "cust_sky_cp.h"
#endif/* T_SKY_MODEL_TARGET_COMMON */

/*
  2011/03/02 권오윤
  PANTECH multimedia engine/codec 개발 관련 feature 정의파일 (최상위 FEATURE_PANTECH_MMP 등)
*/
#include "cust_pantech_mmp.h"

/****************************************************
** HS 3.5PHI EARJACK    PS2 P13106 Kang, Yoonkoo
****************************************************/
#define FEATURE_SKY_3_5PHIEARJACK


/*******************************************************************************
**  WLAN
*******************************************************************************/
#define FEATURE_SKY_WLAN
#define FEATURE_SKY_WLAN_RAWDATA_ACCESS 

/*******************************************************************************
**  FOTA
*******************************************************************************/
#define GOTA_CONFIG

#endif/*_CUST_SKY_FILE_*/
