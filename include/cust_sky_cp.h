#ifndef __CUST_SKY_CP_H__
#define __CUST_SKY_CP_H__


/*****************************************************
    SKY Android 모델 공통 적용사항
    Feature Name : T_SKY_MODEL_TARGET_COMMON
******************************************************/
#ifdef T_SKY_MODEL_TARGET_COMMON


#endif/* T_SKY_MODEL_TARGET_COMMON */


/* ######################################################################### */



/*****************************************************
    SKT/KT 공통 적용사항
    Feature Name : T_SKY_MODEL_TARGET_WCDMA
******************************************************/
#ifdef T_SKY_MODEL_TARGET_WCDMA



#endif/* T_SKY_MODEL_TARGET_WCDMA */


/* ######################################################################### */


/*****************************************************
    SKT 모델 적용사항
    Feature Name : T_SKY_MODEL_TARGET_SKT
******************************************************/
#ifdef T_SKY_MODEL_TARGET_SKT

#ifdef T_SKY_MODEL_TARGET_KT
#error Occured !!  This section is SKT only !!
#endif


#endif/* T_SKY_MODEL_TARGET_SKT */


/* ######################################################################### */


/*****************************************************
    KT 모델 적용사항
    Feature Name : T_SKY_MODEL_TARGET_KT
******************************************************/
#ifdef T_SKY_MODEL_TARGET_KT

#ifdef T_SKY_MODEL_TARGET_SKT
#error Occured !!  This section is KT only !!
#endif


#endif/* T_SKY_MODEL_TARGET_KT */

#endif/* __CUST_SKY_CP_H__ */
