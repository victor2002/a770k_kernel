//=============================================================================
// File       : tdmb_gpio.c
//
// Description: 
//
// Revision History:
//
// Version         Date           Author        Description of Changes
//-----------------------------------------------------------------------------
//  1.0.0       2010/11/02       yschoi         Create
//=============================================================================

#include <linux/kernel.h>
#include <linux/module.h>
#include <mach/gpio.h>

#include "tdmb_comdef.h"
#include "tdmb_dev.h"
#include "tdmb_gpio.h"


#ifdef FEATURE_TDMB_GPIO_INIT
#if defined(CONFIG_EF33_BOARD) || defined(CONFIG_EF34_BOARD) || defined(CONFIG_EF35_BOARD)  || defined(CONFIG_EF40_BOARD) /*yjw*/
#ifdef FEATURE_QTDMB_TSIF_IF
#define DMB_TSIF_CLK      97
#define DMB_TSIF_EN       98
#define DMB_TSIF_DATA     99
#endif /* FEATURE_QTDMB_TSIF_IF */

#ifdef FEATURE_QTDMB_EBI_CMD // EF33S,K EBI2
#define DMB_EBI2_CS       134
#define DMB_EBI2_OE       151
#define DMB_EBI2_WE       157

#define DMB_EBI2_AD0      150
#define DMB_EBI2_AD1      149
#define DMB_EBI2_AD2      148
#define DMB_EBI2_AD3      147
#define DMB_EBI2_AD4      146
#define DMB_EBI2_AD5      145
#define DMB_EBI2_AD6      144
#define DMB_EBI2_AD7      143
#endif /* FEATURE_QTDMB_EBI_CMD */
#endif /* CONFIG_EF33_BOARD */

static uint32_t tdmb_gpio_init_table[] = {
  GPIO_CFG(DMB_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),

#if defined(FEATURE_QTDMB_USE_INC) || defined(FEATURE_QTDMB_USE_FCI)
  GPIO_CFG(DMB_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif

#ifndef FEATURE_TDMB_PMIC_POWER
  GPIO_CFG(DMB_PWR_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif

#ifdef FEATURE_TDMB_SET_ANT_PATH
  GPIO_CFG(DMB_ANT_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif

#ifdef FEATURE_TDMB_PMIC_TCXO_192M
  GPIO_CFG(DMB_XO_SEL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif

#if 0//QUP I2C 사용시 사용하면 안됨 //def FEATURE_QTDMB_I2C_CMD
  GPIO_CFG(DMB_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
  GPIO_CFG(DMB_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif /* FEATURE_QTDMB_I2C_CMD */

#if 0//Not need..  //def FEATURE_QTDMB_TSIF_IF
  GPIO_CFG(DMB_TSIF_CLK, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_TSIF_EN, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_TSIF_DATA, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
#endif /* FEATURE_QTDMB_TSIF_IF */

#ifdef FEATURE_QTDMB_EBI_CMD
  GPIO_CFG(DMB_EBI2_CS, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_OE, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_WE, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD0, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD1, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD2, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD3, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD4, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD5, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD6, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(DMB_EBI2_AD7, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
#endif /* FEATURE_QTDMB_EBI_CMD */
};


void tdmb_gpio_init(void)
{
  int i, rc;

  TDMB_MSG_GPIO("[%s] tdmb_gpio_init!!!\n",__func__);

  for(i = 0; i < ARRAY_SIZE(tdmb_gpio_init_table); i ++)
  {
    rc = gpio_tlmm_config(tdmb_gpio_init_table[i], GPIO_CFG_ENABLE);
    if (rc)
    {
      TDMB_MSG_GPIO("[%s] gpio_tlmm_config(%#x)=%d\n",__func__, tdmb_gpio_init_table[i], rc);
      break;
    }
  }

  TDMB_MSG_GPIO("[%s] tdmb_gpio_init end cnt[%d]!!!\n",__func__, i);
}
#endif /* FEATURE_TDMB_GPIO_INIT */


void tdmb_set_gpio(uint gpio, bool value)
{
#if 1
  gpio_set_value(gpio, value);
  TDMB_MSG_GPIO("[%s] gpio [%d] set [%d]\n", __func__, gpio, value);

#ifdef FEATURE_GPIO_DEBUG
  TDMB_MSG_GPIO("[%s] gpio [%d] get [%d]\n", __func__, gpio, gpio_get_value(gpio));
#endif
#else
  int rc = 0;

  rc = gpio_request(gpio, "tdmb_gpio");
  if (!rc)
  {
    rc = gpio_direction_output(gpio, value);
    TDMB_MSG_GPIO("[%s] gpio [%d] set [%d]\n", __func__, gpio, value);
  }
  else
  {
    TDMB_MSG_GPIO("[%s] gpio_request fail!!!\n", __func__);
  }

#ifdef FEATURE_GPIO_DEBUG
  TDMB_MSG_GPIO("[%s] gpio [%d] get [%d]\n", __func__, gpio, gpio_get_value(gpio));
#endif

  gpio_free(gpio);
#endif // 0
}

