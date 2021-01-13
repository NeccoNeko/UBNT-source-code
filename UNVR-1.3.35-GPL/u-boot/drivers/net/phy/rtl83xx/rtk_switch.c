/*
 * Copyright (C) 2010 Realtek Semiconductor Corp.
 * All Rights Reserved.
 *
 * This program is the proprietary software of Realtek Semiconductor
 * Corporation and/or its licensors, and only be used, duplicated,
 * modified or distributed under the authorized license from Realtek.
 *
 * ANY USE OF THE SOFTWARE OTHER THAN AS AUTHORIZED UNDER
 * THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * $Revision: 79923 $
 * $Date: 2017-06-21 16:40:41 +0800 (週三, 21 六月 2017) $
 *
 * Purpose : RTK switch high-level API
 * Feature : Here is a list of all functions and variables in this module.
 *
 */
#include <common.h>
#include "rtk_switch.h"
#include "rtk_error.h"
#include "port.h"
#include "cpu.h"
#include "stat.h"
#include "led.h"

#include "rtl8367c_asicdrv.h"
#include "rtl8367c_asicdrv_rma.h"
#include "rtl8367c_asicdrv_mirror.h"
#include "rtl8367c_asicdrv_scheduling.h"
#include "rtl8367c_asicdrv_inbwctrl.h"
#include "rtl8367c_asicdrv_port.h"
#include "rtl8367c_asicdrv_mib.h"

static init_state_t init_state = INIT_COMPLETED;

static rtk_switch_halCtrl_t rtl8370b_hal_Ctrl =
{
    /* Switch Chip */
    CHIP_RTL8370B,

    /* Logical to Physical */
    {0, 1, 2, 3, 4, 5, 6, 7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
     8, 9, 10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },

    /* Physical to Logical */
    {UTP_PORT0, UTP_PORT1, UTP_PORT2, UTP_PORT3, UTP_PORT4, UTP_PORT5, UTP_PORT6, UTP_PORT7,
     EXT_PORT0, EXT_PORT1, EXT_PORT2, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT,
     UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT,
     UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT, UNDEFINE_PORT},

    /* Port Type */
    {UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT, UTP_PORT,
     UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT,
     EXT_PORT, EXT_PORT, EXT_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT,
     UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT, UNKNOWN_PORT},

    /* PTP port */
    {1, 1, 1, 1, 1, 1, 1, 1,
     0, 0, 0, 0, 0, 0, 0, 0,
     1, 1, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0 },

    /* Valid port mask */
    ( (0x1 << UTP_PORT0) | (0x1 << UTP_PORT1) | (0x1 << UTP_PORT2) | (0x1 << UTP_PORT3) | (0x1 << UTP_PORT4) | (0x1 << UTP_PORT5) | (0x1 << UTP_PORT6) | (0x1 << UTP_PORT7) | (0x1 << EXT_PORT0) | (0x1 << EXT_PORT1) | (0x1 << EXT_PORT2) ),

    /* Valid UTP port mask */
    ( (0x1 << UTP_PORT0) | (0x1 << UTP_PORT1) | (0x1 << UTP_PORT2) | (0x1 << UTP_PORT3) | (0x1 << UTP_PORT4) | (0x1 << UTP_PORT5) | (0x1 << UTP_PORT6) | (0x1 << UTP_PORT7) ),

    /* Valid EXT port mask */
    ( (0x1 << EXT_PORT0) | (0x1 << EXT_PORT1) | (0x1 << EXT_PORT2) ),

    /* Valid CPU port mask */
    (0x1 << EXT_PORT2),

    /* Minimum physical port number */
    0,

    /* Maxmum physical port number */
    10,

    /* Physical port mask */
    0x7FF,

    /* Combo Logical port ID */
    7,

    /* HSG Logical port ID */
    EXT_PORT1,

    /* SGMII Logical portmask */
    ( (0x1 << EXT_PORT0) | (0x1 << EXT_PORT1) ),

    /* Max Meter ID */
    63,

    /* MAX LUT Address Number 4096 + 64*/
    4160,

    /* Trunk Group Mask */
    0x07
};

static rtk_switch_halCtrl_t *halCtrl = &rtl8370b_hal_Ctrl;

static rtk_uint32 PatchChipData[210][2] =
{
        {0xa436, 0x8028}, {0xa438, 0x6800}, {0xb82e, 0x0001}, {0xa436, 0xb820}, {0xa438, 0x0090}, {0xa436, 0xa012}, {0xa438, 0x0000}, {0xa436, 0xa014}, {0xa438, 0x2c04}, {0xa438, 0x2c6c},
        {0xa438, 0x2c75}, {0xa438, 0x2c77}, {0xa438, 0x1414}, {0xa438, 0x1579}, {0xa438, 0x1536}, {0xa438, 0xc432}, {0xa438, 0x32c0}, {0xa438, 0x42d6}, {0xa438, 0x32b5}, {0xa438, 0x003e},
        {0xa438, 0x614c}, {0xa438, 0x1569}, {0xa438, 0xd705}, {0xa438, 0x318c}, {0xa438, 0x42d6}, {0xa438, 0xd702}, {0xa438, 0x31ef}, {0xa438, 0x42d6}, {0xa438, 0x629c}, {0xa438, 0x2c04},
        {0xa438, 0x653c}, {0xa438, 0x422a}, {0xa438, 0x5d83}, {0xa438, 0xd06a}, {0xa438, 0xd1b0}, {0xa438, 0x1536}, {0xa438, 0xc43a}, {0xa438, 0x32c0}, {0xa438, 0x42d6}, {0xa438, 0x32b5},
        {0xa438, 0x003e}, {0xa438, 0x314a}, {0xa438, 0x42fe}, {0xa438, 0x337b}, {0xa438, 0x02d6}, {0xa438, 0x3063}, {0xa438, 0x0c1b}, {0xa438, 0x22fe}, {0xa438, 0xc435}, {0xa438, 0xd0be},
        {0xa438, 0xd1f7}, {0xa438, 0xe0f0}, {0xa438, 0x1a40}, {0xa438, 0xa320}, {0xa438, 0xd702}, {0xa438, 0x154a}, {0xa438, 0xc434}, {0xa438, 0x32c0}, {0xa438, 0x42d6}, {0xa438, 0x32b5},
        {0xa438, 0x003e}, {0xa438, 0x60ec}, {0xa438, 0x1569}, {0xa438, 0xd705}, {0xa438, 0x619f}, {0xa438, 0xd702}, {0xa438, 0x414f}, {0xa438, 0x2c2e}, {0xa438, 0x610a}, {0xa438, 0xd705},
        {0xa438, 0x5e1f}, {0xa438, 0xc43f}, {0xa438, 0xc88b}, {0xa438, 0xd702}, {0xa438, 0x7fe0}, {0xa438, 0x22f3}, {0xa438, 0xd0a0}, {0xa438, 0xd1b2}, {0xa438, 0xd0c3}, {0xa438, 0xd1c3},
        {0xa438, 0x8d01}, {0xa438, 0x1536}, {0xa438, 0xc438}, {0xa438, 0xe0f0}, {0xa438, 0x1a80}, {0xa438, 0xd706}, {0xa438, 0x60c0}, {0xa438, 0xd710}, {0xa438, 0x409e}, {0xa438, 0xa804},
        {0xa438, 0xad01}, {0xa438, 0x8804}, {0xa438, 0xd702}, {0xa438, 0x32c0}, {0xa438, 0x42d6}, {0xa438, 0x32b5}, {0xa438, 0x003e}, {0xa438, 0x405b}, {0xa438, 0x1576}, {0xa438, 0x7c9c},
        {0xa438, 0x60ec}, {0xa438, 0x1569}, {0xa438, 0xd702}, {0xa438, 0x5d43}, {0xa438, 0x31ef}, {0xa438, 0x02fe}, {0xa438, 0x22d6}, {0xa438, 0x590a}, {0xa438, 0xd706}, {0xa438, 0x5c80},
        {0xa438, 0xd702}, {0xa438, 0x5c44}, {0xa438, 0x3063}, {0xa438, 0x02d6}, {0xa438, 0x5be2}, {0xa438, 0x22fb}, {0xa438, 0xa240}, {0xa438, 0xa104}, {0xa438, 0x8c03}, {0xa438, 0x8178},
        {0xa438, 0xd701}, {0xa438, 0x31ad}, {0xa438, 0x4917}, {0xa438, 0x8102}, {0xa438, 0x2917}, {0xa438, 0xc302}, {0xa438, 0x268a}, {0xa436, 0xA01A}, {0xa438, 0x0000}, {0xa436, 0xA006},
        {0xa438, 0x0fff}, {0xa436, 0xA004}, {0xa438, 0x0689}, {0xa436, 0xA002}, {0xa438, 0x0911}, {0xa436, 0xA000}, {0xa438, 0x7302}, {0xa436, 0xB820}, {0xa438, 0x0010}, {0xa436, 0x8412},
        {0xa438, 0xaf84}, {0xa438, 0x1eaf}, {0xa438, 0x8427}, {0xa438, 0xaf84}, {0xa438, 0x27af}, {0xa438, 0x8427}, {0xa438, 0x0251}, {0xa438, 0x6802}, {0xa438, 0x8427}, {0xa438, 0xaf04},
        {0xa438, 0x0af8}, {0xa438, 0xf9bf}, {0xa438, 0x5581}, {0xa438, 0x0255}, {0xa438, 0x27ef}, {0xa438, 0x310d}, {0xa438, 0x345b}, {0xa438, 0x0fa3}, {0xa438, 0x032a}, {0xa438, 0xe087},
        {0xa438, 0xffac}, {0xa438, 0x2040}, {0xa438, 0xbf56}, {0xa438, 0x7402}, {0xa438, 0x5527}, {0xa438, 0xef31}, {0xa438, 0xef20}, {0xa438, 0xe787}, {0xa438, 0xfee6}, {0xa438, 0x87fd},
        {0xa438, 0xd488}, {0xa438, 0x88bf}, {0xa438, 0x5674}, {0xa438, 0x0254}, {0xa438, 0xe3e0}, {0xa438, 0x87ff}, {0xa438, 0xf720}, {0xa438, 0xe487}, {0xa438, 0xffaf}, {0xa438, 0x847e},
        {0xa438, 0xe087}, {0xa438, 0xffad}, {0xa438, 0x2016}, {0xa438, 0xe387}, {0xa438, 0xfee2}, {0xa438, 0x87fd}, {0xa438, 0xef45}, {0xa438, 0xbf56}, {0xa438, 0x7402}, {0xa438, 0x54e3},
        {0xa438, 0xe087}, {0xa438, 0xfff6}, {0xa438, 0x20e4}, {0xa438, 0x87ff}, {0xa438, 0xfdfc}, {0xa438, 0x0400}, {0xa436, 0xb818}, {0xa438, 0x0407}, {0xa436, 0xb81a}, {0xa438, 0xfffd},
        {0xa436, 0xb81c}, {0xa438, 0xfffd}, {0xa436, 0xb81e}, {0xa438, 0xfffd}, {0xa436, 0xb832}, {0xa438, 0x0001}, {0xb820, 0x0000}, {0xb82e, 0x0000}, {0xa436, 0x8028}, {0xa438, 0x0000}
};

static rtk_api_ret_t _rtk_switch_init_8370b(void)
{
    ret_t retVal;
    rtk_uint32 regData, tmp = 0;
    rtk_uint32 i, prf, counter;
    rtk_uint32 long_link[8] = {0x0210, 0x03e8, 0x0218, 0x03f0, 0x0220, 0x03f8, 0x0208, 0x03e0 };

    if((retVal = rtl8367c_setAsicRegBits(0x1205, 0x0300, 3)) != RT_ERR_OK)
        return retVal;

    for(i=0; i<8; i++)
    {
      if ((retVal = rtl8367c_getAsicPHYOCPReg(i, 0xa420, &regData)) != RT_ERR_OK)
          return retVal;
        tmp = regData & 0x7 ;
       if(tmp == 0x3)
       {
           prf = 1;
           if((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xb83e, 0x6fa9)) != RT_ERR_OK)
              return retVal;
           if((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xb840, 0xa9)) != RT_ERR_OK)
               return retVal;
           for(counter = 0; counter < 10000; counter++); //delay

           if ((retVal = rtl8367c_getAsicPHYOCPReg(i, 0xb820, &regData)) != RT_ERR_OK)
               return retVal;
           tmp = regData | 0x10;
           if ((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xb820, tmp)) != RT_ERR_OK)
               return retVal;
           for(counter = 0; counter < 10000; counter++); //delay
           counter = 0;
           do{
              counter = counter + 1;
              if ((retVal = rtl8367c_getAsicPHYOCPReg(i, 0xb800, &regData)) != RT_ERR_OK)
                   return retVal;
              tmp = regData & 0x40;
              if(tmp != 0)
                break;
           } while (counter < 20);   //Wait for patch ready = 1...
       }
   }
    if ((retVal = rtl8367c_getAsicReg(0x1d01, &regData)) != RT_ERR_OK)
        return retVal;
    tmp = regData;
    tmp = tmp | 0x3BE0; /*Broadcast port enable*/
    tmp = tmp & 0xFFE0; /*Phy_id = 0 */
    if((retVal = rtl8367c_setAsicReg(0x1d01, tmp)) != RT_ERR_OK)
        return retVal;

    for(i=0;i < 210; i++)
    {
        if((retVal = rtl8367c_setAsicPHYOCPReg(0, PatchChipData[i][0], PatchChipData[i][1])) != RT_ERR_OK)
             return retVal;
    }

   if((retVal = rtl8367c_setAsicReg(0x1d01, regData)) != RT_ERR_OK)
        return retVal;

    for(i=0; i < 8; i++)
    {
        if((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xa4b4, long_link[i])) != RT_ERR_OK)
             return retVal;
    }

  if (prf == 0x1)
     {
        for(i=0; i<8; i++)
        {
         if ((retVal = rtl8367c_getAsicPHYOCPReg(i, 0xb820, &regData)) != RT_ERR_OK)
             return retVal;
       tmp = regData & 0xFFEF;
       if ((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xb820, tmp)) != RT_ERR_OK)
             return retVal;

       for(counter = 0; counter < 10000; counter++); //delay

       counter = 0;
       do{
            counter = counter + 1;
            if ((retVal = rtl8367c_getAsicPHYOCPReg(i, 0xb800, &regData)) != RT_ERR_OK)
              return retVal;
            tmp = regData & 0x40;
            if( tmp == 0 )
               break;
       } while (counter < 20);   //Wait for patch ready = 1...
      if ((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xb83e, 0x6f48)) != RT_ERR_OK)
          return retVal;
      if ((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xb840, 0xfa)) != RT_ERR_OK)
          return retVal;
          }
   }

    /*Check phy link status*/
    for(i=0; i<8; i++)
    {
      if ((retVal = rtl8367c_getAsicPHYOCPReg(i, 0xa400, &regData)) != RT_ERR_OK)
          return retVal;
      tmp = regData & 0x800;
        if(tmp == 0x0)
            {
              tmp = regData | 0x200;
          if ((retVal = rtl8367c_setAsicPHYOCPReg(i, 0xa400, tmp)) != RT_ERR_OK)
             return retVal;
            }
    }

  for(counter = 0; counter < 10000; counter++); //delay

  return RT_ERR_OK;
}

static rtk_api_ret_t _rtk_switch_init(void)
{
    rtk_uint32  retVal;
    rtl8367c_rma_t rmaCfg;
    switch_chip_t   switchChip;

    /* probe switch */
    if((retVal = rtk_switch_probe(&switchChip)) != RT_ERR_OK)
        return retVal;

    /* Set initial state */
    if((retVal = rtk_switch_initialState_set(INIT_COMPLETED)) != RT_ERR_OK)
        return retVal;

    /* Initial */
    switch(switchChip)
    {
        case CHIP_RTL8370B:
            if((retVal = _rtk_switch_init_8370b()) != RT_ERR_OK)
                return retVal;
            break;
        default:
            return RT_ERR_CHIP_NOT_FOUND;
    }

    /* Set Old max packet length to 16K */
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_MAX_LENGTH_LIMINT_IPG, RTL8367C_MAX_LENTH_CTRL_MASK, 3)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_MAX_LEN_RX_TX, RTL8367C_MAX_LEN_RX_TX_MASK, 3)) != RT_ERR_OK)
        return retVal;

    /* ACL Mode */
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_ACL_ACCESS_MODE, RTL8367C_ACL_ACCESS_MODE_MASK, 1)) != RT_ERR_OK)
        return retVal;

    /* Max rate */
    if((retVal = rtl8367c_setAsicPortIngressBandwidth(rtk_switch_port_L2P_get(halCtrl->hsg_logical_port), RTL8367C_QOS_RATE_INPUT_MAX_HSG>>3, DISABLED, ENABLED)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPortEgressRate(rtk_switch_port_L2P_get(halCtrl->hsg_logical_port), RTL8367C_QOS_RATE_INPUT_MAX_HSG>>3)) != RT_ERR_OK)
        return retVal;

    if ((retVal = rtl8367c_setAsicPortEgressRateIfg(ENABLED)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicReg(0x03fa, 0x0007)) != RT_ERR_OK)
        return retVal;

    /* Change unknown DA to per port setting */
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_PORT_SECURIT_CTRL_REG, RTL8367C_UNKNOWN_UNICAST_DA_BEHAVE_MASK, 3)) != RT_ERR_OK)
        return retVal;

    /* LUT lookup OP = 1 */
    if ((retVal = rtl8367c_setAsicLutIpLookupMethod(1))!=RT_ERR_OK)
        return retVal;

    /* Set RMA */
    rmaCfg.portiso_leaky = 0;
    rmaCfg.vlan_leaky = 0;
    rmaCfg.keep_format = 0;
    rmaCfg.trap_priority = 0;
    rmaCfg.discard_storm_filter = 0;
    rmaCfg.operation = 0;
    if ((retVal = rtl8367c_setAsicRma(2, &rmaCfg))!=RT_ERR_OK)
        return retVal;

    /* Enable TX Mirror isolation leaky */
    if ((retVal = rtl8367c_setAsicPortMirrorIsolationTxLeaky(ENABLED)) != RT_ERR_OK)
        return retVal;

    /* INT EN */
    if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_IO_MISC_FUNC, RTL8367C_INT_EN_OFFSET, 1)) != RT_ERR_OK)
        return retVal;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_probe
 * Description:
 *      Probe switch
 * Input:
 *      None
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Switch probed
 *      RT_ERR_FAILED   - Switch Unprobed.
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_probe(switch_chip_t *pSwitchChip)
{
    rtk_uint32 retVal;
    rtk_uint32 data, regValue;

    if((retVal = rtl8367c_setAsicReg(0x13C2, 0x0249)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_getAsicReg(0x1300, &data)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_getAsicReg(0x1301, &regValue)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicReg(0x13C2, 0x0000)) != RT_ERR_OK)
        return retVal;

    switch (data)
    {
        case 0x0652:
        case 0x6368:
            *pSwitchChip = CHIP_RTL8370B;
            halCtrl = &rtl8370b_hal_Ctrl;
			printf("%s: CHIP_RTL8370B\n", __func__);
            break;
        default:
			printf("%s: Unsupported chip\n", __func__);
            return RT_ERR_FAILED;
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_initialState_set
 * Description:
 *      Set initial status
 * Input:
 *      state   - Initial state;
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Initialized
 *      RT_ERR_FAILED   - Uninitialized
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_initialState_set(init_state_t state)
{
    if(state >= INIT_STATE_END)
        return RT_ERR_FAILED;

    init_state = state;
    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_initialState_get
 * Description:
 *      Get initial status
 * Input:
 *      None
 * Output:
 *      None
 * Return:
 *      INIT_COMPLETED     - Initialized
 *      INIT_NOT_COMPLETED - Uninitialized
 * Note:
 *
 */
init_state_t rtk_switch_initialState_get(void)
{
    return init_state;
}

/* Function Name:
 *      rtk_switch_port_L2P_get
 * Description:
 *      Get physical port ID
 * Input:
 *      logicalPort       - logical port ID
 * Output:
 *      None
 * Return:
 *      Physical port ID
 * Note:
 *
 */
rtk_uint32 rtk_switch_port_L2P_get(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return UNDEFINE_PHY_PORT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return UNDEFINE_PHY_PORT;

    return (halCtrl->l2p_port[logicalPort]);
}

/* Function Name:
 *      rtk_switch_isExtPort
 * Description:
 *      Check is logical port a Extension port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a EXT port
 *      RT_ERR_FAILED   - Port ID is not a EXT port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_isExtPort(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if(halCtrl->log_port_type[logicalPort] == EXT_PORT)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtk_switch_isHsgPort
 * Description:
 *      Check is logical port a HSG port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a HSG port
 *      RT_ERR_FAILED   - Port ID is not a HSG port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_isHsgPort(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if(logicalPort == halCtrl->hsg_logical_port)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtk_switch_logicalPortCheck
 * Description:
 *      Check logical port ID.
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is correct
 *      RT_ERR_FAILED   - Port ID is not correct
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_logicalPortCheck(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if(halCtrl->l2p_port[logicalPort] == 0xFF)
        return RT_ERR_FAILED;

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_isSgmiiPort
 * Description:
 *      Check is logical port a SGMII port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a SGMII port
 *      RT_ERR_FAILED   - Port ID is not a SGMII port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_isSgmiiPort(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if( ((0x01 << logicalPort) & halCtrl->sg_logical_portmask) != 0)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtk_switch_isCPUPort
 * Description:
 *      Check is logical port a CPU port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a CPU port
 *      RT_ERR_FAILED   - Port ID is not a CPU port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_isCPUPort(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if( ((0x01 << logicalPort) & halCtrl->valid_cpu_portmask) != 0)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtk_switch_isUtpPort
 * Description:
 *      Check is logical port a UTP port
 * Input:
 *      logicalPort     - logical port ID
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK       - Port ID is a UTP port
 *      RT_ERR_FAILED   - Port ID is not a UTP port
 *      RT_ERR_NOT_INIT - Not Initialize
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_isUtpPort(rtk_port_t logicalPort)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(logicalPort >= RTK_SWITCH_PORT_NUM)
        return RT_ERR_FAILED;

    if(halCtrl->log_port_type[logicalPort] == UTP_PORT)
        return RT_ERR_OK;
    else
        return RT_ERR_FAILED;
}

/* Function Name:
 *      rtk_switch_isPortMaskValid
 * Description:
 *      Check portmask is valid or not
 * Input:
 *      pPmask       - logical port mask
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - port mask is valid
 *      RT_ERR_FAILED       - port mask is not valid
 *      RT_ERR_NOT_INIT     - Not Initialize
 *      RT_ERR_NULL_POINTER - Null pointer
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_isPortMaskValid(rtk_portmask_t *pPmask)
{
    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(NULL == pPmask)
        return RT_ERR_NULL_POINTER;

    if( (pPmask->bits[0] | halCtrl->valid_portmask) != halCtrl->valid_portmask )
        return RT_ERR_FAILED;
    else
        return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_portmask_L2P_get
 * Description:
 *      Get physicl portmask from logical portmask
 * Input:
 *      pLogicalPmask       - logical port mask
 * Output:
 *      pPhysicalPortmask   - physical port mask
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_NOT_INIT     - Not Initialize
 *      RT_ERR_NULL_POINTER - Null pointer
 *      RT_ERR_PORT_MASK    - Error port mask
 * Note:
 *
 */
rtk_api_ret_t rtk_switch_portmask_L2P_get(rtk_portmask_t *pLogicalPmask, rtk_uint32 *pPhysicalPortmask)
{
    rtk_uint32 log_port, phy_port;

    if(init_state != INIT_COMPLETED)
        return RT_ERR_NOT_INIT;

    if(NULL == pLogicalPmask)
        return RT_ERR_NULL_POINTER;

    if(NULL == pPhysicalPortmask)
        return RT_ERR_NULL_POINTER;

    if(rtk_switch_isPortMaskValid(pLogicalPmask) != RT_ERR_OK)
        return RT_ERR_PORT_MASK;

    /* reset physical port mask */
    *pPhysicalPortmask = 0;

    RTK_PORTMASK_SCAN((*pLogicalPmask), log_port)
    {
        phy_port = rtk_switch_port_L2P_get((rtk_port_t)log_port);
        *pPhysicalPortmask |= (0x0001 << phy_port);
    }

    return RT_ERR_OK;
}

/* Function Name:
 *      rtk_switch_init
 * Description:
 *      Set chip to default configuration enviroment
 * Input:
 *      None
 * Output:
 *      None
 * Return:
 *      RT_ERR_OK           - OK
 *      RT_ERR_FAILED       - Failed
 *      RT_ERR_SMI          - SMI access error
 * Note:
 *      The API can set chip registers to default configuration for different release chip model.
 */

rtk_api_ret_t rtk_switch_init(void)
{
    printf("%s: %d\n", __func__, _rtk_switch_init());
}

int config_led(void) {
    rtk_api_ret_t retVal;
	rtk_portmask_t portmask;

    if((retVal = rtk_led_operation_set(LED_OP_SERIAL)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtk_led_OutputEnable_set(ENABLED)) != RT_ERR_OK)
        return retVal;

    RTK_PORTMASK_CLEAR(portmask);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT0);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT1);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT2);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT3);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT4);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT5);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT6);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT7);

    // only use LED group 0 & 1
    if((retVal = rtk_led_serialModePortmask_set(SERIAL_LED_0_1, &portmask)) != RT_ERR_OK)
        return retVal;

    // LED group 0:1000/Act.  group1:speed10/100/Act
    // group 0: Green LED
    // group 1: Orange LED
    if((retVal = rtk_led_groupConfig_set(LED_GROUP_0, LED_CONFIG_SPD1000ACT)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtk_led_groupConfig_set(LED_GROUP_1, LED_CONFIG_SPD10010ACT)) != RT_ERR_OK)
        return retVal;

    // Set Active High
    if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_SERI_LED_ACT_LOW_OFFSET, 0)) !=  RT_ERR_OK)
        return retVal;

    // output 8 ports data
    if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_LED_SERIAL_OUT_MODE_OFFSET, 1)) !=  RT_ERR_OK)
        return retVal;

    // 10: Low port LED0->Low port LED1->Low port LED2->High port LED0->High port LED1->High port LED2
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SERIAL_LED_CTRL, RTL8367C_SERIAL_LED_SHIFT_SEQUENCE_MASK, 2)) != RT_ERR_OK)
        return retVal;

    return retVal;
}

int config_led_udm_se(void) {
    rtk_api_ret_t retVal;
	rtk_portmask_t portmask;

    if((retVal = rtk_led_operation_set(LED_OP_SERIAL)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtk_led_OutputEnable_set(ENABLED)) != RT_ERR_OK)
        return retVal;

    RTK_PORTMASK_CLEAR(portmask);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT0);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT1);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT2);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT3);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT4);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT5);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT6);
    RTK_PORTMASK_PORT_SET(portmask, UTP_PORT7);

    // only use LED group 0
    if((retVal = rtk_led_serialModePortmask_set(SERIAL_LED_0, &portmask)) != RT_ERR_OK)
        return retVal;

    // Set Active High
    if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_SERI_LED_ACT_LOW_OFFSET, 0)) !=  RT_ERR_OK)
        return retVal;

    // output 8 ports data
    if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_LED_SERIAL_OUT_MODE_OFFSET, 1)) !=  RT_ERR_OK)
        return retVal;

    return retVal;
}

int config_led_mappings_onep_rm(void) {
	rtk_api_ret_t retVal;

    // RTL8367C_REG_P0_LED_MUX  ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P0_LED_MUX, RTL8367C_CFG_P0_LED0_MUX_MASK, 0x00)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P0_LED_MUX, RTL8367C_CFG_P0_LED1_MUX_MASK, 0x01)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P0_LED_MUX, RTL8367C_CFG_P0_LED2_MUX_MASK, 0x02)) != RT_ERR_OK)
        return retVal;


    // RTL8367C_REG_P1_LED_MUX ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P1_LED_MUX, RTL8367C_CFG_P1_LED0_MUX_MASK, 0x03)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P1_LED_MUX, RTL8367C_CFG_P1_LED1_MUX_MASK, 0x04)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P1_LED_MUX, RTL8367C_CFG_P1_LED2_MUX_MASK, 0x05)) != RT_ERR_OK)
        return retVal;

    // RTL8367C_REG_P2_LED_MUX ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P2_LED_MUX, RTL8367C_CFG_P2_LED0_MUX_MASK, 0x06)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P2_LED_MUX, RTL8367C_CFG_P2_LED1_MUX_MASK, 0x07)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P2_LED_MUX, RTL8367C_CFG_P2_LED2_MUX_MASK, 0x08)) != RT_ERR_OK)
        return retVal;


    // RTL8367C_REG_P3_LED_MUX  ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P3_LED_MUX, RTL8367C_CFG_P3_LED0_MUX_MASK, 0x09)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P3_LED_MUX, RTL8367C_CFG_P3_LED1_MUX_MASK, 0x0A)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P3_LED_MUX, RTL8367C_CFG_P3_LED2_MUX_MASK, 0x0B)) != RT_ERR_OK)
        return retVal;


    // RTL8367C_REG_P4_LED_MUX  ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P4_LED_MUX, RTL8367C_CFG_P4_LED0_MUX_MASK, 0x0C)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P4_LED_MUX, RTL8367C_CFG_P4_LED1_MUX_MASK, 0x0D)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P4_LED_MUX, RTL8367C_CFG_P4_LED2_MUX_MASK, 0x0E)) != RT_ERR_OK)
        return retVal;

    // RTL8367C_REG_P5_LED_MUX  ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P5_LED_MUX, RTL8367C_CFG_P5_LED0_MUX_MASK, 0x0F)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P5_LED_MUX, RTL8367C_CFG_P5_LED1_MUX_MASK, 0x10)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P5_LED_MUX, RTL8367C_CFG_P5_LED2_MUX_MASK, 0x11)) != RT_ERR_OK)
        return retVal;

    // RTL8367C_REG_P6_LED_MUX  ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P6_LED_MUX, RTL8367C_CFG_P6_LED0_MUX_MASK, 0x12)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P6_LED_MUX, RTL8367C_CFG_P6_LED1_MUX_MASK, 0x13)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P6_LED_MUX, RTL8367C_CFG_P6_LED2_MUX_MASK, 0x14)) != RT_ERR_OK)
        return retVal;

    // RTL8367C_REG_P7_LED_MUX  ==> 
    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P7_LED_MUX, RTL8367C_CFG_P7_LED0_MUX_MASK, 0x15)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P7_LED_MUX, RTL8367C_CFG_P7_LED1_MUX_MASK, 0x16)) != RT_ERR_OK)
        return retVal;

    if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_P7_LED_MUX, RTL8367C_CFG_P7_LED2_MUX_MASK, 0x17)) != RT_ERR_OK)
        return retVal;

    return retVal;
}

#define GPIO_SLED_OE 42
// HW default pullup
// pulllow to enable it
int enable_sled_oe() {
    int retVal=0;
#if 1
    retVal = gpio_get_value(GPIO_SLED_OE);
    if(retVal<0)
        printf("%s:: gpio(%d) gpio_get_value(1) failed\n",__func__, GPIO_SLED_OE);
    printf("%s:: gpio(%d) gpio_get_value(1) %d\n",__func__, GPIO_SLED_OE, retVal);
#endif

    retVal = gpio_request(GPIO_SLED_OE, "rtl8370_sled_oe");
    if(retVal<0)
        printf("%s:: gpio(%d) request failed\n",__func__, GPIO_SLED_OE);

    // output pulllow to enable OE
    retVal= gpio_direction_output(GPIO_SLED_OE, 0);
    if(retVal<0)
        printf("%s:: gpio(%d) set direction output failed\n",__func__, GPIO_SLED_OE);

#if 1
    retVal = gpio_get_value(GPIO_SLED_OE);
    if(retVal<0)
        printf("%s:: gpio(%d) gpio_get_value(2) failed\n",__func__, GPIO_SLED_OE);
    printf("%s:: gpio(%d) gpio_get_value(2) %d\n",__func__, GPIO_SLED_OE, retVal);
#endif
}

void config_led_mode(rtk_led_group_t group, rtk_led_force_mode_t mode) {
    rtk_led_modeForce_set(UTP_PORT0, group, mode);
    rtk_led_modeForce_set(UTP_PORT1, group, mode);
    rtk_led_modeForce_set(UTP_PORT2, group, mode);
    rtk_led_modeForce_set(UTP_PORT3, group, mode);
    rtk_led_modeForce_set(UTP_PORT4, group, mode);
    rtk_led_modeForce_set(UTP_PORT5, group, mode);
    rtk_led_modeForce_set(UTP_PORT6, group, mode);
    rtk_led_modeForce_set(UTP_PORT7, group, mode);
}

static int
do_switch(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    rtk_api_ret_t retVal;
	rtk_port_mac_ability_t pPortability;
	rtk_mode_ext_t mode;
	rtk_portmask_t portmask;

	eth_init();
	rtk_switch_init();
	rtk_port_phyEnableAll_set(ENABLED);

#ifdef CONFIG_TARGET_ALPINE_V2_64_UBNT_PLUS_RM
    printf("config_led!!\n");
    retVal = config_led();
    if(retVal != RT_ERR_OK) {
        printf("config_led error\n");
    }

    retVal = config_led_mappings_onep_rm();
    if(retVal != RT_ERR_OK) {
        printf("config_led_mappings() error with ubnt_plus_rm\n");
    }

    enable_sled_oe();

// for test all leds
#define test_all_green 0
#define test_all_orange 0
#if test_all_green
    // test green
    config_led_mode(LED_GROUP_0, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_1, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_0, LED_FORCE_ON);
#elif test_all_orange
    // test orange
    config_led_mode(LED_GROUP_0, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_1, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_1, LED_FORCE_ON);
#endif
#endif


#ifdef CONFIG_TARGET_ALPINE_V2_64_UBNT_UDM_PRO_SE

    printf("config_led_udm_se!!\n");
    retVal = config_led_udm_se();
    if(retVal != RT_ERR_OK) {
        printf("config_led_udm_se error\n");
    }

    retVal = config_led_mappings_onep_rm();
    if(retVal != RT_ERR_OK) {
        printf("config_led_mappings() error with ubnt_plus_rm\n");
    }

    enable_sled_oe();

// for test all leds
#define test_all_green 0
#define test_all_orange 0
#if test_all_green
    // test green
    config_led_mode(LED_GROUP_0, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_1, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_0, LED_FORCE_ON);
#elif test_all_orange
    // test orange
    config_led_mode(LED_GROUP_0, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_1, LED_FORCE_OFF);
    config_led_mode(LED_GROUP_1, LED_FORCE_ON);
#endif

#endif

#ifdef CONFIG_RTL83XX_SGMII
	/* setup GMAC1, GMAC2 as interface SGMII/1000Base-X */
	mode = MODE_EXT_1000X;
	pPortability.speed = PORT_SPEED_1000M;
#elif defined (CONFIG_RTL83XX_HS_SGMII)
	/* setup GMAC2 as interface HS-SGMII */
	mode = MODE_EXT_HSGMII;
	pPortability.speed = PORT_SPEED_2500M;
#elif defined (CONFIG_RTL83XX_RGMII)
	mode = MODE_EXT_RGMII;
	pPortability.speed = PORT_SPEED_1000M;
#endif

	pPortability.forcemode = MAC_FORCE;
	pPortability.duplex = PORT_FULL_DUPLEX;
	pPortability.link = PORT_LINKUP;
	pPortability.nway = DISABLED;
	pPortability.txpause = ENABLED;
	pPortability.rxpause = ENABLED;

#ifdef CONFIG_RTL83XX_SGMII
	retVal = rtk_port_macForceLinkExt_set(EXT_PORT0, mode, &pPortability);
	retVal = rtk_port_macForceLinkExt_set(EXT_PORT1, mode, &pPortability);
#elif defined (CONFIG_RTL83XX_HS_SGMII) || defined (CONFIG_RTL83XX_RGMII)
	retVal = rtk_port_macForceLinkExt_set(EXT_PORT1, mode, &pPortability);
	rtk_port_rgmiiDelayExt_set(EXT_PORT1, 1, 6);
#endif

	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT0);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT1);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT2);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT3);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT4);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT5);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT6);
	RTK_PORTMASK_PORT_SET(portmask, UTP_PORT7);

	return 0;
}

U_BOOT_CMD(rtl83xx, 1, 0, do_switch,
	"initialize rtl83xx switch device", ""
);
