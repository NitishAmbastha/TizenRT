/*
 * stm32l4_dsi.h
 *
 *  Created on: Aug 26, 2019
 *      Author: calebkang
 */

#ifndef OS_ARCH_ARM_SRC_STM32L4_STM32L4_DSI_H_
#define OS_ARCH_ARM_SRC_STM32L4_STM32L4_DSI_H_

#define __IO volatile

typedef struct
{
  __IO uint32_t VR;             /*!< DSI Host Version Register,                                 Address offset: 0x00      */
  __IO uint32_t CR;             /*!< DSI Host Control Register,                                 Address offset: 0x04      */
  __IO uint32_t CCR;            /*!< DSI HOST Clock Control Register,                           Address offset: 0x08      */
  __IO uint32_t LVCIDR;         /*!< DSI Host LTDC VCID Register,                               Address offset: 0x0C      */
  __IO uint32_t LCOLCR;         /*!< DSI Host LTDC Color Coding Register,                       Address offset: 0x10      */
  __IO uint32_t LPCR;           /*!< DSI Host LTDC Polarity Configuration Register,             Address offset: 0x14      */
  __IO uint32_t LPMCR;          /*!< DSI Host Low-Power Mode Configuration Register,            Address offset: 0x18      */
  uint32_t      RESERVED0[4];   /*!< Reserved, 0x1C - 0x2B                                                                */
  __IO uint32_t PCR;            /*!< DSI Host Protocol Configuration Register,                  Address offset: 0x2C      */
  __IO uint32_t GVCIDR;         /*!< DSI Host Generic VCID Register,                            Address offset: 0x30      */
  __IO uint32_t MCR;            /*!< DSI Host Mode Configuration Register,                      Address offset: 0x34      */
  __IO uint32_t VMCR;           /*!< DSI Host Video Mode Configuration Register,                Address offset: 0x38      */
  __IO uint32_t VPCR;           /*!< DSI Host Video Packet Configuration Register,              Address offset: 0x3C      */
  __IO uint32_t VCCR;           /*!< DSI Host Video Chunks Configuration Register,              Address offset: 0x40      */
  __IO uint32_t VNPCR;          /*!< DSI Host Video Null Packet Configuration Register,         Address offset: 0x44      */
  __IO uint32_t VHSACR;         /*!< DSI Host Video HSA Configuration Register,                 Address offset: 0x48      */
  __IO uint32_t VHBPCR;         /*!< DSI Host Video HBP Configuration Register,                 Address offset: 0x4C      */
  __IO uint32_t VLCR;           /*!< DSI Host Video Line Configuration Register,                Address offset: 0x50      */
  __IO uint32_t VVSACR;         /*!< DSI Host Video VSA Configuration Register,                 Address offset: 0x54      */
  __IO uint32_t VVBPCR;         /*!< DSI Host Video VBP Configuration Register,                 Address offset: 0x58      */
  __IO uint32_t VVFPCR;         /*!< DSI Host Video VFP Configuration Register,                 Address offset: 0x5C      */
  __IO uint32_t VVACR;          /*!< DSI Host Video VA Configuration Register,                  Address offset: 0x60      */
  __IO uint32_t LCCR;           /*!< DSI Host LTDC Command Configuration Register,              Address offset: 0x64      */
  __IO uint32_t CMCR;           /*!< DSI Host Command Mode Configuration Register,              Address offset: 0x68      */
  __IO uint32_t GHCR;           /*!< DSI Host Generic Header Configuration Register,            Address offset: 0x6C      */
  __IO uint32_t GPDR;           /*!< DSI Host Generic Payload Data Register,                    Address offset: 0x70      */
  __IO uint32_t GPSR;           /*!< DSI Host Generic Packet Status Register,                   Address offset: 0x74      */
  __IO uint32_t TCCR[6];        /*!< DSI Host Timeout Counter Configuration Register,           Address offset: 0x78-0x8F */
  uint32_t      RESERVED1;      /*!< Reserved, 0x90                                                                       */
  __IO uint32_t CLCR;           /*!< DSI Host Clock Lane Configuration Register,                Address offset: 0x94      */
  __IO uint32_t CLTCR;          /*!< DSI Host Clock Lane Timer Configuration Register,          Address offset: 0x98      */
  __IO uint32_t DLTCR;          /*!< DSI Host Data Lane Timer Configuration Register,           Address offset: 0x9C      */
  __IO uint32_t PCTLR;          /*!< DSI Host PHY Control Register,                             Address offset: 0xA0      */
  __IO uint32_t PCONFR;         /*!< DSI Host PHY Configuration Register,                       Address offset: 0xA4      */
  __IO uint32_t PUCR;           /*!< DSI Host PHY ULPS Control Register,                        Address offset: 0xA8      */
  __IO uint32_t PTTCR;          /*!< DSI Host PHY TX Triggers Configuration Register,           Address offset: 0xAC      */
  __IO uint32_t PSR;            /*!< DSI Host PHY Status Register,                              Address offset: 0xB0      */
  uint32_t      RESERVED2[2];   /*!< Reserved, 0xB4 - 0xBB                                                                */
  __IO uint32_t ISR[2];         /*!< DSI Host Interrupt & Status Register,                      Address offset: 0xBC-0xC3 */
  __IO uint32_t IER[2];         /*!< DSI Host Interrupt Enable Register,                        Address offset: 0xC4-0xCB */
  uint32_t      RESERVED3[3];   /*!< Reserved, 0xD0 - 0xD7                                                                */
  __IO uint32_t FIR[2];         /*!< DSI Host Force Interrupt Register,                         Address offset: 0xD8-0xDF */
  uint32_t      RESERVED4[8];   /*!< Reserved, 0xE0 - 0xFF                                                                */
  __IO uint32_t VSCR;           /*!< DSI Host Video Shadow Control Register,                    Address offset: 0x100     */
  uint32_t      RESERVED5[2];   /*!< Reserved, 0x104 - 0x10B                                                              */
  __IO uint32_t LCVCIDR;        /*!< DSI Host LTDC Current VCID Register,                       Address offset: 0x10C     */
  __IO uint32_t LCCCR;          /*!< DSI Host LTDC Current Color Coding Register,               Address offset: 0x110     */
  uint32_t      RESERVED6;      /*!< Reserved, 0x114                                                                      */
  __IO uint32_t LPMCCR;         /*!< DSI Host Low-power Mode Current Configuration Register,    Address offset: 0x118     */
  uint32_t      RESERVED7[7];   /*!< Reserved, 0x11C - 0x137                                                              */
  __IO uint32_t VMCCR;          /*!< DSI Host Video Mode Current Configuration Register,        Address offset: 0x138     */
  __IO uint32_t VPCCR;          /*!< DSI Host Video Packet Current Configuration Register,      Address offset: 0x13C     */
  __IO uint32_t VCCCR;          /*!< DSI Host Video Chuncks Current Configuration Register,     Address offset: 0x140     */
  __IO uint32_t VNPCCR;         /*!< DSI Host Video Null Packet Current Configuration Register, Address offset: 0x144     */
  __IO uint32_t VHSACCR;        /*!< DSI Host Video HSA Current Configuration Register,         Address offset: 0x148     */
  __IO uint32_t VHBPCCR;        /*!< DSI Host Video HBP Current Configuration Register,         Address offset: 0x14C     */
  __IO uint32_t VLCCR;          /*!< DSI Host Video Line Current Configuration Register,        Address offset: 0x150     */
  __IO uint32_t VVSACCR;        /*!< DSI Host Video VSA Current Configuration Register,         Address offset: 0x154     */
  __IO uint32_t VVBPCCR;        /*!< DSI Host Video VBP Current Configuration Register,         Address offset: 0x158     */
  __IO uint32_t VVFPCCR;        /*!< DSI Host Video VFP Current Configuration Register,         Address offset: 0x15C     */
  __IO uint32_t VVACCR;         /*!< DSI Host Video VA Current Configuration Register,          Address offset: 0x160     */
  uint32_t      RESERVED8[167]; /*!< Reserved, 0x164 - 0x3FF                                                              */
  __IO uint32_t WCFGR;          /*!< DSI Wrapper Configuration Register,                        Address offset: 0x400     */
  __IO uint32_t WCR;            /*!< DSI Wrapper Control Register,                              Address offset: 0x404     */
  __IO uint32_t WIER;           /*!< DSI Wrapper Interrupt Enable Register,                     Address offset: 0x408     */
  __IO uint32_t WISR;           /*!< DSI Wrapper Interrupt and Status Register,                 Address offset: 0x40C     */
  __IO uint32_t WIFCR;          /*!< DSI Wrapper Interrupt Flag Clear Register,                 Address offset: 0x410     */
  uint32_t      RESERVED9;      /*!< Reserved, 0x414                                                                      */
  __IO uint32_t WPCR[5];        /*!< DSI Wrapper PHY Configuration Register,                    Address offset: 0x418-0x42B */
  uint32_t      RESERVED10;     /*!< Reserved, 0x42C                                                                      */
  __IO uint32_t WRPCR;          /*!< DSI Wrapper Regulator and PLL Control Register,            Address offset: 0x430     */
} DSI_TypeDef;


typedef enum
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;


typedef enum
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))


/**
  * @brief  DSI Init Structure definition
  */
typedef struct
{
  uint32_t AutomaticClockLaneControl;    /*!< Automatic clock lane control
                                              This parameter can be any value of @ref DSI_Automatic_Clk_Lane_Control */

  uint32_t TXEscapeCkdiv;                /*!< TX Escape clock division
                                              The values 0 and 1 stop the TX_ESC clock generation                    */

  uint32_t NumberOfLanes;                /*!< Number of lanes
                                              This parameter can be any value of @ref DSI_Number_Of_Lanes            */

} DSI_InitTypeDef;

/**
  * @brief  DSI PLL Clock structure definition
  */
typedef struct
{
  uint32_t PLLNDIV;                      /*!< PLL Loop Division Factor
                                              This parameter must be a value between 10 and 125   */

  uint32_t PLLIDF;                       /*!< PLL Input Division Factor
                                              This parameter can be any value of @ref DSI_PLL_IDF */

  uint32_t PLLODF;                       /*!< PLL Output Division Factor
                                              This parameter can be any value of @ref DSI_PLL_ODF */

} DSI_PLLInitTypeDef;

/**
  * @brief  DSI Video mode configuration
  */
typedef struct
{
  uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                 */

  uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding           */

  uint32_t LooselyPacked;                /*!< Enable or disable loosely packed stream (needed only when using
                                              18-bit configuration).
                                              This parameter can be any value of @ref DSI_LooselyPacked          */

  uint32_t Mode;                         /*!< Video mode type
                                              This parameter can be any value of @ref DSI_Video_Mode_Type        */

  uint32_t PacketSize;                   /*!< Video packet size                                                  */

  uint32_t NumberOfChunks;               /*!< Number of chunks                                                   */

  uint32_t NullPacketSize;               /*!< Null packet size                                                   */

  uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity         */

  uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity  */

  uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity   */

  uint32_t HorizontalSyncActive;         /*!< Horizontal synchronism active duration (in lane byte clock cycles) */

  uint32_t HorizontalBackPorch;          /*!< Horizontal back-porch duration (in lane byte clock cycles)         */

  uint32_t HorizontalLine;               /*!< Horizontal line duration (in lane byte clock cycles)               */

  uint32_t VerticalSyncActive;           /*!< Vertical synchronism active duration                               */

  uint32_t VerticalBackPorch;            /*!< Vertical back-porch duration                                       */

  uint32_t VerticalFrontPorch;           /*!< Vertical front-porch duration                                      */

  uint32_t VerticalActive;               /*!< Vertical active duration                                           */

  uint32_t LPCommandEnable;              /*!< Low-power command enable
                                              This parameter can be any value of @ref DSI_LP_Command             */

  uint32_t LPLargestPacketSize;          /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VSA, VBP and VFP regions                  */

  uint32_t LPVACTLargestPacketSize;      /*!< The size, in bytes, of the low power largest packet that
                                              can fit in a line during VACT region                               */

  uint32_t LPHorizontalFrontPorchEnable; /*!< Low-power horizontal front-porch enable
                                              This parameter can be any value of @ref DSI_LP_HFP                 */

  uint32_t LPHorizontalBackPorchEnable;  /*!< Low-power horizontal back-porch enable
                                              This parameter can be any value of @ref DSI_LP_HBP                 */

  uint32_t LPVerticalActiveEnable;       /*!< Low-power vertical active enable
                                              This parameter can be any value of @ref DSI_LP_VACT                */

  uint32_t LPVerticalFrontPorchEnable;   /*!< Low-power vertical front-porch enable
                                              This parameter can be any value of @ref DSI_LP_VFP                 */

  uint32_t LPVerticalBackPorchEnable;    /*!< Low-power vertical back-porch enable
                                              This parameter can be any value of @ref DSI_LP_VBP                 */

  uint32_t LPVerticalSyncActiveEnable;   /*!< Low-power vertical sync active enable
                                              This parameter can be any value of @ref DSI_LP_VSYNC               */

  uint32_t FrameBTAAcknowledgeEnable;    /*!< Frame bus-turn-around acknowledge enable
                                              This parameter can be any value of @ref DSI_FBTA_acknowledge       */

} DSI_VidCfgTypeDef;

/**
  * @brief  DSI Adapted command mode configuration
  */
typedef struct
{
  uint32_t VirtualChannelID;             /*!< Virtual channel ID                                                */

  uint32_t ColorCoding;                  /*!< Color coding for LTDC interface
                                              This parameter can be any value of @ref DSI_Color_Coding          */

  uint32_t CommandSize;                  /*!< Maximum allowed size for an LTDC write memory command, measured in
                                              pixels. This parameter can be any value between 0x00 and 0xFFFFU   */

  uint32_t TearingEffectSource;          /*!< Tearing effect source
                                              This parameter can be any value of @ref DSI_TearingEffectSource   */

  uint32_t TearingEffectPolarity;        /*!< Tearing effect pin polarity
                                              This parameter can be any value of @ref DSI_TearingEffectPolarity */

  uint32_t HSPolarity;                   /*!< HSYNC pin polarity
                                              This parameter can be any value of @ref DSI_HSYNC_Polarity        */

  uint32_t VSPolarity;                   /*!< VSYNC pin polarity
                                              This parameter can be any value of @ref DSI_VSYNC_Active_Polarity */

  uint32_t DEPolarity;                   /*!< Data Enable pin polarity
                                              This parameter can be any value of @ref DSI_DATA_ENABLE_Polarity  */

  uint32_t VSyncPol;                     /*!< VSync edge on which the LTDC is halted
                                              This parameter can be any value of @ref DSI_Vsync_Polarity        */

  uint32_t AutomaticRefresh;             /*!< Automatic refresh mode
                                              This parameter can be any value of @ref DSI_AutomaticRefresh      */

  uint32_t TEAcknowledgeRequest;         /*!< Tearing Effect Acknowledge Request Enable
                                              This parameter can be any value of @ref DSI_TE_AcknowledgeRequest */

} DSI_CmdCfgTypeDef;

/**
  * @brief  DSI command transmission mode configuration
  */
typedef struct
{
  uint32_t LPGenShortWriteNoP;           /*!< Generic Short Write Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteNoP  */

  uint32_t LPGenShortWriteOneP;          /*!< Generic Short Write One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteOneP */

  uint32_t LPGenShortWriteTwoP;          /*!< Generic Short Write Two parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortWriteTwoP */

  uint32_t LPGenShortReadNoP;            /*!< Generic Short Read Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadNoP   */

  uint32_t LPGenShortReadOneP;           /*!< Generic Short Read One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadOneP  */

  uint32_t LPGenShortReadTwoP;           /*!< Generic Short Read Two parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenShortReadTwoP  */

  uint32_t LPGenLongWrite;               /*!< Generic Long Write Transmission
                                              This parameter can be any value of @ref DSI_LP_LPGenLongWrite      */

  uint32_t LPDcsShortWriteNoP;           /*!< DCS Short Write Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortWriteNoP  */

  uint32_t LPDcsShortWriteOneP;          /*!< DCS Short Write One parameter Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortWriteOneP */

  uint32_t LPDcsShortReadNoP;            /*!< DCS Short Read Zero parameters Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsShortReadNoP   */

  uint32_t LPDcsLongWrite;               /*!< DCS Long Write Transmission
                                              This parameter can be any value of @ref DSI_LP_LPDcsLongWrite      */

  uint32_t LPMaxReadPacket;              /*!< Maximum Read Packet Size Transmission
                                              This parameter can be any value of @ref DSI_LP_LPMaxReadPacket     */

  uint32_t AcknowledgeRequest;           /*!< Acknowledge Request Enable
                                              This parameter can be any value of @ref DSI_AcknowledgeRequest     */

} DSI_LPCmdTypeDef;

/**
  * @brief  DSI PHY Timings definition
  */
typedef struct
{
  uint32_t ClockLaneHS2LPTime;           /*!< The maximum time that the D-PHY clock lane takes to go from high-speed
                                              to low-power transmission                                              */

  uint32_t ClockLaneLP2HSTime;           /*!< The maximum time that the D-PHY clock lane takes to go from low-power
                                              to high-speed transmission                                             */

  uint32_t DataLaneHS2LPTime;            /*!< The maximum time that the D-PHY data lanes takes to go from high-speed
                                              to low-power transmission                                              */

  uint32_t DataLaneLP2HSTime;            /*!< The maximum time that the D-PHY data lanes takes to go from low-power
                                              to high-speed transmission                                             */

  uint32_t DataLaneMaxReadTime;          /*!< The maximum time required to perform a read command */

  uint32_t StopWaitTime;                 /*!< The minimum wait period to request a High-Speed transmission after the
                                              Stop state                                                             */

} DSI_PHY_TimerTypeDef;

/**
  * @brief  DSI HOST Timeouts definition
  */
typedef struct
{
  uint32_t TimeoutCkdiv;                 /*!< Time-out clock division                                  */

  uint32_t HighSpeedTransmissionTimeout; /*!< High-speed transmission time-out                         */

  uint32_t LowPowerReceptionTimeout;     /*!< Low-power reception time-out                             */

  uint32_t HighSpeedReadTimeout;         /*!< High-speed read time-out                                 */

  uint32_t LowPowerReadTimeout;          /*!< Low-power read time-out                                  */

  uint32_t HighSpeedWriteTimeout;        /*!< High-speed write time-out                                */

  uint32_t HighSpeedWritePrespMode;      /*!< High-speed write presp mode
                                              This parameter can be any value of @ref DSI_HS_PrespMode */

  uint32_t LowPowerWriteTimeout;         /*!< Low-speed write time-out                                 */

  uint32_t BTATimeout;                   /*!< BTA time-out                                             */

} DSI_HOST_TimeoutTypeDef;

/**
  * @brief  DSI States Structure definition
  */
typedef enum
{
  HAL_DSI_STATE_RESET   = 0x00U,
  HAL_DSI_STATE_READY   = 0x01U,
  HAL_DSI_STATE_ERROR   = 0x02U,
  HAL_DSI_STATE_BUSY    = 0x03U,
  HAL_DSI_STATE_TIMEOUT = 0x04U
} HAL_DSI_StateTypeDef;

/**
  * @brief  DSI Handle Structure definition
  */
#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
typedef struct __DSI_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */
{
  DSI_TypeDef               *Instance;    /*!< Register base address      */
  DSI_InitTypeDef           Init;         /*!< DSI required parameters    */
  HAL_LockTypeDef           Lock;         /*!< DSI peripheral status      */
  __IO HAL_DSI_StateTypeDef State;        /*!< DSI communication state    */
  __IO uint32_t             ErrorCode;    /*!< DSI Error code             */
  uint32_t                  ErrorMsk;     /*!< DSI Error monitoring mask  */

#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
  void (* TearingEffectCallback)(struct __DSI_HandleTypeDef *hdsi);   /*!< DSI Tearing Effect Callback */
  void (* EndOfRefreshCallback)(struct __DSI_HandleTypeDef *hdsi);    /*!< DSI End Of Refresh Callback */
  void (* ErrorCallback)(struct __DSI_HandleTypeDef *hdsi);           /*!< DSI Error Callback          */

  void (* MspInitCallback)(struct __DSI_HandleTypeDef *hdsi);         /*!< DSI Msp Init callback       */
  void (* MspDeInitCallback)(struct __DSI_HandleTypeDef *hdsi);       /*!< DSI Msp DeInit callback     */

#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */

} DSI_HandleTypeDef;

#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL DSI Callback ID enumeration definition
  */
typedef enum
{
  HAL_DSI_MSPINIT_CB_ID            = 0x00U,    /*!< DSI MspInit callback ID        */
  HAL_DSI_MSPDEINIT_CB_ID          = 0x01U,    /*!< DSI MspDeInit callback ID      */

  HAL_DSI_TEARING_EFFECT_CB_ID     = 0x02U,    /*!< DSI Tearing Effect Callback ID */
  HAL_DSI_ENDOF_REFRESH_CB_ID      = 0x03U,    /*!< DSI End Of Refresh Callback ID */
  HAL_DSI_ERROR_CB_ID              = 0x04U     /*!< DSI Error Callback ID          */

} HAL_DSI_CallbackIDTypeDef;

/**
  * @brief  HAL DSI Callback pointer definition
  */
typedef  void (*pDSI_CallbackTypeDef)(DSI_HandleTypeDef *hdsi);  /*!< pointer to an DSI callback function */

#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DSI_Exported_Constants DSI Exported Constants
  * @{
  */
/** @defgroup DSI_DCS_Command DSI DCS Command
  * @{
  */
#define DSI_ENTER_IDLE_MODE       0x39U
#define DSI_ENTER_INVERT_MODE     0x21U
#define DSI_ENTER_NORMAL_MODE     0x13U
#define DSI_ENTER_PARTIAL_MODE    0x12U
#define DSI_ENTER_SLEEP_MODE      0x10U
#define DSI_EXIT_IDLE_MODE        0x38U
#define DSI_EXIT_INVERT_MODE      0x20U
#define DSI_EXIT_SLEEP_MODE       0x11U
#define DSI_GET_3D_CONTROL        0x3FU
#define DSI_GET_ADDRESS_MODE      0x0BU
#define DSI_GET_BLUE_CHANNEL      0x08U
#define DSI_GET_DIAGNOSTIC_RESULT 0x0FU
#define DSI_GET_DISPLAY_MODE      0x0DU
#define DSI_GET_GREEN_CHANNEL     0x07U
#define DSI_GET_PIXEL_FORMAT      0x0CU
#define DSI_GET_POWER_MODE        0x0AU
#define DSI_GET_RED_CHANNEL       0x06U
#define DSI_GET_SCANLINE          0x45U
#define DSI_GET_SIGNAL_MODE       0x0EU
#define DSI_NOP                   0x00U
#define DSI_READ_DDB_CONTINUE     0xA8U
#define DSI_READ_DDB_START        0xA1U
#define DSI_READ_MEMORY_CONTINUE  0x3EU
#define DSI_READ_MEMORY_START     0x2EU
#define DSI_SET_3D_CONTROL        0x3DU
#define DSI_SET_ADDRESS_MODE      0x36U
#define DSI_SET_COLUMN_ADDRESS    0x2AU
#define DSI_SET_DISPLAY_OFF       0x28U
#define DSI_SET_DISPLAY_ON        0x29U
#define DSI_SET_GAMMA_CURVE       0x26U
#define DSI_SET_PAGE_ADDRESS      0x2BU
#define DSI_SET_PARTIAL_COLUMNS   0x31U
#define DSI_SET_PARTIAL_ROWS      0x30U
#define DSI_SET_PIXEL_FORMAT      0x3AU
#define DSI_SET_SCROLL_AREA       0x33U
#define DSI_SET_SCROLL_START      0x37U
#define DSI_SET_TEAR_OFF          0x34U
#define DSI_SET_TEAR_ON           0x35U
#define DSI_SET_TEAR_SCANLINE     0x44U
#define DSI_SET_VSYNC_TIMING      0x40U
#define DSI_SOFT_RESET            0x01U
#define DSI_WRITE_LUT             0x2DU
#define DSI_WRITE_MEMORY_CONTINUE 0x3CU
#define DSI_WRITE_MEMORY_START    0x2CU
/**
  * @}
  */

/** @defgroup DSI_Video_Mode_Type DSI Video Mode Type
  * @{
  */
#define DSI_VID_MODE_NB_PULSES    0U
#define DSI_VID_MODE_NB_EVENTS    1U
#define DSI_VID_MODE_BURST        2U
/**
  * @}
  */

/** @defgroup DSI_Color_Mode DSI Color Mode
  * @{
  */
#define DSI_COLOR_MODE_FULL       0x00000000U
#define DSI_COLOR_MODE_EIGHT      DSI_WCR_COLM
/**
  * @}
  */

/** @defgroup DSI_ShutDown DSI ShutDown
  * @{
  */
#define DSI_DISPLAY_ON            0x00000000U
#define DSI_DISPLAY_OFF           DSI_WCR_SHTDN
/**
  * @}
  */

/** @defgroup DSI_LP_Command DSI LP Command
  * @{
  */
#define DSI_LP_COMMAND_DISABLE    0x00000000U
#define DSI_LP_COMMAND_ENABLE     DSI_VMCR_LPCE
/**
  * @}
  */

/** @defgroup DSI_LP_HFP DSI LP HFP
  * @{
  */
#define DSI_LP_HFP_DISABLE        0x00000000U
#define DSI_LP_HFP_ENABLE         DSI_VMCR_LPHFPE
/**
  * @}
  */

/** @defgroup DSI_LP_HBP DSI LP HBP
  * @{
  */
#define DSI_LP_HBP_DISABLE        0x00000000U
#define DSI_LP_HBP_ENABLE         DSI_VMCR_LPHBPE
/**
  * @}
  */

/** @defgroup DSI_LP_VACT DSI LP VACT
  * @{
  */
#define DSI_LP_VACT_DISABLE       0x00000000U
#define DSI_LP_VACT_ENABLE        DSI_VMCR_LPVAE
/**
  * @}
  */

/** @defgroup DSI_LP_VFP DSI LP VFP
  * @{
  */
#define DSI_LP_VFP_DISABLE       0x00000000U
#define DSI_LP_VFP_ENABLE        DSI_VMCR_LPVFPE
/**
  * @}
  */

/** @defgroup DSI_LP_VBP DSI LP VBP
  * @{
  */
#define DSI_LP_VBP_DISABLE       0x00000000U
#define DSI_LP_VBP_ENABLE        DSI_VMCR_LPVBPE
/**
  * @}
  */

/** @defgroup DSI_LP_VSYNC DSI LP VSYNC
  * @{
  */
#define DSI_LP_VSYNC_DISABLE     0x00000000U
#define DSI_LP_VSYNC_ENABLE      DSI_VMCR_LPVSAE
/**
  * @}
  */

/** @defgroup DSI_FBTA_acknowledge DSI FBTA Acknowledge
  * @{
  */
#define DSI_FBTAA_DISABLE        0x00000000U
#define DSI_FBTAA_ENABLE         DSI_VMCR_FBTAAE
/**
  * @}
  */

/** @defgroup DSI_TearingEffectSource DSI Tearing Effect Source
  * @{
  */
#define DSI_TE_DSILINK           0x00000000U
#define DSI_TE_EXTERNAL          DSI_WCFGR_TESRC
/**
  * @}
  */

/** @defgroup DSI_TearingEffectPolarity DSI Tearing Effect Polarity
  * @{
  */
#define DSI_TE_RISING_EDGE       0x00000000U
#define DSI_TE_FALLING_EDGE      DSI_WCFGR_TEPOL
/**
  * @}
  */

/** @defgroup DSI_Vsync_Polarity DSI Vsync Polarity
  * @{
  */
#define DSI_VSYNC_FALLING        0x00000000U
#define DSI_VSYNC_RISING         DSI_WCFGR_VSPOL
/**
  * @}
  */

/** @defgroup DSI_AutomaticRefresh DSI Automatic Refresh
  * @{
  */
#define DSI_AR_DISABLE           0x00000000U
#define DSI_AR_ENABLE            DSI_WCFGR_AR
/**
  * @}
  */

/** @defgroup DSI_TE_AcknowledgeRequest DSI TE Acknowledge Request
  * @{
  */
#define DSI_TE_ACKNOWLEDGE_DISABLE 0x00000000U
#define DSI_TE_ACKNOWLEDGE_ENABLE  DSI_CMCR_TEARE
/**
  * @}
  */

/** @defgroup DSI_AcknowledgeRequest DSI Acknowledge Request
  * @{
  */
#define DSI_ACKNOWLEDGE_DISABLE   0x00000000U
#define DSI_ACKNOWLEDGE_ENABLE    DSI_CMCR_ARE
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteNoP DSI LP LPGen Short Write NoP
  * @{
  */
#define DSI_LP_GSW0P_DISABLE     0x00000000U
#define DSI_LP_GSW0P_ENABLE      DSI_CMCR_GSW0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteOneP DSI LP LPGen Short Write OneP
  * @{
  */
#define DSI_LP_GSW1P_DISABLE     0x00000000U
#define DSI_LP_GSW1P_ENABLE      DSI_CMCR_GSW1TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortWriteTwoP DSI LP LPGen Short Write TwoP
  * @{
  */
#define DSI_LP_GSW2P_DISABLE     0x00000000U
#define DSI_LP_GSW2P_ENABLE      DSI_CMCR_GSW2TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadNoP DSI LP LPGen Short Read NoP
  * @{
  */
#define DSI_LP_GSR0P_DISABLE     0x00000000U
#define DSI_LP_GSR0P_ENABLE      DSI_CMCR_GSR0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadOneP DSI LP LPGen Short Read OneP
  * @{
  */
#define DSI_LP_GSR1P_DISABLE     0x00000000U
#define DSI_LP_GSR1P_ENABLE      DSI_CMCR_GSR1TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenShortReadTwoP DSI LP LPGen Short Read TwoP
  * @{
  */
#define DSI_LP_GSR2P_DISABLE     0x00000000U
#define DSI_LP_GSR2P_ENABLE      DSI_CMCR_GSR2TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPGenLongWrite DSI LP LPGen LongWrite
  * @{
  */
#define DSI_LP_GLW_DISABLE       0x00000000U
#define DSI_LP_GLW_ENABLE        DSI_CMCR_GLWTX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortWriteNoP DSI LP LPDcs Short Write NoP
  * @{
  */
#define DSI_LP_DSW0P_DISABLE     0x00000000U
#define DSI_LP_DSW0P_ENABLE      DSI_CMCR_DSW0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortWriteOneP DSI LP LPDcs Short Write OneP
  * @{
  */
#define DSI_LP_DSW1P_DISABLE     0x00000000U
#define DSI_LP_DSW1P_ENABLE      DSI_CMCR_DSW1TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsShortReadNoP DSI LP LPDcs Short Read NoP
  * @{
  */
#define DSI_LP_DSR0P_DISABLE     0x00000000U
#define DSI_LP_DSR0P_ENABLE      DSI_CMCR_DSR0TX
/**
  * @}
  */

/** @defgroup DSI_LP_LPDcsLongWrite DSI LP LPDcs Long Write
  * @{
  */
#define DSI_LP_DLW_DISABLE       0x00000000U
#define DSI_LP_DLW_ENABLE        DSI_CMCR_DLWTX
/**
  * @}
  */

/** @defgroup DSI_LP_LPMaxReadPacket DSI LP LPMax Read Packet
  * @{
  */
#define DSI_LP_MRDP_DISABLE      0x00000000U
#define DSI_LP_MRDP_ENABLE       DSI_CMCR_MRDPS
/**
  * @}
  */

/** @defgroup DSI_HS_PrespMode DSI HS Presp Mode
  * @{
  */
#define DSI_HS_PM_DISABLE        0x00000000U
#define DSI_HS_PM_ENABLE         DSI_TCCR3_PM
/**
  * @}
  */


/** @defgroup DSI_Automatic_Clk_Lane_Control DSI Automatic Clk Lane Control
  * @{
  */
#define DSI_AUTO_CLK_LANE_CTRL_DISABLE 0x00000000U
#define DSI_AUTO_CLK_LANE_CTRL_ENABLE  DSI_CLCR_ACR
/**
  * @}
  */

/** @defgroup DSI_Number_Of_Lanes DSI Number Of Lanes
  * @{
  */
#define DSI_ONE_DATA_LANE          0U
#define DSI_TWO_DATA_LANES         1U
/**
  * @}
  */

/** @defgroup DSI_FlowControl DSI Flow Control
  * @{
  */
#define DSI_FLOW_CONTROL_CRC_RX    DSI_PCR_CRCRXE
#define DSI_FLOW_CONTROL_ECC_RX    DSI_PCR_ECCRXE
#define DSI_FLOW_CONTROL_BTA       DSI_PCR_BTAE
#define DSI_FLOW_CONTROL_EOTP_RX   DSI_PCR_ETRXE
#define DSI_FLOW_CONTROL_EOTP_TX   DSI_PCR_ETTXE
#define DSI_FLOW_CONTROL_ALL       (DSI_FLOW_CONTROL_CRC_RX | DSI_FLOW_CONTROL_ECC_RX | \
                                    DSI_FLOW_CONTROL_BTA | DSI_FLOW_CONTROL_EOTP_RX | \
                                    DSI_FLOW_CONTROL_EOTP_TX)
/**
  * @}
  */

/** @defgroup DSI_Color_Coding DSI Color Coding
  * @{
  */
#define DSI_RGB565                 0x00000000U /*!< The values 0x00000001 and 0x00000002 can also be used for the RGB565 color mode configuration */
#define DSI_RGB666                 0x00000003U /*!< The value 0x00000004 can also be used for the RGB666 color mode configuration                 */
#define DSI_RGB888                 0x00000005U
/**
  * @}
  */

/** @defgroup DSI_LooselyPacked DSI Loosely Packed
  * @{
  */
#define DSI_LOOSELY_PACKED_ENABLE  DSI_LCOLCR_LPE
#define DSI_LOOSELY_PACKED_DISABLE 0x00000000U
/**
  * @}
  */

/** @defgroup DSI_HSYNC_Polarity DSI HSYNC Polarity
  * @{
  */
#define DSI_HSYNC_ACTIVE_HIGH       0x00000000U
#define DSI_HSYNC_ACTIVE_LOW        DSI_LPCR_HSP
/**
  * @}
  */

/** @defgroup DSI_VSYNC_Active_Polarity DSI VSYNC Active Polarity
  * @{
  */
#define DSI_VSYNC_ACTIVE_HIGH       0x00000000U
#define DSI_VSYNC_ACTIVE_LOW        DSI_LPCR_VSP
/**
  * @}
  */

/** @defgroup DSI_DATA_ENABLE_Polarity DSI DATA ENABLE Polarity
  * @{
  */
#define DSI_DATA_ENABLE_ACTIVE_HIGH 0x00000000U
#define DSI_DATA_ENABLE_ACTIVE_LOW  DSI_LPCR_DEP
/**
  * @}
  */

/** @defgroup DSI_PLL_IDF DSI PLL IDF
  * @{
  */
#define DSI_PLL_IN_DIV1             0x00000001U
#define DSI_PLL_IN_DIV2             0x00000002U
#define DSI_PLL_IN_DIV3             0x00000003U
#define DSI_PLL_IN_DIV4             0x00000004U
#define DSI_PLL_IN_DIV5             0x00000005U
#define DSI_PLL_IN_DIV6             0x00000006U
#define DSI_PLL_IN_DIV7             0x00000007U
/**
  * @}
  */

/** @defgroup DSI_PLL_ODF DSI PLL ODF
  * @{
  */
#define DSI_PLL_OUT_DIV1            0x00000000U
#define DSI_PLL_OUT_DIV2            0x00000001U
#define DSI_PLL_OUT_DIV4            0x00000002U
#define DSI_PLL_OUT_DIV8            0x00000003U
/**
  * @}
  */

/** @defgroup DSI_Flags DSI Flags
  * @{
  */
#define DSI_FLAG_TE                 DSI_WISR_TEIF
#define DSI_FLAG_ER                 DSI_WISR_ERIF
#define DSI_FLAG_BUSY               DSI_WISR_BUSY
#define DSI_FLAG_PLLLS              DSI_WISR_PLLLS
#define DSI_FLAG_PLLL               DSI_WISR_PLLLIF
#define DSI_FLAG_PLLU               DSI_WISR_PLLUIF
#define DSI_FLAG_RRS                DSI_WISR_RRS
#define DSI_FLAG_RR                 DSI_WISR_RRIF
/**
  * @}
  */

/** @defgroup DSI_Interrupts DSI Interrupts
  * @{
  */
#define DSI_IT_TE                   DSI_WIER_TEIE
#define DSI_IT_ER                   DSI_WIER_ERIE
#define DSI_IT_PLLL                 DSI_WIER_PLLLIE
#define DSI_IT_PLLU                 DSI_WIER_PLLUIE
#define DSI_IT_RR                   DSI_WIER_RRIE
/**
  * @}
  */

/** @defgroup DSI_SHORT_WRITE_PKT_Data_Type DSI SHORT WRITE PKT Data Type
  * @{
  */
#define DSI_DCS_SHORT_PKT_WRITE_P0  0x00000005U /*!< DCS short write, no parameters      */
#define DSI_DCS_SHORT_PKT_WRITE_P1  0x00000015U /*!< DCS short write, one parameter      */
#define DSI_GEN_SHORT_PKT_WRITE_P0  0x00000003U /*!< Generic short write, no parameters  */
#define DSI_GEN_SHORT_PKT_WRITE_P1  0x00000013U /*!< Generic short write, one parameter  */
#define DSI_GEN_SHORT_PKT_WRITE_P2  0x00000023U /*!< Generic short write, two parameters */
/**
  * @}
  */

/** @defgroup DSI_LONG_WRITE_PKT_Data_Type DSI LONG WRITE PKT Data Type
  * @{
  */
#define DSI_DCS_LONG_PKT_WRITE      0x00000039U /*!< DCS long write     */
#define DSI_GEN_LONG_PKT_WRITE      0x00000029U /*!< Generic long write */
/**
  * @}
  */

/** @defgroup DSI_SHORT_READ_PKT_Data_Type DSI SHORT READ PKT Data Type
  * @{
  */
#define DSI_DCS_SHORT_PKT_READ      0x00000006U /*!< DCS short read                     */
#define DSI_GEN_SHORT_PKT_READ_P0   0x00000004U /*!< Generic short read, no parameters  */
#define DSI_GEN_SHORT_PKT_READ_P1   0x00000014U /*!< Generic short read, one parameter  */
#define DSI_GEN_SHORT_PKT_READ_P2   0x00000024U /*!< Generic short read, two parameters */
/**
  * @}
  */

/** @defgroup DSI_Error_Data_Type DSI Error Data Type
  * @{
  */
#define HAL_DSI_ERROR_NONE              0U
#define HAL_DSI_ERROR_ACK               0x00000001U /*!< acknowledge errors          */
#define HAL_DSI_ERROR_PHY               0x00000002U /*!< PHY related errors          */
#define HAL_DSI_ERROR_TX                0x00000004U /*!< transmission error          */
#define HAL_DSI_ERROR_RX                0x00000008U /*!< reception error             */
#define HAL_DSI_ERROR_ECC               0x00000010U /*!< ECC errors                  */
#define HAL_DSI_ERROR_CRC               0x00000020U /*!< CRC error                   */
#define HAL_DSI_ERROR_PSE               0x00000040U /*!< Packet Size error           */
#define HAL_DSI_ERROR_EOT               0x00000080U /*!< End Of Transmission error   */
#define HAL_DSI_ERROR_OVF               0x00000100U /*!< FIFO overflow error         */
#define HAL_DSI_ERROR_GEN               0x00000200U /*!< Generic FIFO related errors */
#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
#define HAL_DSI_ERROR_INVALID_CALLBACK  0x00000400U /*!< DSI Invalid Callback error  */
#endif /* USE_HAL_DSI_REGISTER_CALLBACKS */
/**
  * @}
  */

/** @defgroup DSI_Lane_Group DSI Lane Group
  * @{
  */
#define DSI_CLOCK_LANE              0x00000000U
#define DSI_DATA_LANES              0x00000001U
/**
  * @}
  */

/** @defgroup DSI_Communication_Delay DSI Communication Delay
  * @{
  */
#define DSI_SLEW_RATE_HSTX          0x00000000U
#define DSI_SLEW_RATE_LPTX          0x00000001U
#define DSI_HS_DELAY                0x00000002U
/**
  * @}
  */

/** @defgroup DSI_CustomLane DSI CustomLane
  * @{
  */
#define DSI_SWAP_LANE_PINS          0x00000000U
#define DSI_INVERT_HS_SIGNAL        0x00000001U
/**
  * @}
  */

/** @defgroup DSI_Lane_Select DSI Lane Select
  * @{
  */
#define DSI_CLK_LANE                0x00000000U
#define DSI_DATA_LANE0              0x00000001U
#define DSI_DATA_LANE1              0x00000002U
/**
  * @}
  */

/** @defgroup DSI_PHY_Timing DSI PHY Timing
  * @{
  */
#define DSI_TCLK_POST               0x00000000U
#define DSI_TLPX_CLK                0x00000001U
#define DSI_THS_EXIT                0x00000002U
#define DSI_TLPX_DATA               0x00000003U
#define DSI_THS_ZERO                0x00000004U
#define DSI_THS_TRAIL               0x00000005U
#define DSI_THS_PREPARE             0x00000006U
#define DSI_TCLK_ZERO               0x00000007U
#define DSI_TCLK_PREPARE            0x00000008U
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup DSI_Exported_Macros DSI Exported Macros
  * @{
  */

/**
  * @brief Reset DSI handle state.
  * @param  __HANDLE__: DSI handle
  * @retval None
  */
#if (USE_HAL_DSI_REGISTER_CALLBACKS == 1)
#define __HAL_DSI_RESET_HANDLE_STATE(__HANDLE__) do{                                               \
                                                     (__HANDLE__)->State = HAL_DSI_STATE_RESET;    \
                                                     (__HANDLE__)->MspInitCallback = NULL;         \
                                                     (__HANDLE__)->MspDeInitCallback = NULL;       \
                                                   } while(0)
#else
#define __HAL_DSI_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DSI_STATE_RESET)
#endif /*USE_HAL_DSI_REGISTER_CALLBACKS */

/**
  * @brief  Enables the DSI host.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_ENABLE(__HANDLE__) do { \
                                          __IO uint32_t tmpreg = 0x00U; \
                                          SET_BIT((__HANDLE__)->Instance->CR, DSI_CR_EN);\
                                          /* Delay after an DSI Host enabling */ \
                                          tmpreg = READ_BIT((__HANDLE__)->Instance->CR, DSI_CR_EN);\
                                          UNUSED(tmpreg); \
                                        } while(0U)

/**
  * @brief  Disables the DSI host.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_DISABLE(__HANDLE__) do { \
                                          __IO uint32_t tmpreg = 0x00U; \
                                          CLEAR_BIT((__HANDLE__)->Instance->CR, DSI_CR_EN);\
                                          /* Delay after an DSI Host disabling */ \
                                          tmpreg = READ_BIT((__HANDLE__)->Instance->CR, DSI_CR_EN);\
                                          UNUSED(tmpreg); \
                                         } while(0U)

/**
  * @brief  Enables the DSI wrapper.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_WRAPPER_ENABLE(__HANDLE__) do { \
                                                 __IO uint32_t tmpreg = 0x00U; \
                                                 SET_BIT((__HANDLE__)->Instance->WCR, DSI_WCR_DSIEN);\
                                                 /* Delay after an DSI warpper enabling */ \
                                                 tmpreg = READ_BIT((__HANDLE__)->Instance->WCR, DSI_WCR_DSIEN);\
                                                 UNUSED(tmpreg); \
                                                } while(0U)

/**
  * @brief  Disable the DSI wrapper.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_WRAPPER_DISABLE(__HANDLE__) do { \
                                                  __IO uint32_t tmpreg = 0x00U; \
                                                  CLEAR_BIT((__HANDLE__)->Instance->WCR, DSI_WCR_DSIEN);\
                                                  /* Delay after an DSI warpper disabling*/ \
                                                  tmpreg = READ_BIT((__HANDLE__)->Instance->WCR, DSI_WCR_DSIEN);\
                                                  UNUSED(tmpreg); \
                                                 } while(0U)

/**
  * @brief  Enables the DSI PLL.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_PLL_ENABLE(__HANDLE__) do { \
                                             __IO uint32_t tmpreg = 0x00U; \
                                             SET_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_PLLEN);\
                                             /* Delay after an DSI PLL enabling */ \
                                             tmpreg = READ_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_PLLEN);\
                                             UNUSED(tmpreg); \
                                            } while(0U)

/**
  * @brief  Disables the DSI PLL.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_PLL_DISABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_PLLEN);\
                                              /* Delay after an DSI PLL disabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_PLLEN);\
                                              UNUSED(tmpreg); \
                                             } while(0U)

/**
  * @brief  Enables the DSI regulator.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_REG_ENABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              SET_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_REGEN);\
                                              /* Delay after an DSI regulator enabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_REGEN);\
                                              UNUSED(tmpreg); \
                                            } while(0U)

/**
  * @brief  Disables the DSI regulator.
  * @param  __HANDLE__  DSI handle
  * @retval None.
  */
#define __HAL_DSI_REG_DISABLE(__HANDLE__) do { \
                                              __IO uint32_t tmpreg = 0x00U; \
                                              CLEAR_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_REGEN);\
                                              /* Delay after an DSI regulator disabling */ \
                                              tmpreg = READ_BIT((__HANDLE__)->Instance->WRPCR, DSI_WRPCR_REGEN);\
                                              UNUSED(tmpreg); \
                                             } while(0U)

/**
  * @brief  Get the DSI pending flags.
  * @param  __HANDLE__  DSI handle.
  * @param  __FLAG__  Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag
  *            @arg DSI_FLAG_BUSY : Busy Flag
  *            @arg DSI_FLAG_PLLLS: PLL Lock Status
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RRS  : Regulator Ready Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag
  * @retval The state of FLAG (SET or RESET).
  */
#define __HAL_DSI_GET_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->WISR & (__FLAG__))

/**
  * @brief  Clears the DSI pending flags.
  * @param  __HANDLE__  DSI handle.
  * @param  __FLAG__  specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_FLAG_TE   : Tearing Effect Interrupt Flag
  *            @arg DSI_FLAG_ER   : End of Refresh Interrupt Flag
  *            @arg DSI_FLAG_PLLL : PLL Lock Interrupt Flag
  *            @arg DSI_FLAG_PLLU : PLL Unlock Interrupt Flag
  *            @arg DSI_FLAG_RR   : Regulator Ready Interrupt Flag
  * @retval None
  */
#define __HAL_DSI_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->Instance->WIFCR = (__FLAG__))

/**
  * @brief  Enables the specified DSI interrupts.
  * @param  __HANDLE__  DSI handle.
  * @param __INTERRUPT__  specifies the DSI interrupt sources to be enabled.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval None
  */
#define __HAL_DSI_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->WIER |= (__INTERRUPT__))

/**
  * @brief  Disables the specified DSI interrupts.
  * @param  __HANDLE__  DSI handle
  * @param __INTERRUPT__  specifies the DSI interrupt sources to be disabled.
  *          This parameter can be any combination of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval None
  */
#define __HAL_DSI_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->WIER &= ~(__INTERRUPT__))

/**
  * @brief  Checks whether the specified DSI interrupt source is enabled or not.
  * @param  __HANDLE__  DSI handle
  * @param  __INTERRUPT__  specifies the DSI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg DSI_IT_TE  : Tearing Effect Interrupt
  *            @arg DSI_IT_ER  : End of Refresh Interrupt
  *            @arg DSI_IT_PLLL: PLL Lock Interrupt
  *            @arg DSI_IT_PLLU: PLL Unlock Interrupt
  *            @arg DSI_IT_RR  : Regulator Ready Interrupt
  * @retval The state of INTERRUPT (SET or RESET).
  */
#define __HAL_DSI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->Instance->WIER & (__INTERRUPT__))



int stm32l4_dsiinitialize(void);
void stm32l4_dsiuninitialize(void);

#define DSI_MAX_RETURN_PKT_SIZE (0x00000037U) /*!< Maximum return packet configuration */


/* Private macros ------------------------------------------------------------*/
/** @defgroup DSI_Private_Macros DSI Private Macros
  * @{
  */
#define IS_DSI_PLL_NDIV(NDIV)                       ((10U <= (NDIV)) && ((NDIV) <= 125U))
#define IS_DSI_PLL_IDF(IDF)                         (((IDF) == DSI_PLL_IN_DIV1) || \
                                                     ((IDF) == DSI_PLL_IN_DIV2) || \
                                                     ((IDF) == DSI_PLL_IN_DIV3) || \
                                                     ((IDF) == DSI_PLL_IN_DIV4) || \
                                                     ((IDF) == DSI_PLL_IN_DIV5) || \
                                                     ((IDF) == DSI_PLL_IN_DIV6) || \
                                                     ((IDF) == DSI_PLL_IN_DIV7))
#define IS_DSI_PLL_ODF(ODF)                         (((ODF) == DSI_PLL_OUT_DIV1) || \
                                                     ((ODF) == DSI_PLL_OUT_DIV2) || \
                                                     ((ODF) == DSI_PLL_OUT_DIV4) || \
                                                     ((ODF) == DSI_PLL_OUT_DIV8))
#define IS_DSI_AUTO_CLKLANE_CONTROL(AutoClkLane)    (((AutoClkLane) == DSI_AUTO_CLK_LANE_CTRL_DISABLE) || ((AutoClkLane) == DSI_AUTO_CLK_LANE_CTRL_ENABLE))
#define IS_DSI_NUMBER_OF_LANES(NumberOfLanes)       (((NumberOfLanes) == DSI_ONE_DATA_LANE) || ((NumberOfLanes) == DSI_TWO_DATA_LANES))
#define IS_DSI_FLOW_CONTROL(FlowControl)            (((FlowControl) | DSI_FLOW_CONTROL_ALL) == DSI_FLOW_CONTROL_ALL)
#define IS_DSI_COLOR_CODING(ColorCoding)            ((ColorCoding) <= 5U)
#define IS_DSI_LOOSELY_PACKED(LooselyPacked)        (((LooselyPacked) == DSI_LOOSELY_PACKED_ENABLE) || ((LooselyPacked) == DSI_LOOSELY_PACKED_DISABLE))
#define IS_DSI_DE_POLARITY(DataEnable)              (((DataEnable) == DSI_DATA_ENABLE_ACTIVE_HIGH) || ((DataEnable) == DSI_DATA_ENABLE_ACTIVE_LOW))
#define IS_DSI_VSYNC_POLARITY(VSYNC)                (((VSYNC) == DSI_VSYNC_ACTIVE_HIGH) || ((VSYNC) == DSI_VSYNC_ACTIVE_LOW))
#define IS_DSI_HSYNC_POLARITY(HSYNC)                (((HSYNC) == DSI_HSYNC_ACTIVE_HIGH) || ((HSYNC) == DSI_HSYNC_ACTIVE_LOW))
#define IS_DSI_VIDEO_MODE_TYPE(VideoModeType)       (((VideoModeType) == DSI_VID_MODE_NB_PULSES) || \
                                                     ((VideoModeType) == DSI_VID_MODE_NB_EVENTS) || \
                                                     ((VideoModeType) == DSI_VID_MODE_BURST))
#define IS_DSI_COLOR_MODE(ColorMode)                (((ColorMode) == DSI_COLOR_MODE_FULL) || ((ColorMode) == DSI_COLOR_MODE_EIGHT))
#define IS_DSI_SHUT_DOWN(ShutDown)                  (((ShutDown) == DSI_DISPLAY_ON) || ((ShutDown) == DSI_DISPLAY_OFF))
#define IS_DSI_LP_COMMAND(LPCommand)                (((LPCommand) == DSI_LP_COMMAND_DISABLE) || ((LPCommand) == DSI_LP_COMMAND_ENABLE))
#define IS_DSI_LP_HFP(LPHFP)                        (((LPHFP) == DSI_LP_HFP_DISABLE) || ((LPHFP) == DSI_LP_HFP_ENABLE))
#define IS_DSI_LP_HBP(LPHBP)                        (((LPHBP) == DSI_LP_HBP_DISABLE) || ((LPHBP) == DSI_LP_HBP_ENABLE))
#define IS_DSI_LP_VACTIVE(LPVActive)                (((LPVActive) == DSI_LP_VACT_DISABLE) || ((LPVActive) == DSI_LP_VACT_ENABLE))
#define IS_DSI_LP_VFP(LPVFP)                        (((LPVFP) == DSI_LP_VFP_DISABLE) || ((LPVFP) == DSI_LP_VFP_ENABLE))
#define IS_DSI_LP_VBP(LPVBP)                        (((LPVBP) == DSI_LP_VBP_DISABLE) || ((LPVBP) == DSI_LP_VBP_ENABLE))
#define IS_DSI_LP_VSYNC(LPVSYNC)                    (((LPVSYNC) == DSI_LP_VSYNC_DISABLE) || ((LPVSYNC) == DSI_LP_VSYNC_ENABLE))
#define IS_DSI_FBTAA(FrameBTAAcknowledge)           (((FrameBTAAcknowledge) == DSI_FBTAA_DISABLE) || ((FrameBTAAcknowledge) == DSI_FBTAA_ENABLE))
#define IS_DSI_TE_SOURCE(TESource)                  (((TESource) == DSI_TE_DSILINK) || ((TESource) == DSI_TE_EXTERNAL))
#define IS_DSI_TE_POLARITY(TEPolarity)              (((TEPolarity) == DSI_TE_RISING_EDGE) || ((TEPolarity) == DSI_TE_FALLING_EDGE))
#define IS_DSI_AUTOMATIC_REFRESH(AutomaticRefresh)  (((AutomaticRefresh) == DSI_AR_DISABLE) || ((AutomaticRefresh) == DSI_AR_ENABLE))
#define IS_DSI_VS_POLARITY(VSPolarity)              (((VSPolarity) == DSI_VSYNC_FALLING) || ((VSPolarity) == DSI_VSYNC_RISING))
#define IS_DSI_TE_ACK_REQUEST(TEAcknowledgeRequest) (((TEAcknowledgeRequest) == DSI_TE_ACKNOWLEDGE_DISABLE) || ((TEAcknowledgeRequest) == DSI_TE_ACKNOWLEDGE_ENABLE))
#define IS_DSI_ACK_REQUEST(AcknowledgeRequest)      (((AcknowledgeRequest) == DSI_ACKNOWLEDGE_DISABLE) || ((AcknowledgeRequest) == DSI_ACKNOWLEDGE_ENABLE))
#define IS_DSI_LP_GSW0P(LP_GSW0P)                   (((LP_GSW0P) == DSI_LP_GSW0P_DISABLE) || ((LP_GSW0P) == DSI_LP_GSW0P_ENABLE))
#define IS_DSI_LP_GSW1P(LP_GSW1P)                   (((LP_GSW1P) == DSI_LP_GSW1P_DISABLE) || ((LP_GSW1P) == DSI_LP_GSW1P_ENABLE))
#define IS_DSI_LP_GSW2P(LP_GSW2P)                   (((LP_GSW2P) == DSI_LP_GSW2P_DISABLE) || ((LP_GSW2P) == DSI_LP_GSW2P_ENABLE))
#define IS_DSI_LP_GSR0P(LP_GSR0P)                   (((LP_GSR0P) == DSI_LP_GSR0P_DISABLE) || ((LP_GSR0P) == DSI_LP_GSR0P_ENABLE))
#define IS_DSI_LP_GSR1P(LP_GSR1P)                   (((LP_GSR1P) == DSI_LP_GSR1P_DISABLE) || ((LP_GSR1P) == DSI_LP_GSR1P_ENABLE))
#define IS_DSI_LP_GSR2P(LP_GSR2P)                   (((LP_GSR2P) == DSI_LP_GSR2P_DISABLE) || ((LP_GSR2P) == DSI_LP_GSR2P_ENABLE))
#define IS_DSI_LP_GLW(LP_GLW)                       (((LP_GLW) == DSI_LP_GLW_DISABLE) || ((LP_GLW) == DSI_LP_GLW_ENABLE))
#define IS_DSI_LP_DSW0P(LP_DSW0P)                   (((LP_DSW0P) == DSI_LP_DSW0P_DISABLE) || ((LP_DSW0P) == DSI_LP_DSW0P_ENABLE))
#define IS_DSI_LP_DSW1P(LP_DSW1P)                   (((LP_DSW1P) == DSI_LP_DSW1P_DISABLE) || ((LP_DSW1P) == DSI_LP_DSW1P_ENABLE))
#define IS_DSI_LP_DSR0P(LP_DSR0P)                   (((LP_DSR0P) == DSI_LP_DSR0P_DISABLE) || ((LP_DSR0P) == DSI_LP_DSR0P_ENABLE))
#define IS_DSI_LP_DLW(LP_DLW)                       (((LP_DLW) == DSI_LP_DLW_DISABLE) || ((LP_DLW) == DSI_LP_DLW_ENABLE))
#define IS_DSI_LP_MRDP(LP_MRDP)                     (((LP_MRDP) == DSI_LP_MRDP_DISABLE) || ((LP_MRDP) == DSI_LP_MRDP_ENABLE))
#define IS_DSI_SHORT_WRITE_PACKET_TYPE(MODE)        (((MODE) == DSI_DCS_SHORT_PKT_WRITE_P0) || \
                                                     ((MODE) == DSI_DCS_SHORT_PKT_WRITE_P1) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_WRITE_P0) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_WRITE_P1) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_WRITE_P2))
#define IS_DSI_LONG_WRITE_PACKET_TYPE(MODE)         (((MODE) == DSI_DCS_LONG_PKT_WRITE) || \
                                                     ((MODE) == DSI_GEN_LONG_PKT_WRITE))
#define IS_DSI_READ_PACKET_TYPE(MODE)               (((MODE) == DSI_DCS_SHORT_PKT_READ) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_READ_P0) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_READ_P1) || \
                                                     ((MODE) == DSI_GEN_SHORT_PKT_READ_P2))
#define IS_DSI_COMMUNICATION_DELAY(CommDelay)       (((CommDelay) == DSI_SLEW_RATE_HSTX) || ((CommDelay) == DSI_SLEW_RATE_LPTX) || ((CommDelay) == DSI_HS_DELAY))
#define IS_DSI_LANE_GROUP(Lane)                     (((Lane) == DSI_CLOCK_LANE) || ((Lane) == DSI_DATA_LANES))
#define IS_DSI_CUSTOM_LANE(CustomLane)              (((CustomLane) == DSI_SWAP_LANE_PINS) || ((CustomLane) == DSI_INVERT_HS_SIGNAL))
#define IS_DSI_LANE(Lane)                           (((Lane) == DSI_CLOCK_LANE) || ((Lane) == DSI_DATA_LANE0) || ((Lane) == DSI_DATA_LANE1))
#define IS_DSI_PHY_TIMING(Timing)                   (((Timing) == DSI_TCLK_POST   ) || \
                                                     ((Timing) == DSI_TLPX_CLK    ) || \
                                                     ((Timing) == DSI_THS_EXIT    ) || \
                                                     ((Timing) == DSI_TLPX_DATA   ) || \
                                                     ((Timing) == DSI_THS_ZERO    ) || \
                                                     ((Timing) == DSI_THS_TRAIL   ) || \
                                                     ((Timing) == DSI_THS_PREPARE ) || \
                                                     ((Timing) == DSI_TCLK_ZERO   ) || \
                                                     ((Timing) == DSI_TCLK_PREPARE))

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)25000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#endif /* OS_ARCH_ARM_SRC_STM32L4_STM32L4_DSI_H_ */
