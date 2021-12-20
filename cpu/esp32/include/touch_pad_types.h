#ifndef TOUCH_PAD_TYPES_H
#define TOUCH_PAD_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

//Some register bits of touch sensor 8 and 9 are mismatched, we need to swap the bits.
#define TOUCH_LL_BIT_SWAP(data, n, m)       (((data >> n) &  0x1)  == ((data >> m) & 0x1) ? (data) : ((data) ^ ((0x1 <<n) | (0x1 << m))))
#define TOUCH_LL_BITS_SWAP(v)               TOUCH_LL_BIT_SWAP(v, TOUCH_PAD_NUM8, TOUCH_PAD_NUM9)


#define SOC_TOUCH_PAD_MEASURE_WAIT_MAX      (0xFF)  /*!<The timer frequency is 8Mhz, the max value is 0xff */
#define SOC_TOUCH_SENSOR_NUM                (10)
#define TOUCH_PAD_BIT_MASK_ALL              ((1 << SOC_TOUCH_SENSOR_NUM) - 1)


/** Touch pad channel */
typedef enum {
    TOUCH_PAD_NUM0 = 0, /*!< Touch pad channel 0 is GPIO4(ESP32) */
    TOUCH_PAD_NUM1,     /*!< Touch pad channel 1 is GPIO0(ESP32) / GPIO1(ESP32-S2) */
    TOUCH_PAD_NUM2,     /*!< Touch pad channel 2 is GPIO2(ESP32) / GPIO2(ESP32-S2) */
    TOUCH_PAD_NUM3,     /*!< Touch pad channel 3 is GPIO15(ESP32) / GPIO3(ESP32-S2) */
    TOUCH_PAD_NUM4,     /*!< Touch pad channel 4 is GPIO13(ESP32) / GPIO4(ESP32-S2) */
    TOUCH_PAD_NUM5,     /*!< Touch pad channel 5 is GPIO12(ESP32) / GPIO5(ESP32-S2) */
    TOUCH_PAD_NUM6,     /*!< Touch pad channel 6 is GPIO14(ESP32) / GPIO6(ESP32-S2) */
    TOUCH_PAD_NUM7,     /*!< Touch pad channel 7 is GPIO27(ESP32) / GPIO7(ESP32-S2) */
    TOUCH_PAD_NUM8,     /*!< Touch pad channel 8 is GPIO33(ESP32) / GPIO8(ESP32-S2) */
    TOUCH_PAD_NUM9,     /*!< Touch pad channel 9 is GPIO32(ESP32) / GPIO9(ESP32-S2) */
    TOUCH_PAD_MAX,
} touch_pad_t;


typedef enum {
    TOUCH_TRIGGER_BELOW = 0,   /*!<Touch interrupt will happen if counter value is less than threshold.*/
    TOUCH_TRIGGER_ABOVE = 1,   /*!<Touch interrupt will happen if counter value is larger than threshold.*/
    TOUCH_TRIGGER_MAX,
} touch_trigger_mode_t;

typedef enum {
    TOUCH_TRIGGER_SOURCE_BOTH = 0,  /*!< wakeup interrupt is generated if both SET1 and SET2 are "touched"*/
    TOUCH_TRIGGER_SOURCE_SET1 = 1,  /*!< wakeup interrupt is generated if SET1 is "touched"*/
    TOUCH_TRIGGER_SOURCE_MAX,
} touch_trigger_src_t;

typedef enum {
    TOUCH_FSM_MODE_TIMER = 0,   /*!<To start touch FSM by timer */
    TOUCH_FSM_MODE_SW,          /*!<To start touch FSM by software trigger */
    TOUCH_FSM_MODE_MAX,
} touch_fsm_mode_t;


#define TOUCH_PAD_MEASURE_CYCLE_DEFAULT (0x7fff)  /*!<The timer frequency is 8Mhz, the max value is 0x7fff */
#define TOUCH_FSM_MODE_DEFAULT          (TOUCH_FSM_MODE_SW)  /*!<The touch FSM my be started by the software or timer */
#define TOUCH_TRIGGER_MODE_DEFAULT      (TOUCH_TRIGGER_BELOW)   /*!<Interrupts can be triggered if sensor value gets below or above threshold */
#define TOUCH_TRIGGER_SOURCE_DEFAULT    (TOUCH_TRIGGER_SOURCE_SET1)  /*!<The wakeup trigger source can be SET1 or both SET1 and SET2 */


/** Touch sensor high reference voltage */
typedef enum {
    TOUCH_HVOLT_KEEP = -1, /*!<Touch sensor high reference voltage, no change  */
    TOUCH_HVOLT_2V4 = 0,   /*!<Touch sensor high reference voltage, 2.4V  */
    TOUCH_HVOLT_2V5,       /*!<Touch sensor high reference voltage, 2.5V  */
    TOUCH_HVOLT_2V6,       /*!<Touch sensor high reference voltage, 2.6V  */
    TOUCH_HVOLT_2V7,       /*!<Touch sensor high reference voltage, 2.7V  */
    TOUCH_HVOLT_MAX,
} touch_high_volt_t;

/** Touch sensor low reference voltage */
typedef enum {
    TOUCH_LVOLT_KEEP = -1, /*!<Touch sensor low reference voltage, no change  */
    TOUCH_LVOLT_0V5 = 0,   /*!<Touch sensor low reference voltage, 0.5V  */
    TOUCH_LVOLT_0V6,       /*!<Touch sensor low reference voltage, 0.6V  */
    TOUCH_LVOLT_0V7,       /*!<Touch sensor low reference voltage, 0.7V  */
    TOUCH_LVOLT_0V8,       /*!<Touch sensor low reference voltage, 0.8V  */
    TOUCH_LVOLT_MAX,
} touch_low_volt_t;

/** Touch sensor high reference voltage attenuation */
typedef enum {
    TOUCH_HVOLT_ATTEN_KEEP = -1,  /*!<Touch sensor high reference voltage attenuation, no change  */
    TOUCH_HVOLT_ATTEN_1V5 = 0,    /*!<Touch sensor high reference voltage attenuation, 1.5V attenuation  */
    TOUCH_HVOLT_ATTEN_1V,         /*!<Touch sensor high reference voltage attenuation, 1.0V attenuation  */
    TOUCH_HVOLT_ATTEN_0V5,        /*!<Touch sensor high reference voltage attenuation, 0.5V attenuation  */
    TOUCH_HVOLT_ATTEN_0V,         /*!<Touch sensor high reference voltage attenuation,   0V attenuation  */
    TOUCH_HVOLT_ATTEN_MAX,
} touch_volt_atten_t;

/** Touch sensor charge/discharge speed */
typedef enum {
    TOUCH_PAD_SLOPE_0 = 0,       /*!<Touch sensor charge / discharge speed, always zero  */
    TOUCH_PAD_SLOPE_1 = 1,       /*!<Touch sensor charge / discharge speed, slowest  */
    TOUCH_PAD_SLOPE_2 = 2,       /*!<Touch sensor charge / discharge speed */
    TOUCH_PAD_SLOPE_3 = 3,       /*!<Touch sensor charge / discharge speed  */
    TOUCH_PAD_SLOPE_4 = 4,       /*!<Touch sensor charge / discharge speed  */
    TOUCH_PAD_SLOPE_5 = 5,       /*!<Touch sensor charge / discharge speed  */
    TOUCH_PAD_SLOPE_6 = 6,       /*!<Touch sensor charge / discharge speed  */
    TOUCH_PAD_SLOPE_7 = 7,       /*!<Touch sensor charge / discharge speed, fast  */
    TOUCH_PAD_SLOPE_MAX,
} touch_cnt_slope_t;

/** Touch sensor initial charge level */
typedef enum {
    TOUCH_PAD_TIE_OPT_LOW = 0,    /*!<Initial level of charging voltage, low level */
    TOUCH_PAD_TIE_OPT_HIGH = 1,   /*!<Initial level of charging voltage, high level */
    TOUCH_PAD_TIE_OPT_MAX,
} touch_tie_opt_t;


#define TOUCH_PAD_NUM0_GPIO_NUM     4
#define TOUCH_PAD_NUM1_GPIO_NUM     0
#define TOUCH_PAD_NUM2_GPIO_NUM     2
#define TOUCH_PAD_NUM3_GPIO_NUM     15
#define TOUCH_PAD_NUM4_GPIO_NUM     13
#define TOUCH_PAD_NUM5_GPIO_NUM     12
#define TOUCH_PAD_NUM6_GPIO_NUM     14
#define TOUCH_PAD_NUM7_GPIO_NUM     27
#define TOUCH_PAD_NUM8_GPIO_NUM     33
#define TOUCH_PAD_NUM9_GPIO_NUM     32




#define SOC_GPIO_PIN_COUNT          40

#define RTC_GPIO_NUMBER             18

//RTC GPIO channels
#define RTCIO_GPIO36_CHANNEL        0   //RTCIO_CHANNEL_0
#define RTCIO_CHANNEL_0_GPIO_NUM    36

#define RTCIO_GPIO37_CHANNEL        1   //RTCIO_CHANNEL_1
#define RTCIO_CHANNEL_1_GPIO_NUM    37

#define RTCIO_GPIO38_CHANNEL        2   //RTCIO_CHANNEL_2
#define RTCIO_CHANNEL_2_GPIO_NUM    38

#define RTCIO_GPIO39_CHANNEL        3   //RTCIO_CHANNEL_3
#define RTCIO_CHANNEL_3_GPIO_NUM    39

#define RTCIO_GPIO34_CHANNEL        4   //RTCIO_CHANNEL_4
#define RTCIO_CHANNEL_4_GPIO_NUM    34

#define RTCIO_GPIO35_CHANNEL        5   //RTCIO_CHANNEL_5
#define RTCIO_CHANNEL_5_GPIO_NUM    35

#define RTCIO_GPIO25_CHANNEL        6   //RTCIO_CHANNEL_6
#define RTCIO_CHANNEL_6_GPIO_NUM    25

#define RTCIO_GPIO26_CHANNEL        7   //RTCIO_CHANNEL_7
#define RTCIO_CHANNEL_7_GPIO_NUM    26

#define RTCIO_GPIO33_CHANNEL        8   //RTCIO_CHANNEL_8
#define RTCIO_CHANNEL_8_GPIO_NUM    33

#define RTCIO_GPIO32_CHANNEL        9   //RTCIO_CHANNEL_9
#define RTCIO_CHANNEL_9_GPIO_NUM    32

#define RTCIO_GPIO4_CHANNEL         10   //RTCIO_CHANNEL_10
#define RTCIO_CHANNEL_10_GPIO_NUM   4

#define RTCIO_GPIO0_CHANNEL         11   //RTCIO_CHANNEL_11
#define RTCIO_CHANNEL_11_GPIO_NUM   0

#define RTCIO_GPIO2_CHANNEL         12   //RTCIO_CHANNEL_12
#define RTCIO_CHANNEL_12_GPIO_NUM   2

#define RTCIO_GPIO15_CHANNEL        13   //RTCIO_CHANNEL_13
#define RTCIO_CHANNEL_13_GPIO_NUM   15

#define RTCIO_GPIO13_CHANNEL        14   //RTCIO_CHANNEL_14
#define RTCIO_CHANNEL_14_GPIO_NUM   13

#define RTCIO_GPIO12_CHANNEL        15   //RTCIO_CHANNEL_15
#define RTCIO_CHANNEL_15_GPIO_NUM   12

#define RTCIO_GPIO14_CHANNEL        16   //RTCIO_CHANNEL_16
#define RTCIO_CHANNEL_16_GPIO_NUM   14

#define RTCIO_GPIO27_CHANNEL        17   //RTCIO_CHANNEL_17
#define RTCIO_CHANNEL_17_GPIO_NUM   27










#define SOC_RTCIO_PIN_COUNT 18

typedef struct {
    uint32_t reg;       /*!< Register of RTC pad, or 0 if not an RTC GPIO */
    uint32_t mux;       /*!< Bit mask for selecting digital pad or RTC pad */
    uint32_t func;      /*!< Shift of pad function (FUN_SEL) field */
    uint32_t ie;        /*!< Mask of input enable */
    uint32_t pullup;    /*!< Mask of pullup enable */
    uint32_t pulldown;  /*!< Mask of pulldown enable */
    uint32_t slpsel;    /*!< If slpsel bit is set, slpie will be used as pad input enabled signal in sleep mode */
    uint32_t slpie;     /*!< Mask of input enable in sleep mode */
    uint32_t slpoe;     /*!< Mask of output enable in sleep mode */
    uint32_t hold;      /*!< Mask of hold enable */
    // uint32_t hold_force;/*!< Mask of hold_force bit for RTC IO in RTC_CNTL_HOLD_REG */
    uint32_t drv_v;     /*!< Mask of drive capability */
    uint32_t drv_s;     /*!< Offset of drive capability */
    int rtc_num;        /*!< GPIO number (corresponds to RTC pad) */
} rtc_io_desc_t;

extern const rtc_io_desc_t rtc_io_desc[SOC_RTCIO_PIN_COUNT];


#define RTCIO_LL_PIN_FUNC                   (0U)



#define SOC_TOUCH_PAD_THRESHOLD_MAX         (0)
#define TOUCH_PAD_THRESHOLD_MAX             (SOC_TOUCH_PAD_THRESHOLD_MAX)
#define TOUCH_PAD_SLOPE_DEFAULT             (TOUCH_PAD_SLOPE_7)
#define TOUCH_PAD_TIE_OPT_DEFAULT           (TOUCH_PAD_TIE_OPT_LOW)


#ifdef __cplusplus
}
#endif

#endif // TOUCH_PAD_TYPES_H