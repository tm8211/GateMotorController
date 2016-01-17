/**
 * \file
 *
 * \brief The main loop and associated routines to run the DC gate opener.
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <ctype.h>
#include <avr32/io.h>
#include <stdio.h>
#include "compiler.h"
#include "pll_init.h"
#include "config/conf_board.h"
#include "asf/common/drivers/nvm/uc3/flash_api.h"
//#include "gpio.h"
//#include "usart.h"

#define INPUT_BUFFER_LENGTH     (128)
#define OUTPUT_BUFFER_LENGTH    (128)
static char obuf[OUTPUT_BUFFER_LENGTH+10];
static int obuflen = 0;

#define NUM_ADC_CHANNELS    8
volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address

static const char encoder[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

// GPIO pin/adc-function map.
static const gpio_map_t ADC_GPIO_MAP =
{
    { AVR32_ADC_AD_0_PIN, AVR32_ADC_AD_0_FUNCTION },
    { AVR32_ADC_AD_1_PIN, AVR32_ADC_AD_1_FUNCTION },
    { AVR32_ADC_AD_2_PIN, AVR32_ADC_AD_2_FUNCTION },
    { AVR32_ADC_AD_3_PIN, AVR32_ADC_AD_3_FUNCTION },
    { AVR32_ADC_AD_4_PIN, AVR32_ADC_AD_4_FUNCTION },
    { AVR32_ADC_AD_5_PIN, AVR32_ADC_AD_5_FUNCTION },
    { AVR32_ADC_AD_6_PIN, AVR32_ADC_AD_6_FUNCTION },
    { AVR32_ADC_AD_7_PIN, AVR32_ADC_AD_7_FUNCTION }
};

// The ADC clock range is between CLK_ADC/2, if PRESCAL is 0, and
// CLK_ADC/128, if PRESCAL is set to 63 (0x3F). PRESCAL must be
// programmed in order to provide an ADC clock frequency according
// to the parameters given in the Product definition section.
#define MC_ADC_PRESCAL      0x10        // [0..0x3F] ADCClock = CLK_ADC / ( (PRESCAL+1) * 2 )
#define MC_ADC_SHTIM        0x08        // [0..0x0F] Sample & Hold Time = (SHTIM+1) / ADCClock

#define MC_ADC_CHANNEL_CH1_CURRENT      (0)
#define MC_ADC_CHANNEL_CH2_CURRENT      (1)
#define MC_ADC_CHANNEL_CHARGER_VOLTAGE  (2)
#define MC_ADC_CHANNEL_BATTERY_VOLTAGE  (3)
#define MC_ADC_CHANNEL_CH1_MOTOR_POS    (4)
#define MC_ADC_CHANNEL_CH1_MOTOR_NEG    (5)
#define MC_ADC_CHANNEL_CH2_MOTOR_POS    (6)
#define MC_ADC_CHANNEL_CH2_MOTOR_NEG    (7)

#define MC_ADC_REFERENCE_VOLTAGE        (3.300)
#define MC_ADC_MAX_READING              (0x03FF + 1)

#define MC_RESISTIVE_DIVIDER_30K1_6K19(v)           ((v) * ((30.1 / 6.19) + 1))
#define MC_RESISTIVE_DIVIDER_30K1_7K50(v)           ((v) * ((30.1 / 7.50) + 1))
#define MC_VOLTS_TO_AMPS_MULTIPLIER                 (3.444 / 2.29)      // 2.29V corresponds to 3.44A
#define MC_ADC_CHANNEL_CHARGER_VOLTAGE_THRESHOLD    (16.0)  // volts - this is the lowest to charge; nominal 18V.
#define MC_BATTERY_MINIMUM_VOLTAGE                  (11.6)  // volts - see http://www.kendrickastro.com/lvc.html

#define GET_ADC_CURRENT_CHANNEL_FOR_MOTOR(m)        (((m) == MOTOR_1) ? MC_ADC_CHANNEL_CH1_CURRENT : MC_ADC_CHANNEL_CH2_CURRENT)
#define MC_MAX_OVERCURRENT_THRESHOLD_AMPS           (6.0)               // For each motor

#define SECONDS_TO_MS(secs)     (1000*(secs))
#define MS_TO_SECONDS(ms)       (((float)(ms))/1000)

static const gpio_map_t USART_GPIO_MAP = {
	{MC_RS232_USART_RX_PIN, MC_RS232_USART_RX_FUNCTION},
	{MC_RS232_USART_TX_PIN, MC_RS232_USART_TX_FUNCTION},
	{MC_USB_USART_RX_PIN, MC_USB_USART_RX_FUNCTION},
	{MC_USB_USART_TX_PIN, MC_USB_USART_TX_FUNCTION}
};

// USART options.
#define COM_CHANNEL_RS232       0
#define COM_CHANNEL_FTDI_USB    1
#define NUM_COM_CHANNELS        2

static int default_com_channel = COM_CHANNEL_RS232; // On boot UART may be not connected yet

static const usart_options_t USART_OPTIONS_USB = {
    .baudrate     = 115200,
    .charlength   = 8,
    .paritytype   = USART_NO_PARITY,
    .stopbits     = USART_1_STOPBIT,
    .channelmode  = USART_NORMAL_CHMODE
};

static const usart_options_t USART_OPTIONS_RS232 = {
    .baudrate     = 9600,
    .charlength   = 8,
    .paritytype   = USART_NO_PARITY,
    .stopbits     = USART_1_STOPBIT,
    .channelmode  = USART_NORMAL_CHMODE
};

typedef struct uart_buffer_struct {
    volatile avr32_usart_t *usart_ptr;
    char rxbuf[INPUT_BUFFER_LENGTH];
    int rxlen;
} uart_buffer_t;

static uart_buffer_t ubuf[NUM_COM_CHANNELS];

#define MC_SW_TIMER_CHANNEL     0
/*
 * We configure it to count every 1 milliseconds.
 * We want: (1 / (fPBA / 8)) * RC = 1 ms, hence RC = (fPBA / 8) / 1000
 * to get an interrupt every 1 ms.
 */
#define MC_SW_TIMER_RC          ((CONFIG_PBACLK_FREQ_HZ / 8) / 1000)

static const tc_interrupt_t tc0_cfg = {
    .covfs      = 0,    // Counter overflow interrupt.
    .cpas       = 0,    // RA compare interrupt.
    .cpbs       = 0,    // RB compare interrupt.
    .cpcs       = 1,    // RC compare interrupt.
    .etrgs      = 0,    // External trigger interrupt.
    .ldras      = 0,    // RA load interrupt.
    .ldrbs      = 0     // RB load interrupt.
};

static const tc_waveform_opt_t tc0_waveform = {
    .channel    = MC_SW_TIMER_CHANNEL,
    .bswtrg     = TC_EVT_EFFECT_NOOP,
    .beevt      = TC_EVT_EFFECT_NOOP,
    .bcpc       = TC_EVT_EFFECT_NOOP,   // RC compare effect on TIOB
    .bcpb       = TC_EVT_EFFECT_NOOP,   // RB compare effect on TIOB
    .aswtrg     = TC_EVT_EFFECT_NOOP,   // Software trigger effect on TIOA
    .aeevt      = TC_EVT_EFFECT_NOOP,   // External event effect on TIOA
    .acpc       = TC_EVT_EFFECT_NOOP,   // RC compare effect on TIOA
    .acpa       = TC_EVT_EFFECT_NOOP,   // RA compare effect on TIOA
    .wavsel     = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
    .enetrg     = 0,                    // External event trigger enable
    .eevt       = TC_EXT_EVENT_SEL_TIOB_INPUT,  // N/A
    .eevtedg    = TC_SEL_NO_EDGE,       // N/A
    .cpcdis     = 0,                    // Counter clock disable with RC compare
    .cpcstop    = 0,                    // Counter clock stopped with RC compare
    .burst      = TC_BURST_NOT_GATED,   // Burst signal selection
    .clki       = TC_CLOCK_RISING_EDGE, // Clock invert
    .tcclks     = TC_CLOCK_SOURCE_TC3   // Clock selection
};
static volatile uint64_t timer_ticks;

static volatile uint16_t adc_readings[NUM_ADC_CHANNELS];
static volatile float adc_readings_volts[NUM_ADC_CHANNELS];

#define LED_BUTTON_DEBOUNCE_COUNTER     1000
static const int pin_led_power_state    = PIN_LED7;
static const int pin_led_error          = PIN_LED6;
static const int pin_led_gate_state     = PIN_LED5;
static const int pin_led_timer_state    = PIN_LED4;

static uint8_t led_button_state = 1;
static uint16_t led_button_debounce_counter = 0;

#define LED_IDLE_BLINKING_PERIOD_MS             512
#define LED_IDLE_ON_BATTERY_BLINKING_PERIOD_MS  (3 * LED_IDLE_BLINKING_PERIOD_MS)
#define LED_ACTIVE_BLINKING_PERIOD_MS           128
#define LED_ERROR_BLINKING_TIME_MS              SECONDS_TO_MS(3)

static uint16_t power_state_led_blinking_period_ms;
static uint16_t error_led_blinking_period_ms = LED_ACTIVE_BLINKING_PERIOD_MS;
static uint16_t error_condition_timer_ms = 0;

//
// Here is where purpose of input pins is assigned.
//
#define MC_INPUT_PIN_EMERGENCY_STOP     0       // Stops all movement, idles motors; gate state becomes indeterminate
#define MC_INPUT_PIN_SAFETY_BEAM        1       // Stops or reverses the gate; if open, inhibits closing.
#define MC_INPUT_PIN_LOCK_STATE         2       // Prevents activation of motors
#define MC_INPUT_PIN_PUSHBUTTON_SINGLE  3       // Opens M1 when pressed; when released, reverses M1 until it stops.
#define MC_INPUT_PIN_PUSHBUTTON_DUAL    4       // Opens M1, M2 when pressed; when released, reverses M1, M2 until it stops.
#define MC_INPUT_PIN_CYCLE_SINGLE       5       // Initiates optionally timer-driven cycle on M1
#define MC_INPUT_PIN_CYCLE_DUAL         6       // Initiates optionally timer-driven cycle on M2
#define MC_INPUT_PIN_UNUSED1            7
#define MC_INPUT_PIN_UNUSED2            8
#define MC_INPUT_PIN_UNUSED3            9
#define MC_INPUT_PIN_ZW_TRIAC           10
#define NUM_INPUT_PINS                  (11)        // INP0 .. INP9, ZW_TRIAC

#define IS_PIN_EMERGENCY_STOP()         (!input_pins[MC_INPUT_PIN_EMERGENCY_STOP].state)
#define IS_PIN_SAFETY_BEAM()            (!input_pins[MC_INPUT_PIN_SAFETY_BEAM].state)
#define IS_PIN_PUSHBUTTON_SINGLE()      (!input_pins[MC_INPUT_PIN_PUSHBUTTON_SINGLE].state)
#define IS_PIN_PUSHBUTTON_DUAL()        (!input_pins[MC_INPUT_PIN_PUSHBUTTON_DUAL].state)
#define IS_PIN_CYCLE_SINGLE()           (!input_pins[MC_INPUT_PIN_CYCLE_SINGLE].state)
#define IS_PIN_CYCLE_DUAL()             (!input_pins[MC_INPUT_PIN_CYCLE_DUAL].state)
#define IS_PIN_LOCK_STATE()             (!input_pins[MC_INPUT_PIN_LOCK_STATE].state)
#define IS_PIN_ZW_TRIAC()               ( input_pins[MC_INPUT_PIN_ZW_TRIAC].state)

#define MC_INPUT_PIN_DEBOUNCE_TIME_MS   (20)

typedef struct {
    int pin_number;
    uint8_t state;
    uint16_t debounce_counter;
} input_pin_t;
static input_pin_t input_pins[NUM_INPUT_PINS];

typedef enum {
    GATE_CLOSED = 0,
    GATE_OPENED = 1,
    GATE_INDETERMINATE = 2
} gate_state_t;

typedef enum {
    GATE_POSITION_IDLE = 0,
    GATE_POSITION_MANUAL_MOTOR_OPERATION,     // 'm' command
    GATE_POSITION_OPENING_BY_COMMAND_SINGLE,  // 'oso'
    GATE_POSITION_CLOSING_BY_COMMAND_SINGLE,  // 'osc'
    GATE_POSITION_OPENING_BY_COMMAND_DUAL,    // 'odo'
    GATE_POSITION_CLOSING_BY_COMMAND_DUAL,    // 'odc'
    GATE_POSITION_OPENING_PUSHBUTTON_SINGLE,  // Pushbutton down
    GATE_POSITION_CLOSING_PUSHBUTTON_SINGLE,  // Pushbutton up
    GATE_POSITION_OPENING_PUSHBUTTON_DUAL,    // Pushbutton down
    GATE_POSITION_CLOSING_PUSHBUTTON_DUAL,    // Pushbutton up
    GATE_POSITION_OPENING_ZWAVE_DUAL,         // Z-Wave ON
    GATE_POSITION_CLOSING_ZWAVE_DUAL,         // Z-Wave OFF
    GATE_POSITION_REVERSED_BY_ZWAVE_CANCEL,   // Z-Wave change of mind mid-close
    GATE_POSITION_STOPPED_BY_SAFETY_BEAM,
    GATE_POSITION_OPENING_BY_SAFETY_BEAM,
    GATE_POSITION_CYCLE_PIN_SINGLE,           // Single gate cycle pin event caused opening or closing
    GATE_POSITION_CYCLE_PIN_DUAL,             // Single gate cycle pin event caused opening or closing
    GATE_POSITION_CYCLE_ZWAVE_EDGE_DUAL,
    GATE_POSITION_WAITING_FOR_TIMER_CLOSE,    // Cycle timer runs, and when it expires the gate should close (dual only)
    GATE_POSITION_CLOSING_BY_TIMER            // Cycle timer ran out and the gate is closing (dual only)
} gate_position_t;
static gate_position_t gate_position = GATE_POSITION_IDLE;
static uint8_t log_gate_status_transitions = true;
static void PrintGatePosition(int channel);
static void SetGatePosition(gate_position_t gpos);

#define MC_SAFETY_BEAM_OPEN_DELAY_MS        (1500)
static uint32_t downcounter_open_on_safety_beam = 0;
static uint8_t inhibit_next_timer_close_on_safety_beam = false;

typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_FORWARD = 1,
    MOTOR_STATE_REVERSE = 2
} motor_state_t;

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1
} motor_number_t;
#define NUM_MOTORS      2

typedef struct {
    motor_number_t number;
    gate_state_t gate_state;
    gate_state_t gate_next_state;
    motor_state_t state;
    motor_state_t state_after_delay;
    uint32_t start_delay_ms;
    uint32_t start_delay_downcounter_ms;
    uint32_t max_running_time_ms;
    uint32_t running_time_downcounter_ms;
    uint8_t stop_reasons;
    uint16_t driver_diag;
    volatile float current;
    float overload_current;
} motor_t;

static motor_t motors[NUM_MOTORS];

#define MC_MEAS_CURRENT_PERIOD_MS           (32)
#define MC_OVERCURRENT_GUARD_TIME_SEC       (0.25)       // seconds
#define MC_OVERCURRENT_MIN_GUARD_TIME_SEC   (0)
#define MC_OVERCURRENT_MAX_GUARD_TIME_SEC   (3)
#define MC_SIMULATED_OVERCURRENT_AMPS       (99.00)

static uint8_t flag_battery_ok = false;                 // True if the battery voltage is above 11.6V - set by the ADC.
static uint8_t flag_print_pin_status = false;
static uint8_t current_measurement_flag = false;
static uint8_t simulate_overcurrent_after_seconds = 0;  // If not zero will fake overcurrent at that time
static uint8_t flag_pending_close_on_timer = false;     // Used to launch the closing cycle
static uint32_t downcounter_close_on_timer_ms = 0;

// Open dual cycle: motor1 runs first, motor2 later.
#define MC_MOTOR1_OPEN_DELAY_MS             0
#define MC_MOTOR2_OPEN_DELAY_MS             3000

// Close dual cycle: motor2 runs first, motor1 later.
#define MC_MOTOR1_CLOSE_DELAY_MS            3000
#define MC_MOTOR2_CLOSE_DELAY_MS            0

#define MC_MOTOR_MIN_RUNNING_TIME_SECS      (5)
#define MC_MOTOR_MAX_RUNNING_TIME_SECS      (45)

#define MC_STOP_REASON_TIMEOUT      (1 << 0)
#define MC_STOP_REASON_DRIVER_ERROR (1 << 1)
#define MC_STOP_REASON_OVERCURRENT  (1 << 2)    // This is a normal reason to stop in a system with no end contacts
#define MC_STOP_REASON_COMMAND      (1 << 3)
#define MC_STOP_REASON_INPUT_PIN    (1 << 4)
#define MC_STOP_REASON_SAFETY_BEAM  (1 << 5)
#define MC_STOP_REASON_ZWAVE_CANCEL (1 << 6)

#define IS_MOTORx_IDLE(x)           (MOTOR_STATE_IDLE == motors[x].state)
#define IS_MOTOR1_IDLE()            IS_MOTORx_IDLE(MOTOR_1)
#define IS_MOTOR2_IDLE()            IS_MOTORx_IDLE(MOTOR_2)
#define IS_ALL_MOTORS_IDLE()        (IS_MOTOR1_IDLE() && IS_MOTOR2_IDLE())

#define IS_MOTOR1_POWERED()         (!IS_MOTOR1_IDLE())
#define IS_MOTOR2_POWERED()         (!IS_MOTOR2_IDLE())
#define IS_ANY_MOTOR_POWERED()      (IS_MOTOR1_POWERED() || IS_MOTOR2_POWERED())
#define IS_ANY_MOTOR_FORWARD()      ((MOTOR_STATE_FORWARD == motors[MOTOR_1].state) || (MOTOR_STATE_FORWARD == motors[MOTOR_2].state))
#define IS_ANY_MOTOR_REVERSE()      ((MOTOR_STATE_REVERSE == motors[MOTOR_1].state) || (MOTOR_STATE_REVERSE == motors[MOTOR_2].state))

#define MC_MOTOR_MIN_CURRENT_AMPS   (0.1)
#define MC_MOTOR_MAX_CURRENT_AMPS   (6.0)

#define RESPONSE_CHAR               '='

#define CMD_CLEAR_SCREEN            ('L' & 0x1F)        // Ctrl-L
#define CMD_DUAL                    'd'
#define CMD_EEPROM                  'e'
#define CMD_MOTOR                   'm'
#define CMD_OPERATE                 'o'
#define CMD_PRINT_PIN_STATUS        'p'
#define CMD_PRINT_STATUS            's'
#define CMD_SIMULATE_OVERCURRENT    't'
#define CMD_CURRENT_MEASUREMENT     'u'
#define CMD_EMERGENCY_STOP          'x'
#define CMD_ZWAVE_CFG               'z'

#define MC_MIN_CYCLE_DELAY          (15)    // seconds
#define MC_MAX_CYCLE_DELAY          (600)   // seconds

#define CLAMP_TO_RANGE(v,mi,ma)     (((v) < (mi)) ? (mi) : (((v) > (ma)) ? (ma) : (v)) )

typedef enum {
    ZWAVE_MODE_OFF = 0,
    ZWAVE_MODE_STATIC,
    ZWAVE_MODE_EDGE_RISING,
    ZWAVE_MODE_EDGE_FALLING,
    ZWAVE_MODE_EDGE_ANY
} zwave_mode_t;

#define L9958_SPI_CHIPSELECT    (0)                 /* CS_N[0] */
#define L9958_MAX_SPI_CLOCK_HZ  (5 * 1000 * 1000)   /* 5 MHz */

// The CFG_REG register, bits reversed to match the MSB logic of the SPI hardware.
#define L9958_CFG_OL_ON         (1 << 4)    // [0] Open Load in ON state Enable
#define L9958_CFG_ISR_DIS       (1 << 5)    // [0] Current Slew Rate Control Disable
#define L9958_CFG_ISR           (1 << 6)    // [0] Current Slew Rate Control Value
#define L9958_CFG_VSR           (1 << 7)    // [0] Voltage Slew Rate Control Value
#define L9958_CFG_CL_2          (1 << 12)   // [1] Bit2 for Regulation Current Level
#define L9958_CFG_CL_1          (1 << 13)   // [0] Bit1 for Regulation Current Level
#define L9958_CFG_DR            (1 << 14)   // [0] Diagnostic Reset Bit

// The DIA_REG register, bits reversed to match the MSB logic of the SPI hardware.
#define L9958_OL_OFF_MASK       (1 << 15)
#define L9958_OL_ON_MASK        (1 << 14)
#define L9958_VS_UV_MASK        (1 << 13)
#define L9958_VDD_OV_MASK       (1 << 12)
#define L9958_ILIM_MASK         (1 << 11)
#define L9958_TWARN_MASK        (1 << 10)
#define L9958_TSD_MASK          (1 << 9)
#define L9958_ACT_MASK          (1 << 8)
#define L9958_OC_LS1_MASK       (1 << 7)
#define L9958_OC_LS2_MASK       (1 << 6)
#define L9958_OC_HS1_MASK       (1 << 5)
#define L9958_OC_HS2_MASK       (1 << 4)
#define L9958_SGND_OFF_MASK     (1 << 1)
#define L9958_SBAT_OFF_MASK     (1 << 0)

#define EEPROM_VERSION_CODE     (0x0100)
#define EEPROM_DEFAULT_CURRENT  (2.0)       // In amps; probably insufficient for an actual motor

// In this structure one has to pay attention to alignment of fields.
// The compiler will add padding, which causes headache when matching fields.
typedef struct {
    uint32_t version_code;
    float cur_m1_open_max;
    float cur_m1_close_max;
    float cur_m2_open_max;
    float cur_m2_close_max;
    float runtime_max_seconds;
    uint32_t overcurrent_guard_time_ms;
    uint32_t cycle_period_seconds;
    uint32_t zwave_mode;
    uint32_t checksum;
} eeprom_shadow_t;

static eeprom_shadow_t eeprom_shadow;

static int is_eol(char c)
{
    return (c == '\n') || (c == '\r');
}

static void Obuf_Append(char c)
{
    if (obuflen < OUTPUT_BUFFER_LENGTH) {
        obuf[obuflen++] = c;
    }
}

static void queue_char(char c, int channel)
{
    if (is_eol(c) || (c == 0)) {
        if (obuflen > 0) {
            Obuf_Append('\r');
            Obuf_Append('\n');
            Obuf_Append(0);
            usart_write_line(ubuf[channel].usart_ptr, obuf);
            obuflen = 0;
        }
    } else {
        // Queue the character for output
        Obuf_Append(c);
    }
}

static void queue_hex_digit(uint8_t b, int channel)
{
    queue_char(encoder[0x0F & b], channel);
}

static void queue_hex_byte(uint8_t b, int channel)
{
    queue_hex_digit((b >> 4), channel);
    queue_hex_digit(b, channel);
}

static void queue_hex_short(uint16_t b, int channel)
{
    queue_hex_byte((uint8_t)(b >> 8), channel);
    queue_hex_byte((uint8_t)(b), channel);
}

/*
static void queue_hex_long(uint32_t b, int channel)
{
    queue_hex_short((uint16_t)(b >> 16), channel);
    queue_hex_short((uint16_t)(b), channel);
}

static void queue_hex_longlong(uint64_t b, int channel)
{
    queue_hex_long((uint32_t)(b >> 32), channel);
    queue_hex_long((uint32_t)(b), channel);
}
*/

static void queue_string(const char *str, int channel)
{
    while (*str) {
        queue_char(*str++, channel);
    }
}

static void queue_string_and_send(const char *str, int channel)
{
    queue_string(str, channel);
    queue_char(0, channel);
}

// CCITT 16bit algorithm (X^16 + X^12 + X^5 + 1)
static void CRC_Function_CRC16(uint16_t *crcp, uint8_t data)
{
    unsigned short crc = *crcp;
    crc  = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= data;
    crc ^= (uint8_t)(crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0xff) << 4) << 1;
    *crcp = crc;
}

static uint16_t Calculate_CRC16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0;
    for (uint16_t k=0; k < size; k++) {
        CRC_Function_CRC16(&crc, data[k]);
    }
    return crc;
}

static uint8_t EEPROM_Save(void)
{
    return (STATUS_OK == nvm_write(INT_USERPAGE, (uint32_t)FLASH_API_USER_PAGE_ADDRESS, &eeprom_shadow, sizeof(eeprom_shadow)));
}

static uint8_t EEPROM_LoadDefaults(void)
{
    eeprom_shadow.version_code = EEPROM_VERSION_CODE;
    eeprom_shadow.cur_m1_open_max = EEPROM_DEFAULT_CURRENT;
    eeprom_shadow.cur_m1_close_max = EEPROM_DEFAULT_CURRENT;
    eeprom_shadow.cur_m2_open_max = EEPROM_DEFAULT_CURRENT;
    eeprom_shadow.cur_m2_close_max = EEPROM_DEFAULT_CURRENT;
    eeprom_shadow.runtime_max_seconds = MC_MOTOR_MAX_RUNNING_TIME_SECS;
    eeprom_shadow.overcurrent_guard_time_ms = SECONDS_TO_MS(MC_OVERCURRENT_GUARD_TIME_SEC);
    eeprom_shadow.cycle_period_seconds = 0;
    eeprom_shadow.zwave_mode = ZWAVE_MODE_STATIC;
    eeprom_shadow.checksum = Calculate_CRC16((uint8_t *)&eeprom_shadow, sizeof(eeprom_shadow.checksum));
    return EEPROM_Save();
}

static void Initialize_EEPROM(void)
{
    if (STATUS_OK == nvm_init(INT_USERPAGE)) {
        if (STATUS_OK == nvm_read(INT_USERPAGE, (uint32_t)FLASH_API_USER_PAGE_ADDRESS, &eeprom_shadow, sizeof(eeprom_shadow))) {
            uint16_t csum = Calculate_CRC16((uint8_t *)&eeprom_shadow, sizeof(eeprom_shadow.checksum));
            if (csum == eeprom_shadow.checksum) {
                if (EEPROM_VERSION_CODE == EEPROM_VERSION_CODE) {
                    // Load completed.
                    return;
                }
            }
            if (EEPROM_LoadDefaults()) {
                queue_string_and_send("=e: NVM INIT", default_com_channel);
                return;
            }
        }
    }
    queue_string_and_send("=e: NVM ERR", default_com_channel);
}

static int get_line(int channel)
{
    if (channel < NUM_COM_CHANNELS) {
        uart_buffer_t *p = ubuf + channel;
        while (true) {
            int c;
            int rv = usart_read_char(ubuf[channel].usart_ptr, &c); 
            if (rv == USART_RX_ERROR) {
                return -1;
            }
            if (rv == USART_RX_EMPTY) {
                return 0;
            }
            // We have some character here
            if (is_eol(c)) {
                if (p->rxlen > 0) {
                    // Terminate the string so that we can run sscanf on it
                    if (p->rxlen < INPUT_BUFFER_LENGTH) {
                        p->rxbuf[p->rxlen] = 0;
                    }
                    return p->rxlen;
                } else {
                    // Ignore multiple consequent EOL characters
                }
            } if (CMD_CLEAR_SCREEN == c) {
                // We want it to execute immediately on empty buffer, or ignored
                // if the buffer already contains something.
                if (0 == p->rxlen) {
                    p->rxbuf[p->rxlen++] = c;
                    return p->rxlen;
                }
            } else {
                // Not an EOL character.
                if (p->rxlen < INPUT_BUFFER_LENGTH) {
                    p->rxbuf[p->rxlen++] = c;
                }
            }
        }
    } else {
        return -1; // No such COM channel
    }
}

static void SetPin(uint32_t pin, int value)
{
    if (value)
        gpio_set_gpio_pin(pin);
    else
        gpio_clr_gpio_pin(pin);
}

static uint16_t SPI_ExchangeWithMotorDriver(motor_number_t mi)
{
    volatile avr32_spi_t *spi;
    // Note that the transmitted and received data is sent/received MSB first.
    // The DR bit ensures that errors are latched until we turn the driver off.
    // The CL[2:1] bits are set to [10] which corresponds to the current limit
    // of 6.6A - which is the default value for this driver.
    static const uint16_t txsw = L9958_CFG_DR /*| L9958_CFG_CL_1 */| L9958_CFG_CL_2;
    uint16_t rxsw = 0xFFFF; // All error bits are set in case SPI fails.

    switch (mi) {
        case MOTOR_1:
            spi = &AVR32_SPI0;
            break;
        case MOTOR_2:
            spi = &AVR32_SPI1;
            break;
        default:
            return 0xFFFF;
    }
    if (SPI_OK == spi_selectChip(spi, L9958_SPI_CHIPSELECT)) {
        if (SPI_OK == spi_write(spi, txsw)) {
            if (SPI_OK == spi_read(spi, &rxsw)) {
                // Everything worked just fine.
            } else {
                queue_string_and_send("=E: spi_read", default_com_channel);
            }
        } else {
            queue_string_and_send("=E: spi_write", default_com_channel);
        }
        spi_unselectChip(spi, L9958_SPI_CHIPSELECT);
    } else {
        queue_string_and_send("=E: spi_selectChip", default_com_channel);
    }
    motors[mi].driver_diag = rxsw;
    return rxsw;
}

/**
 * The hardware is built such the current measuring channels (ADC0 and ADC1) are connected
 * directly to the ADC pins; and all other channels use resistive dividers. Most of those
 * dividers use 30.1k/7.5k, but one (the charger) uses 30.1k/6.19k because of higher
 * voltage there.
 *
 * This method stores calculated voltages of all ADC channels into the adc_readings_volts[]
 * array; it also calculates the current, in amps, and stores it into motor_t structures.
 */
static void Convert_ADC_Reading(int channel)
{
    float rv = (MC_ADC_REFERENCE_VOLTAGE * (float) adc_readings[channel]) / MC_ADC_MAX_READING;
    if (channel == MC_ADC_CHANNEL_CH1_CURRENT) {
        motors[MOTOR_1].current = rv * MC_VOLTS_TO_AMPS_MULTIPLIER;
    } else if (channel == MC_ADC_CHANNEL_CH2_CURRENT) {
        motors[MOTOR_2].current = rv * MC_VOLTS_TO_AMPS_MULTIPLIER;
    } else if (channel == MC_ADC_CHANNEL_CHARGER_VOLTAGE) {
        rv = MC_RESISTIVE_DIVIDER_30K1_6K19(rv);
    } else {
        rv = MC_RESISTIVE_DIVIDER_30K1_7K50(rv);
    }
    adc_readings_volts[channel] = rv;
}

static void check_adc(void)
{
    int n, completed=0;
    for (n=0; n < NUM_ADC_CHANNELS; n++) {
        if (adc_check_eoc(adc, n)) {
            adc_readings[n] = adc_get_value(adc, n);
            Convert_ADC_Reading(n);
            ++completed;
        }
    }
    // Initiate a new measurement
    if (completed) {
        adc_start(adc);
    }
}

static void print_adc_values(int channel)
{
    char msg[100];
    int n = sprintf(msg, "=a:");
    for (int k=0; k < NUM_ADC_CHANNELS; k++) {
        n += sprintf(msg+n, " %.2fV", adc_readings_volts[k]);
    }
    n += sprintf(msg+n, " |");
    for (motor_number_t mi=0; mi < NUM_MOTORS; mi++) {
        n += sprintf(msg+n, " %.2fA", motors[mi].current);
    }
    queue_string_and_send(msg, channel);
}

static void PrintInputPins(int channel)
{
    char msg[100];
    int n = sprintf(msg, "=p:");
    for (uint8_t k=0; k < NUM_INPUT_PINS; k++) {
        n += sprintf(msg+n, " %d", input_pins[k].state);
    }
    queue_string_and_send(msg, channel);
}

static void Print_EEPROM(int channel)
{
    char msg[80];
    sprintf(msg, "=e: V=%02X.%02X M1={o=%.2fA c=%.2fA} M2={o=%.2fA c=%.2fA}", 
        (unsigned int)(eeprom_shadow.version_code >> 8),
        (unsigned int)(eeprom_shadow.version_code & 0xFF),
        eeprom_shadow.cur_m1_open_max,
        eeprom_shadow.cur_m1_close_max,
        eeprom_shadow.cur_m2_open_max,
        eeprom_shadow.cur_m2_close_max);
    queue_string_and_send(msg, channel);
    sprintf(msg, "=e: RUN=%.3fs GUARD=%.3fs CYCLE=%ds",
        eeprom_shadow.runtime_max_seconds,
        MS_TO_SECONDS(eeprom_shadow.overcurrent_guard_time_ms),
        (unsigned int)(eeprom_shadow.cycle_period_seconds));
    queue_string_and_send(msg, channel);
}

static const char *GateStateToString(gate_state_t gs)
{
    static const char *gs_closed = "CLOSED";
    static const char *gs_opened = "OPENED";
    static const char *gs_indeterminate = "INDETERMINATE";
    switch (gs) {
        case GATE_CLOSED:
            return gs_closed;
        case GATE_OPENED:
            return gs_opened;
        case GATE_INDETERMINATE:
        default:
            return gs_indeterminate;
    }
}

static void MotorPrintStatus(motor_number_t mi, int channel)
{
    queue_char(RESPONSE_CHAR, channel);
    queue_char(CMD_MOTOR, channel);
    queue_hex_digit(mi, channel);
    queue_char(':', channel);
    if (motors[mi].stop_reasons & MC_STOP_REASON_TIMEOUT) {
        queue_string(" TIMEOUT", channel);
    }
    if (motors[mi].stop_reasons & MC_STOP_REASON_DRIVER_ERROR) {
        queue_string(" DRIVER_ERROR", channel);
    }
    if (motors[mi].stop_reasons & MC_STOP_REASON_OVERCURRENT) {
        queue_string(" OVERCURRENT", channel);
    }
    if (motors[mi].stop_reasons & MC_STOP_REASON_COMMAND) {
        queue_string(" COMMAND", channel);
    }
    if (motors[mi].stop_reasons & MC_STOP_REASON_INPUT_PIN) {
        queue_string(" INPUT_PIN", channel);
    }
    if (motors[mi].stop_reasons & MC_STOP_REASON_SAFETY_BEAM) {
        queue_string(" SAFETY_BEAM", channel);
    }
    if (motors[mi].stop_reasons & MC_STOP_REASON_ZWAVE_CANCEL) {
        queue_string(" ZWAVE_CANCEL", channel);
    }
    queue_char(' ', channel);
    switch (motors[mi].state) {
        case MOTOR_STATE_IDLE:
            queue_string("IDLE", channel);
            break;
        case MOTOR_STATE_FORWARD:
            queue_string("FORWARD", channel);
            break;
        case MOTOR_STATE_REVERSE:
            queue_string("REVERSE", channel);
            break;
    }
    queue_char(' ', channel);
    queue_string(GateStateToString(motors[mi].gate_state), channel);
    queue_char('/', channel);
    queue_string(GateStateToString(motors[mi].gate_next_state), channel);
    queue_char(0, channel);
}

static void SetMotor(motor_number_t mi, motor_state_t state, uint32_t start_delay, int channel)
{
    int dir, pwm;

    // Force idle if called while the emergency stop pin is active.
    // Such a call should not be made in the first place, this is just a sanity check.
    if (IS_PIN_EMERGENCY_STOP()) {
        state = MOTOR_STATE_IDLE;
    }

    // Force idle if called while the lock state pin is active.
    // Such a call should not be made in the first place, this is just a sanity check.
    if (IS_PIN_LOCK_STATE()) {
        state = MOTOR_STATE_IDLE;
    }

    if (!flag_battery_ok) {
        queue_string("SetMotor: Battery is too low", channel);
        state = MOTOR_STATE_IDLE;
    }

    if (start_delay > 0) {
        motors[mi].start_delay_ms = start_delay;
        motors[mi].state_after_delay = state;
        return;
    }

    dir = (state == MOTOR_STATE_FORWARD);
    pwm = ((state == MOTOR_STATE_FORWARD) || (state == MOTOR_STATE_REVERSE));
    if (mi == MOTOR_1) {
        SetPin(PIN_CH1_DIR, dir);
        SetPin(PIN_CH1_PWM, pwm);
        SetPin(PIN_CH1_ENA, pwm);
        if (dir) {
            motors[mi].overload_current = eeprom_shadow.cur_m1_open_max;
        } else {
            motors[mi].overload_current = eeprom_shadow.cur_m1_close_max;
        }
    } else if (mi == MOTOR_2) {
        SetPin(PIN_CH2_DIR, dir);
        SetPin(PIN_CH2_PWM, pwm);
        SetPin(PIN_CH2_ENA, pwm);
        if (dir) {
            motors[mi].overload_current = eeprom_shadow.cur_m2_open_max;
        } else {
            motors[mi].overload_current = eeprom_shadow.cur_m2_close_max;
        }
    } else {
        return;
    }
    motors[mi].state = state;
    motors[mi].state_after_delay = MOTOR_STATE_IDLE;
    motors[mi].max_running_time_ms = (uint32_t)(SECONDS_TO_MS(eeprom_shadow.runtime_max_seconds));
    if (pwm) {
        // We are running, so no stop reasons.
        motors[mi].stop_reasons = 0;
    }
    MotorPrintStatus(mi, channel);

    motors[mi].start_delay_ms = 0;
    motors[mi].start_delay_downcounter_ms = 0;
    motors[mi].running_time_downcounter_ms = pwm ? motors[mi].max_running_time_ms : 0;
}

static void PrintMotorControllerDiagnostic(motor_number_t mi, int channel)
{
    const uint16_t diag = motors[mi].driver_diag;
    queue_char(RESPONSE_CHAR, channel);
    queue_char('i', channel);
    queue_hex_digit(mi, channel);
    queue_char(':', channel);
    queue_hex_short(diag, channel);
    if (L9958_OL_OFF_MASK & diag)   queue_string(" OL_OFF", channel);
    if (L9958_OL_ON_MASK & diag)    queue_string(" OL_ON", channel);
    if (L9958_VS_UV_MASK & diag)    queue_string(" VS_UV", channel);
    if (L9958_VDD_OV_MASK & diag)   queue_string(" VDD_OV", channel);
    if (L9958_ILIM_MASK & diag)     queue_string(" ILIM", channel);
    if (L9958_TWARN_MASK & diag)    queue_string(" TWARN", channel);
    if (L9958_TSD_MASK & diag)      queue_string(" TSD", channel);
    if (L9958_ACT_MASK & diag)      queue_string(" ACT", channel);
    if (L9958_OC_LS1_MASK & diag)   queue_string(" OC_LS1", channel);
    if (L9958_OC_LS2_MASK & diag)   queue_string(" OC_LS2", channel);
    if (L9958_OC_HS1_MASK & diag)   queue_string(" OC_HS1", channel);
    if (L9958_OC_HS2_MASK & diag)   queue_string(" OC_HS2", channel);
    if (L9958_SGND_OFF_MASK & diag) queue_string(" SGND_OFF", channel);
    if (L9958_SBAT_OFF_MASK & diag) queue_string(" SBAT_OFF", channel);
    queue_char(0, channel);
}

static void PrintZwaveMode(int channel)
{
    queue_string("=z: Z-WAVE ", channel);
    switch (eeprom_shadow.zwave_mode) {
        case ZWAVE_MODE_OFF:
            queue_string("OFF", channel);
            break;
        case ZWAVE_MODE_STATIC:
            queue_string("STATIC", channel);
            break;
        case ZWAVE_MODE_EDGE_RISING:
            queue_string("RISING EDGE", channel);
            break;
        case ZWAVE_MODE_EDGE_FALLING:
            queue_string("FALLING EDGE", channel);
            break;
        case ZWAVE_MODE_EDGE_ANY:
            queue_string("ANY EDGE", channel);
            break;
        default:
            queue_char('?', channel);
    }
    queue_char(0, channel);
}

static void CancelPendingCloseOnTimer(void)
{
    flag_pending_close_on_timer = false;
    downcounter_close_on_timer_ms = 0;
}

static void ExecuteEmergencyStop(uint8_t stop_reason_flags, uint8_t led_error, int channel)
{
    CancelPendingCloseOnTimer();
    downcounter_open_on_safety_beam = 0;
    SetGatePosition(GATE_POSITION_IDLE);
    for (motor_number_t mi=0; mi < NUM_MOTORS; mi++) {
        SetMotor(mi, MOTOR_STATE_IDLE, 0, channel);
        motors[mi].stop_reasons = stop_reason_flags;
        motors[mi].start_delay_ms = motors[mi].start_delay_downcounter_ms = 0;
        motors[mi].gate_state = motors[mi].gate_next_state = GATE_INDETERMINATE;
        if (led_error) {
            error_condition_timer_ms = LED_ERROR_BLINKING_TIME_MS;
        }
    }
}

/**
 * This method scrolls the screen by printing 24 empty lines.
 * Works on every terminal - even on a typewriter.
 */
static void Command_ClearScreen(int channel)
{
    for (uint8_t v1=24; v1; --v1) {
        queue_char(' ', channel);
        queue_char(0, channel);
    }
}

static void PrintOvercurrentSimulationOption(uint8_t force, int channel)
{
    if (simulate_overcurrent_after_seconds) {
        char msg[80];
        sprintf(msg, "=t: Simulating overcurrent after %d seconds", simulate_overcurrent_after_seconds);
        queue_string_and_send(msg, channel);
    } else {
        if (force) {
            queue_string_and_send("=t: Not simulating overcurrent", channel);
        }
    }
}

static void PrintPinPrintoutOption(uint8_t force, int channel)
{
    if (flag_print_pin_status) {
        queue_string_and_send("=p: Will print input pin state changes", channel);
    } else {
        if (force) {
            queue_string_and_send("=p: Will not print input pin state changes", channel);
        }
    }
}

static void PrintCurrentMeasurementFlag(uint8_t force, int channel)
{
    if (current_measurement_flag) {
        queue_string_and_send("=u: Will print real-time motor currents", channel);
    } else {
        if (force) {
            queue_string_and_send("=u: Will not print motor currents", channel);
        }
    }
}

static void Command_PrintStatus(int channel)
{
    for (uint8_t v1=0; v1 < NUM_MOTORS; v1++) {
        MotorPrintStatus(v1, channel);
        PrintMotorControllerDiagnostic(v1, channel);
    }
    PrintGatePosition(channel);
    print_adc_values(channel);
    PrintInputPins(channel);
    Print_EEPROM(channel);
    PrintCurrentMeasurementFlag(false, channel);
    PrintOvercurrentSimulationOption(false, channel);
    PrintPinPrintoutOption(false, channel);
    PrintZwaveMode(channel);
}

static void Report_Invalid_Input(int channel)
{
    queue_string_and_send("=! INVALID INPUT.", channel);
}

static void Report_Not_Idle(int channel)
{
    queue_string_and_send("=! NOT IDLE", channel);
}

static void Report_Already_Opened(int channel)
{
    queue_string_and_send("=! ALREADY OPENED", channel);
}

static void Report_Already_Closed(int channel)
{
    queue_string_and_send("=! ALREADY CLOSED", channel);
}

static void Report_In_Emergency_Stop_State(int channel)
{
    queue_string_and_send("=! EMERGENCY STOP PIN ACTIVE", channel);
}

static void Report_In_Locked_State(int channel)
{
    queue_string_and_send("=! LOCK PIN ACTIVE", channel);
}

static void Report_Inhibited_By_Safety_Beam(int channel)
{
    queue_string_and_send("=! SAFETY BEAM CROSSED", channel);
}

static void Command_RunMotor(motor_number_t mi, motor_state_t new_state, uint32_t start_delay, int channel)
{
    if (IS_PIN_EMERGENCY_STOP())
        Report_In_Emergency_Stop_State(channel);
    else if (IS_PIN_LOCK_STATE())
        Report_In_Locked_State(channel);
    else
        SetMotor(mi, new_state, start_delay, channel);
}

static void Command_RunSingleGate(uint8_t to_open, uint8_t ignore_idle, int channel)
{
    if (IS_PIN_EMERGENCY_STOP())
        Report_In_Emergency_Stop_State(channel);
    else if (IS_PIN_LOCK_STATE())
        Report_In_Locked_State(channel);
    else {
        if (to_open) {
            if (ignore_idle || (motors[MOTOR_1].state == MOTOR_STATE_IDLE)) {
                if (motors[MOTOR_1].gate_state != GATE_OPENED) {
                    motors[MOTOR_1].gate_next_state = GATE_OPENED;
                    Command_RunMotor(MOTOR_1, MOTOR_STATE_FORWARD, 0, channel);
                } else {
                    Report_Already_Opened(channel);
                }
            } else {
                Report_Not_Idle(channel);
            }
        } else {
            if (IS_PIN_SAFETY_BEAM())
                Report_Inhibited_By_Safety_Beam(channel);
            else {
                if (ignore_idle || (motors[MOTOR_1].state == MOTOR_STATE_IDLE)) {
                    if (motors[MOTOR_1].gate_state != GATE_CLOSED) {
                        motors[MOTOR_1].gate_next_state = GATE_CLOSED;
                        Command_RunMotor(MOTOR_1, MOTOR_STATE_REVERSE, 0, channel);
                    } else {
                        Report_Already_Closed(channel);
                    }
                } else {
                    Report_Not_Idle(channel);
                }
            }
        }
    }
}

static void Command_RunDualGate(uint8_t to_open, uint8_t ignore_idle, int channel)
{
    if (IS_PIN_EMERGENCY_STOP())
        Report_In_Emergency_Stop_State(channel);
    else if (IS_PIN_LOCK_STATE())
        Report_In_Locked_State(channel);
    else {
        if (to_open) {
            if (ignore_idle || ((motors[MOTOR_1].state == MOTOR_STATE_IDLE) && (motors[MOTOR_2].state == MOTOR_STATE_IDLE))) {
                if (motors[MOTOR_1].gate_state != GATE_OPENED) {
                    motors[MOTOR_1].gate_next_state = GATE_OPENED;
                    Command_RunMotor(MOTOR_1, MOTOR_STATE_FORWARD, MC_MOTOR1_OPEN_DELAY_MS, channel);
                    motors[MOTOR_2].gate_next_state = GATE_OPENED;
                    Command_RunMotor(MOTOR_2, MOTOR_STATE_FORWARD, MC_MOTOR2_OPEN_DELAY_MS, channel);
                } else {
                    Report_Already_Opened(channel);
                }
            } else {
                Report_Not_Idle(channel);
            }
        } else {
            if (IS_PIN_SAFETY_BEAM())
                Report_Inhibited_By_Safety_Beam(channel);
            else {
                if (ignore_idle || ((motors[MOTOR_1].state == MOTOR_STATE_IDLE) && (motors[MOTOR_2].state == MOTOR_STATE_IDLE))) {
                    if (motors[MOTOR_1].gate_state != GATE_CLOSED) {
                        motors[MOTOR_1].gate_next_state = GATE_CLOSED;
                        Command_RunMotor(MOTOR_1, MOTOR_STATE_REVERSE, MC_MOTOR1_CLOSE_DELAY_MS, channel);
                        motors[MOTOR_2].gate_next_state = GATE_CLOSED;
                        Command_RunMotor(MOTOR_2, MOTOR_STATE_REVERSE, MC_MOTOR2_CLOSE_DELAY_MS, channel);
                    } else {
                        Report_Already_Closed(channel);
                    }
                } else {
                    Report_Not_Idle(channel);
                }
            }
        }
    }
}

static void parse_input(int channel)
{
    int srclen, offset=0;
    uint8_t cmd;
    const char *src;

    src = ubuf[channel].rxbuf;
    srclen = ubuf[channel].rxlen;

    // The command string consists of a command and an optional list of parameters.
    // A command is a single byte. The parameters are command-dependent.
    if (offset >= srclen) {
        goto have_parse_error;
    }        
    cmd = src[offset++];

    switch (cmd) {

        case '?':
            queue_string_and_send("Commands: ^L,e,m,o,p,s,t,u,x", channel);
            queue_string_and_send("Use <command>? for more help", channel);
            return;

        case CMD_CLEAR_SCREEN:
            Command_ClearScreen(channel);
            break;

        case CMD_PRINT_STATUS:
            if ((offset < srclen) && ('?' == src[offset])) {
                queue_string_and_send("s : prints status", channel);
                return;
            }
            Command_PrintStatus(channel);
            break;

        case CMD_CURRENT_MEASUREMENT:
            if (offset < srclen) {
                switch (src[offset]) {
                    case '?':
                        queue_string_and_send("u{0,1} : disable (0) or enable (1) current printout for the installer.", channel);
                        return;
                    case '0':
                        current_measurement_flag = false;
                        break;
                    case '1':
                        current_measurement_flag = true;
                        break;
                    default:
                        goto have_parse_error;
                }
                PrintCurrentMeasurementFlag(true, channel);
            } else {
                goto have_parse_error;
            }
            break;

        case CMD_PRINT_PIN_STATUS:
            if (offset < srclen) {
                switch (src[offset]) {
                    case '?':
                        queue_string_and_send("p{0,1} : disable (0) or enable (1) input printout for the installer.", channel);
                        return;
                    case '0':
                        flag_print_pin_status = false;
                        break;
                    case '1':
                        flag_print_pin_status = true;
                        break;
                    default:
                        goto have_parse_error;
                }
                PrintPinPrintoutOption(true, channel);
            } else {
                goto have_parse_error;
            }
            break;

        case CMD_SIMULATE_OVERCURRENT:
            if (offset < srclen) {
                if ('?' == src[offset]) {
                    queue_string_and_send("t<seconds> : simulate overcurrent after delay (0=off)", channel);
                } else {
                    int delay;
                    if (1 == sscanf(src+offset, "%d", &delay)) {
                        simulate_overcurrent_after_seconds = delay; // 0 = off
                        PrintOvercurrentSimulationOption(true, channel);
                    } else {
                        goto have_parse_error;
                    }
                }
            } else {
                goto have_parse_error;
            }
            break;

        case CMD_EEPROM:
            if (offset < srclen) {
                static const char *flt_format = "%f";
                float fvar;
                switch (src[offset++]) {

                    case '?':
                        queue_string_and_send("em{1,2}{o,c}<amps> : set overload current (FP) for motor, direction", channel);
                        queue_string_and_send("ea<current>        : set overload current (FP) for all motors, directions", channel);
                        queue_string_and_send("ec<delay_seconds>  : set cycle delay (gate open time) in seconds (FP)", channel);
                        queue_string_and_send("eg<delay_seconds>  : set guard time in seconds (FP)", channel);
                        queue_string_and_send("er<run_seconds>    : set maximum motor running time, in seconds (FP)", channel);
                        queue_string_and_send("eW                 : write the current settings into EEPROM", channel);
                        queue_string_and_send("ez{o,r,f,a,s}      : Z-Wave settings (ez? for help)", channel);
                        return;

                    case 'c':
                        do {
                            if (1 == sscanf(src+offset, "%f", &fvar)) {
                                if (0 != fvar) {
                                    fvar = CLAMP_TO_RANGE(fvar, MC_MIN_CYCLE_DELAY, MC_MAX_CYCLE_DELAY);
                                }
                                eeprom_shadow.cycle_period_seconds = fvar;
                                Print_EEPROM(channel);
                            } else {
                                goto have_parse_error;
                            }
                        } while (false);
                        break;

                    case 'g':
                        do {
                            if (1 == sscanf(src+offset, "%f", &fvar)) {
                                if (0 != fvar) {
                                    fvar = CLAMP_TO_RANGE(fvar, MC_OVERCURRENT_MIN_GUARD_TIME_SEC, MC_OVERCURRENT_MAX_GUARD_TIME_SEC);
                                }
                                eeprom_shadow.overcurrent_guard_time_ms = (uint32_t)SECONDS_TO_MS(fvar);
                                Print_EEPROM(channel);
                            } else {
                                goto have_parse_error;
                            }
                        } while (false);
                        break;

                    case 'a':
                        if (offset < srclen) {
                            float fvar;
                            if (1 == sscanf(src+offset, flt_format, &fvar)) {
                                fvar = CLAMP_TO_RANGE(fvar, MC_MOTOR_MIN_CURRENT_AMPS, MC_MOTOR_MAX_CURRENT_AMPS);
                                eeprom_shadow.cur_m1_open_max = eeprom_shadow.cur_m1_close_max =
                                    eeprom_shadow.cur_m2_open_max = eeprom_shadow.cur_m2_close_max = fvar;
                                Print_EEPROM(channel);
                            } else {
                                goto have_parse_error;
                            }
                        } else {
                            goto have_parse_error;
                        }
                        break;

                    case 'm':
                        if (offset < srclen) {
                            motor_number_t mi;
                            switch (src[offset++]) {
                                case '1':
                                    mi = MOTOR_1;
                                    break;
                                case '2':
                                    mi = MOTOR_2;
                                    break;
                                default:
                                    goto have_parse_error;
                            }
                            // Motor number had been retrieved. Now we need the open/close direction.
                            if (offset < srclen) {
                                motor_state_t dir;
                                switch (src[offset++]) {
                                    case 'o':
                                        dir = MOTOR_STATE_FORWARD;
                                        break;
                                    case 'c':
                                        dir = MOTOR_STATE_REVERSE;
                                        break;
                                    default:
                                        goto have_parse_error;
                                }
                                // Now retrieve the current
                                if (1 == sscanf(src+offset, flt_format, &fvar)) {
                                    fvar = CLAMP_TO_RANGE(fvar, MC_MOTOR_MIN_CURRENT_AMPS, MC_MOTOR_MAX_CURRENT_AMPS);
                                    if (mi == MOTOR_1) {
                                        if (dir == MOTOR_STATE_FORWARD)
                                            eeprom_shadow.cur_m1_open_max = fvar;
                                        else
                                            eeprom_shadow.cur_m1_close_max = fvar;
                                    } else {
                                        if (dir == MOTOR_STATE_FORWARD)
                                            eeprom_shadow.cur_m2_open_max = fvar;
                                        else
                                            eeprom_shadow.cur_m2_close_max = fvar;
                                    }
                                    Print_EEPROM(channel);
                                } else {
                                    goto have_parse_error;
                                }
                            } else {
                                goto have_parse_error;
                            }
                        } else {
                            goto have_parse_error;
                        }
                        break;

                    case 'r':
                        do {
                            if (1 == sscanf(src+offset, "%f", &fvar)) {
                                fvar = CLAMP_TO_RANGE(fvar, MC_MOTOR_MIN_RUNNING_TIME_SECS, MC_MOTOR_MAX_RUNNING_TIME_SECS);
                                eeprom_shadow.runtime_max_seconds = fvar;
                                Print_EEPROM(channel);
                            }
                        } while (false);
                        break;

                    case 'W':
                        if (EEPROM_Save()) {
                            queue_string_and_send("=e: Saved", channel);
                        } else {
                            queue_string_and_send("=e: ERROR writing into EEPROM", channel);
                        }
                        break;

                    case 'z':
                        if (offset < srclen) {
                            switch (src[offset]) {
                                case '?':
                                    queue_string_and_send("ezo : Z-Wave off", channel);
                                    queue_string_and_send("ezr : Cycle on Z-Wave rising edge", channel);
                                    queue_string_and_send("ezf : Cycle on Z-Wave falling edge", channel);
                                    queue_string_and_send("eza : Cycle on any Z-Wave edge", channel);
                                    queue_string_and_send("ezs : Open/Close on Z-Wave on/off", channel);
                                    return;
                                case 'o':
                                    eeprom_shadow.zwave_mode = ZWAVE_MODE_OFF;
                                    break;
                                case 'r':
                                    eeprom_shadow.zwave_mode = ZWAVE_MODE_EDGE_RISING;
                                    break;
                                case 'f':
                                    eeprom_shadow.zwave_mode = ZWAVE_MODE_EDGE_FALLING;
                                    break;
                                case 'a':
                                    eeprom_shadow.zwave_mode = ZWAVE_MODE_EDGE_ANY;
                                    break;
                                case 's':
                                    eeprom_shadow.zwave_mode = ZWAVE_MODE_STATIC;
                                    break;
                                default:
                                    goto have_parse_error;
                            }
                            PrintZwaveMode(channel);
                        }
                        break;

                    default:
                        goto have_parse_error;
                }
            } else {
                goto have_parse_error;
            }
            break;

        case CMD_MOTOR:
            if (offset < srclen) {
                motor_number_t mi = NUM_MOTORS;
                // Get the motor number
                switch (src[offset++]) {
                    case '?':
                        queue_string_and_send("m{1,2}{o,c,i}: start given motor in given direction or idle", channel);
                        queue_string_and_send("m            : stop all motors", channel);
                        return;
                    case '1':
                        mi = MOTOR_1;
                        break;
                    case '2':
                        mi = MOTOR_2;
                        break;
                    default:
                        goto have_parse_error;
                }
                // We have a good motor number. Get the direction.
                if (offset < srclen) {
                    switch (src[offset++]) {
                        case 'o':
                            Command_RunMotor(mi, MOTOR_STATE_FORWARD, 0, channel);
                            break;
                        case 'c':
                            Command_RunMotor(mi, MOTOR_STATE_REVERSE, 0, channel);
                            break;
                        case 'i':
                            Command_RunMotor(mi, MOTOR_STATE_IDLE, 0, channel);
                            break;
                        default:
                            goto have_parse_error;
                    }
                    SetGatePosition(GATE_POSITION_MANUAL_MOTOR_OPERATION);
                } else {
                    goto have_parse_error;
                }
            } else {
                // Issue the idle command to all motor controllers
                for (motor_number_t k = 0; k < NUM_MOTORS; k++) {
                    Command_RunMotor(k, MOTOR_STATE_IDLE, 0, channel);
                }
            }
            break;

        case CMD_EMERGENCY_STOP:
            if ((offset < srclen) && ('?' == src[offset])) {
                queue_string_and_send("x : emergency stop", channel);
                return;
            }
            ExecuteEmergencyStop(MC_STOP_REASON_COMMAND, false, channel);
            break;

        case CMD_OPERATE:
            if (offset < srclen) {
                uint8_t is_dual = false;
                switch (src[offset++]) {
                    case '?':
                        queue_string_and_send("o{s,d}{o,c} : operate single (s) or dual (d) gate to open (o) or close (c)", channel);
                        return;
                    case 's':
                        break;
                    case 'd':
                        is_dual = true;
                        break;
                    default:
                        goto have_parse_error;
                }
                // We have either single or dual gate operation now.
                if (offset < srclen) {
                    switch (src[offset++]) {
                        case 'c':
                            if (is_dual) {
                                SetGatePosition(GATE_POSITION_CLOSING_BY_COMMAND_DUAL);
                                Command_RunDualGate(false, true, channel);
                            } else {
                                SetGatePosition(GATE_POSITION_CLOSING_BY_COMMAND_SINGLE);
                                Command_RunSingleGate(false, true, channel);
                            }
                            break;
                        case 'o':
                            if (is_dual) {
                                SetGatePosition(GATE_POSITION_OPENING_BY_COMMAND_DUAL);
                                Command_RunDualGate(true, true, channel);
                            } else {
                                SetGatePosition(GATE_POSITION_OPENING_BY_COMMAND_SINGLE);
                                Command_RunSingleGate(true, true, channel);
                            }
                            break;
                        default:
                            goto have_parse_error;
                    }
                } else {
                    goto have_parse_error;
                }
            } else {
                goto have_parse_error;
            }
            break;

        default:
            goto have_parse_error;
    }
    return;

have_parse_error:
    Report_Invalid_Input(channel);
    return;
}

/**
 * This method checks the current state of MOTOR_1 half of the gate and returns
 * the next state that should be achieved if a cycle is called for.
 */
static gate_state_t GetGateStateForNextCycle(void)
{
    switch (motors[MOTOR_1].gate_state) {
        case GATE_CLOSED:
        case GATE_INDETERMINATE:
            return GATE_OPENED;
        case GATE_OPENED:
            return GATE_CLOSED;
        default:
            return GATE_INDETERMINATE;
    }
}

static motor_state_t GetMotorStateToAchieveGateState(gate_state_t gs)
{
    switch (gs) {
        case GATE_CLOSED:
            return MOTOR_STATE_REVERSE;
        case GATE_OPENED:
            return MOTOR_STATE_FORWARD;
        default:
            return MOTOR_STATE_IDLE;
    }
}

/**
 * This method returns true if the gate must be opened to achieve the given state.
 */
static uint8_t GetMotorDirectionToAchieveGateState(gate_state_t gs)
{
    switch (gs) {
        default:
        case GATE_CLOSED:
            return false;
        case GATE_OPENED:
            return true;
    }
}

static void ExecuteGateCycle(uint8_t is_dual)
{
    if (IS_ALL_MOTORS_IDLE()) {
        gate_state_t new_gate_state = GetGateStateForNextCycle();
        motor_state_t new_motor_state = GetMotorStateToAchieveGateState(new_gate_state);
        uint8_t new_to_open = GetMotorDirectionToAchieveGateState(new_gate_state);
        if (MOTOR_STATE_IDLE != new_motor_state) {
            if (is_dual)
                Command_RunDualGate(new_to_open, true, default_com_channel);
            else
                Command_RunSingleGate(new_to_open, true, default_com_channel);
        }
    }
}

static void on_action_button_toggle_state(uint8_t button_state)
{
    // Action occurs when the button is depressed. This corresponds to the falling edge.
    if (button_state == 0) {
#if 1
        // Run the cycle of dual gate. One action opens, another closes.
        // There is no timed action here.
        SetGatePosition(GATE_POSITION_CYCLE_PIN_DUAL);
        ExecuteGateCycle(true);
#else
        // Run both motors manually - useful for hardware testing
        switch (motors[MOTOR_1].state) {
            case MOTOR_STATE_IDLE:
                SetMotor(MOTOR_1, MOTOR_STATE_FORWARD, 0, default_com_channel);
                break;
            case MOTOR_STATE_FORWARD:
                SetMotor(MOTOR_1, MOTOR_STATE_REVERSE, 0, default_com_channel);
                break;
            case MOTOR_STATE_REVERSE:
                SetMotor(MOTOR_1, MOTOR_STATE_IDLE, 0, default_com_channel);
                break;
        }
        switch (motors[MOTOR_2].state) {
            case MOTOR_STATE_IDLE:
                SetMotor(MOTOR_2, MOTOR_STATE_FORWARD, 0, default_com_channel);
                break;
            case MOTOR_STATE_FORWARD:
                SetMotor(MOTOR_2, MOTOR_STATE_REVERSE, 0, default_com_channel);
                break;
            case MOTOR_STATE_REVERSE:
                SetMotor(MOTOR_2, MOTOR_STATE_IDLE, 0, default_com_channel);
                break;
        }
#endif
    }    
}

static void poll_action_button(void)
{
    uint8_t button_state = (0 == gpio_get_pin_value(PIN_LED_BTN)) ? 0 : 1;
    if (button_state != led_button_state) {
        if (led_button_debounce_counter > 0) {
            --led_button_debounce_counter;
            if (0 == led_button_debounce_counter) {
                led_button_state = button_state;
                on_action_button_toggle_state(led_button_state);
            }
        } else {
            // The debounce counter is 0, and the machine has settled.
        }
    } else {
        led_button_debounce_counter = LED_BUTTON_DEBOUNCE_COUNTER;
    }
}

/**
 * The timer interrupt fires every millisecond.
 */
#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined (__ICCAVR32__)
#pragma handler = EXAMPLE_TC_IRQ_GROUP, 0
__interrupt
#endif
static void TC0_InterruptHandler(void)
{
    ++timer_ticks;
    tc_read_sr(&AVR32_TC, MC_SW_TIMER_CHANNEL); // Clear the interrupt
}

// TABLE OF MOTOR CONTROL
// DIR      PWM     ACTION
// -------  ------- -------
// H        H       FORWARD
// L        L       FREEWHEELING LOW
// L        H       REVERSE
// H        L       FREEWHEELING LOW

// The outputs can be disabled (set to tri-state) by the Disable and Enable inputs DI and EN.
// Input DI has an internal pull-up. Input EN has an internal pull-down. During freewheeling
// phase, an active freewheeling on the Low-Side MOS is automatically set, switching ON the
// power transistor in parallel to the internal freewheeling diode.
//
// EN       DI      BIT ACT ACTION
// -------  ------- ------- -------
// L        L       0       TRI-STATE
// L        H       0       TRI-STATE
// H        L       1       ON-STATE
// H        H       0       TRI-STATE
//
static void ConfigurePins(void)
{
    // Inputs
    gpio_configure_pin(PIN_INP0, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP1, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP2, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP3, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP4, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP5, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP6, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP7, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP8, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_INP9, GPIO_DIR_INPUT | GPIO_PULL_UP);
    gpio_configure_pin(PIN_CH1_MISO, GPIO_DIR_INPUT);
    gpio_configure_pin(PIN_LED_BTN, GPIO_DIR_INPUT);
    gpio_configure_pin(PIN_UART0_INV_N, GPIO_DIR_INPUT);
    gpio_configure_pin(PIN_ZW_AUX_N, GPIO_DIR_INPUT);   // P1.5 on ZM3102
    gpio_configure_pin(PIN_ZW_TRIAC, GPIO_DIR_INPUT);   // P0.1 on ZM3102, used to signal on/off
    gpio_configure_pin(PIN_CH2_MISO, GPIO_DIR_INPUT);

    // Outputs defaulting to Low
    gpio_clr_gpio_pin(PIN_CH1_MOSI);
    gpio_clr_gpio_pin(PIN_CH2_MOSI);
    gpio_clr_gpio_pin(PIN_CH1_SCK);
    gpio_clr_gpio_pin(PIN_CH2_SCK);
    gpio_clr_gpio_pin(PIN_CH1_ENA);
    gpio_clr_gpio_pin(PIN_CH2_ENA);
    gpio_clr_gpio_pin(PIN_CH1_DIR);
    gpio_clr_gpio_pin(PIN_CH2_DIR);
    gpio_clr_gpio_pin(PIN_OUT0);
    gpio_clr_gpio_pin(PIN_OUT1);
    gpio_clr_gpio_pin(PIN_OUT2);
    gpio_clr_gpio_pin(PIN_OUT3);
    gpio_clr_gpio_pin(PIN_OUT4);
    gpio_clr_gpio_pin(PIN_OUT5);
    gpio_clr_gpio_pin(PIN_OUT6);
    gpio_clr_gpio_pin(PIN_OUT7);
    gpio_clr_gpio_pin(PIN_OUT8);
    gpio_clr_gpio_pin(PIN_OUT9);
    gpio_clr_gpio_pin(PIN_LED7);
    gpio_clr_gpio_pin(PIN_CH1_DIS); // We do not operate DIS pins
    gpio_clr_gpio_pin(PIN_CH2_DIS); // We do not operate DIS pins
    gpio_clr_gpio_pin(PIN_CH1_PWM); // PWM LOW = Freewheeling
    gpio_clr_gpio_pin(PIN_CH2_PWM); // PWM LOW = Freewheeling

    // Outputs defaulting to High
    gpio_set_gpio_pin(PIN_CH1_CS_N);
    gpio_set_gpio_pin(PIN_CH2_CS_N);
    gpio_set_gpio_pin(PIN_ZW_INT0);
    gpio_set_gpio_pin(PIN_LED0);
    gpio_set_gpio_pin(PIN_LED1);
    gpio_set_gpio_pin(PIN_LED2);
    gpio_set_gpio_pin(PIN_LED3);
    gpio_set_gpio_pin(PIN_LED4);
    gpio_set_gpio_pin(PIN_LED5);
    gpio_set_gpio_pin(PIN_LED6);
}

static void Configure_UARTs(void)
{
	// Assign GPIO to both USARTs.
	gpio_enable_module(USART_GPIO_MAP, sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	// USART0 is connected to RS232
	ubuf[COM_CHANNEL_RS232].usart_ptr = MC_RS232_USART;
	ubuf[COM_CHANNEL_RS232].rxlen = 0;
	// USART1 is connected to FTDI USB
	ubuf[COM_CHANNEL_FTDI_USB].usart_ptr = MC_USB_USART;
	ubuf[COM_CHANNEL_FTDI_USB].rxlen = 0;

	// Initialize USART in RS232 mode.
	usart_init_rs232(MC_RS232_USART, &USART_OPTIONS_RS232, CONFIG_PBACLK_FREQ_HZ);
	usart_init_rs232(MC_USB_USART,   &USART_OPTIONS_USB, CONFIG_PBACLK_FREQ_HZ);

	usart_write_line(MC_RS232_USART, "# Motor Controller (c) DXParts.com RS-232 interface\r\n");
	usart_write_line(MC_USB_USART, "# Motor Controller (c) DXParts.com USB interface\r\n");
}

/**
 * Poll all UARTs and process their input.
 */
static void Poll_UARTs(void)
{
    int n;
    for (n=0; n < NUM_COM_CHANNELS; n++) {
        if (get_line(n) > 0) {
#if 0
            {
                int k;
                for (k=0; k < ubuf[n].rxlen; k++) {
                    //usart_putchar(ubuf[n].usart_ptr, ubuf[n].rxbuf[k]);
                    queue_char(ubuf[n].rxbuf[k], n);
                }
                queue_char(0, n);
            }
#endif
            // Process command received on channel 'n'
            parse_input(n);
            ubuf[n].rxlen = 0;
            default_com_channel = n;
        }
    }
}

static void Configure_Timer(void)
{
    // Initialize interrupts for TC
    Disable_global_interrupt();
    INTC_init_interrupts();
    INTC_register_interrupt(&TC0_InterruptHandler, AVR32_TC_IRQ0, AVR32_INTC_INT0);
    Enable_global_interrupt();

    sysclk_enable_peripheral_clock(&AVR32_TC);
    tc_init_waveform(&AVR32_TC, &tc0_waveform);
    tc_write_rc(&AVR32_TC, MC_SW_TIMER_CHANNEL, MC_SW_TIMER_RC);
    tc_configure_interrupts(&AVR32_TC, MC_SW_TIMER_CHANNEL, &tc0_cfg);
    tc_start(&AVR32_TC, MC_SW_TIMER_CHANNEL);
}

static void Configure_ADC(void)
{
	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));

    sysclk_enable_peripheral_clock(&AVR32_ADC);

	// Lower the ADC clock to match the ADC characteristics (because we configured
	// the CPU clock to 12MHz, and the ADC clock characteristics are usually lower;
	// cf. the ADC Characteristic section in the datasheet).
	AVR32_ADC.mr |=
	    (MC_ADC_SHTIM << AVR32_ADC_MR_SHTIM_OFFSET) |
	    (MC_ADC_PRESCAL << AVR32_ADC_MR_PRESCAL_OFFSET);
	adc_configure(adc);

	// Enable the ADC channels.
	for (uint8_t n=0; n < NUM_ADC_CHANNELS; n++) {
    	adc_enable(adc, n);
	}
	adc_start(adc);
}

static void Configure_SPI(void)
{
    static const gpio_map_t SPI_GPIO_MAP =
    {
        // SPI0
        {AVR32_SPI0_NPCS_0_0_PIN , AVR32_SPI0_NPCS_0_0_FUNCTION},
        {AVR32_SPI0_SCK_0_0_PIN  , AVR32_SPI0_SCK_0_0_FUNCTION },
        {AVR32_SPI0_MISO_0_0_PIN , AVR32_SPI0_MISO_0_0_FUNCTION},
        {AVR32_SPI0_MOSI_0_0_PIN , AVR32_SPI0_MOSI_0_0_FUNCTION},
        // SPI1
        {AVR32_SPI1_NPCS_0_1_PIN , AVR32_SPI1_NPCS_0_1_FUNCTION},
        {AVR32_SPI1_SCK_0_1_PIN  , AVR32_SPI1_SCK_0_1_FUNCTION },
        {AVR32_SPI1_MISO_0_1_PIN , AVR32_SPI1_MISO_0_1_FUNCTION},
        {AVR32_SPI1_MOSI_0_1_PIN , AVR32_SPI1_MOSI_0_1_FUNCTION},

    };
    static const spi_options_t SPI_Options = {
	    .reg = L9958_SPI_CHIPSELECT,
	    .baudrate = L9958_MAX_SPI_CLOCK_HZ,
	    .bits = 16,
	    .spck_delay = 1,
	    .trans_delay = 1,
	    .stay_act = false,
	    .spi_mode = SPI_MODE_1,
	    .modfdis = false
    };
    gpio_enable_module(SPI_GPIO_MAP, sizeof(SPI_GPIO_MAP)/sizeof(SPI_GPIO_MAP[0]));

    sysclk_enable_peripheral_clock(&AVR32_SPI0);
    sysclk_enable_pba_module(SYSCLK_SPI0);
    spi_initMaster(&AVR32_SPI0, &SPI_Options);
    spi_selectionMode(&AVR32_SPI0, 0, L9958_SPI_CHIPSELECT, 0);
    spi_enable(&AVR32_SPI0);
    spi_setupChipReg(&AVR32_SPI0, &SPI_Options, CONFIG_PBACLK_FREQ_HZ);

    sysclk_enable_peripheral_clock(&AVR32_SPI1);
    sysclk_enable_pba_module(SYSCLK_SPI1);
    spi_initMaster(&AVR32_SPI1, &SPI_Options);
    spi_selectionMode(&AVR32_SPI1, 0, L9958_SPI_CHIPSELECT, 0);
    spi_enable(&AVR32_SPI1);
    spi_setupChipReg(&AVR32_SPI1, &SPI_Options, CONFIG_PBACLK_FREQ_HZ);
}

static void UpdateLedState(void)
{
    static uint8_t blink_led_state;
    static uint16_t blink_led_counter;
    static uint8_t error_led_state;
    static uint16_t error_led_counter;
    static uint8_t gate_open_led_state;
    static uint16_t gate_open_led_counter;
    static uint8_t gate_timer_led_state;
    static uint16_t gate_timer_led_counter;

    // Power state LED

    const uint8_t motors_idle = (motors[MOTOR_1].state == MOTOR_STATE_IDLE) && (motors[MOTOR_2].state == MOTOR_STATE_IDLE);
    const uint8_t charger_connected = (adc_readings_volts[MC_ADC_CHANNEL_CHARGER_VOLTAGE] >= MC_ADC_CHANNEL_CHARGER_VOLTAGE_THRESHOLD);
    if (motors_idle) {
        if (charger_connected)
        power_state_led_blinking_period_ms = LED_IDLE_BLINKING_PERIOD_MS;
        else
        power_state_led_blinking_period_ms = LED_IDLE_ON_BATTERY_BLINKING_PERIOD_MS;
        } else {
        power_state_led_blinking_period_ms = LED_ACTIVE_BLINKING_PERIOD_MS;
    }

    if (++blink_led_counter > (power_state_led_blinking_period_ms >> 1)) {
        blink_led_counter = 0;
        blink_led_state = !blink_led_state;
        SetPin(pin_led_power_state, blink_led_state);
    }

    // Error LED
    if (error_condition_timer_ms > 0) {
        --error_condition_timer_ms;
        if (error_condition_timer_ms > 0) {
            if (++error_led_counter > (error_led_blinking_period_ms >> 1)) {
                error_led_counter = 0;
                error_led_state = !error_led_state;
                SetPin(pin_led_error, error_led_state);
            }
        } else {
            SetPin(pin_led_error, true);
        }
    }

    // Gate state LED: Off=closed, On=open, Blinking = indeterminate
    // We only use MOTOR 1 for the gate state determination, as that
    // covers both cases (of opening one half of the gate, or both.)
    switch (motors[MOTOR_1].gate_state) {
        case GATE_CLOSED:
            SetPin(pin_led_gate_state, true);
            break;
        case GATE_OPENED:
            SetPin(pin_led_gate_state, false);
            break;
        case GATE_INDETERMINATE:
        default:
            do {
                if (0 == gate_open_led_counter) {
                    gate_open_led_state = !gate_open_led_state;
                    gate_open_led_counter = power_state_led_blinking_period_ms;
                } else {
                    --gate_open_led_counter;
                }
                SetPin(pin_led_gate_state, gate_open_led_state);
            } while (false);
            break;
    }

    // Gate timer LED. Indicates if a close-on-timer is pending.
    // The LED blinks at medium rate if the timer is running, and
    // blinks fast if the timer has fired but the closing hasn't yet started.
    if (flag_pending_close_on_timer || (downcounter_close_on_timer_ms > 0)) {
        if (0 == gate_timer_led_counter) {
            gate_timer_led_counter = (flag_pending_close_on_timer ? LED_ACTIVE_BLINKING_PERIOD_MS : LED_IDLE_BLINKING_PERIOD_MS) >> 1;
            gate_timer_led_state = !gate_timer_led_state;
        } else {
            --gate_timer_led_counter;
        }
    } else {
        gate_timer_led_state = true; // Off
    }
    SetPin(pin_led_timer_state, gate_timer_led_state);

    // Update motor state LEDs
    // LED 3/1: REVERSE
    // LED 2/0: FORWARD
    SetPin(PIN_LED3, !(motors[MOTOR_2].state == MOTOR_STATE_REVERSE));
    SetPin(PIN_LED2, !(motors[MOTOR_2].state == MOTOR_STATE_FORWARD));
    SetPin(PIN_LED1, !(motors[MOTOR_1].state == MOTOR_STATE_REVERSE));
    SetPin(PIN_LED0, !(motors[MOTOR_1].state == MOTOR_STATE_FORWARD));
}

static void StartAutomaticCloseTimer(void)
{
    // Default to timer disabled.
    downcounter_close_on_timer_ms = 0;
    if (inhibit_next_timer_close_on_safety_beam)
        inhibit_next_timer_close_on_safety_beam = false;
    else {
        if (!IS_PIN_EMERGENCY_STOP()) {
            if ((eeprom_shadow.cycle_period_seconds > 0)) {
                downcounter_close_on_timer_ms = (uint32_t)SECONDS_TO_MS(eeprom_shadow.cycle_period_seconds);
            }
        }
    }
}

// Check the timer for automatic close, and if necessary raise the flag that we wanted a close initiated.
static void DecrementAutomaticCloseTimer(void)
{
    if (downcounter_close_on_timer_ms > 0) {
        if (0 == --downcounter_close_on_timer_ms) {
            flag_pending_close_on_timer = true;
        }
    }
}

// Execute automatic cycle is warranted.
static void CheckAndExecuteCloseOnTimer(int channel)
{
    if (flag_pending_close_on_timer) {
        if (IS_PIN_EMERGENCY_STOP()) {
            flag_pending_close_on_timer = false;
            //queue_string_and_send("=! CLOSE ON TIMER CANCELLED BY EMERG STOP", channel);
            return;
        }
        else if (IS_ALL_MOTORS_IDLE() && (!IS_PIN_LOCK_STATE() && (!IS_PIN_SAFETY_BEAM()))) {
            flag_pending_close_on_timer = false;
            if ((motors[MOTOR_1].gate_state == GATE_OPENED) && (motors[MOTOR_2].gate_state == GATE_OPENED)) {
                SetGatePosition(GATE_POSITION_CLOSING_BY_TIMER);
                ExecuteGateCycle(true); // Dual gate only
                //queue_string_and_send("=! EXEC CLOSE ON TIMER", channel);
            } else {
                // Do not close gate that is not opened. Skip the closing cycle.
                //queue_string_and_send("=! CANCEL CLOSE ON TIMER - GATE NOT OPEN", channel);
            }
        }
    }
}

static void ProcessMotorState(motor_number_t mi, int channel)
{
    motor_t * const mp = motors + mi;
    uint32_t elapsed_time_ms;

    DecrementAutomaticCloseTimer();
    CheckAndExecuteCloseOnTimer(channel);

    // Activate a motor if it is scheduled for activation
    if (mp->start_delay_ms > 0) {
        mp->start_delay_ms--;
        if (0 == mp->start_delay_ms) {
            SetMotor(mi, mp->state_after_delay, 0, channel);
        }
    }

    // Deactivate a motor if there is a good reason to do so.
    // Here we check for errors, overcurrent, and timeout.

    if (MOTOR_STATE_IDLE != mp->state) {

        if (IS_PIN_EMERGENCY_STOP()) {
            ExecuteEmergencyStop(MC_STOP_REASON_INPUT_PIN, true, channel);
        }

        // Deactivate a motor if timeout expired. This is not a normal way to end
        // the activity; we leave the gate in indeterminate condition, so that it
        // can be operated either way in the next command, and we light up the error
        // LED blinker.
        if (mp->running_time_downcounter_ms > 0) {
            mp->running_time_downcounter_ms--;
        } else {
            ExecuteEmergencyStop(MC_STOP_REASON_TIMEOUT, true, channel);
        }

        //
        // Check error bits in the SPI diagnostic word. They should be all zero except ACT
        // and perhaps ILIM (that one pulses as it regulates the motor's average current.)
        // If that is not so, we stop BOTH motors.
        //
        if ((mp->driver_diag & ~(L9958_ACT_MASK | L9958_ILIM_MASK)) != 0) {
            ExecuteEmergencyStop(MC_STOP_REASON_DRIVER_ERROR, true, channel);
            PrintMotorControllerDiagnostic(mi, channel);
        }

        //
        // Check the current for overload. We do it on per-motor basis. The other motor, if it is
        // operating, will continue to run until it encounters its own stop condition(s).
        // We ignore guard time here, as this is a debugging/installer function.
        //
        elapsed_time_ms = mp->max_running_time_ms - mp->running_time_downcounter_ms;
        if (simulate_overcurrent_after_seconds) {
            if (elapsed_time_ms > SECONDS_TO_MS(simulate_overcurrent_after_seconds)) {
                mp->current = MC_SIMULATED_OVERCURRENT_AMPS;
            }
        }
        //
        // The guard time is the initial time, a small one, under 1s usually, after the motor is powered up.
        // We do not perform current measurement during this time because the motor that starts up may draw
        // far more current than expected. The system would cut the power to the motor. The guard time
        // allows the motor to start and begin moving the gate.
        //
        if (elapsed_time_ms >= eeprom_shadow.overcurrent_guard_time_ms) {
            if (mp->current >= mp->overload_current) {
                // Note: this is the normal way to end the cycle.
                if (current_measurement_flag) {
                    char msg[80];
                    sprintf(msg, "=m%d: OVERCURRENT=", mi);
                    queue_string(msg, channel);
                    if (mp->current != MC_SIMULATED_OVERCURRENT_AMPS) {
                        sprintf(msg, "%.2fA (ADC=%.2fV, OVC=%.2fA)",
							mp->current, adc_readings_volts[GET_ADC_CURRENT_CHANNEL_FOR_MOTOR(mi)], mp->overload_current);
                        queue_string(msg, channel);
                    } else {
                        queue_string("SIMULATED", channel);
                    }
                    queue_char(0, channel);
                }
                // Timed gate cycle works only in dual gate configuration, by design.
                // Therefore completion of opening of the MOTOR_2 is a good moment to start the timer.
                if ((mi == MOTOR_2) && (mp->state == MOTOR_STATE_FORWARD) &&
                    ((gate_position == GATE_POSITION_CYCLE_PIN_DUAL) || (gate_position == GATE_POSITION_OPENING_BY_SAFETY_BEAM)))
                {
                    StartAutomaticCloseTimer();
                }
                mp->gate_state = mp->gate_next_state;
                mp->stop_reasons = MC_STOP_REASON_OVERCURRENT;
                SetMotor(mi, MOTOR_STATE_IDLE, 0, channel);
            }
        }

        //
        // If the gate is CLOSING, the photo beam may cause reversal.
        // We allow the safety beam interruption if the gate is opening.
        //
        if (IS_PIN_SAFETY_BEAM() && (mp->state == MOTOR_STATE_REVERSE)) {
            ExecuteEmergencyStop(MC_STOP_REASON_SAFETY_BEAM, true, channel);
            SetGatePosition(GATE_POSITION_STOPPED_BY_SAFETY_BEAM);
            downcounter_open_on_safety_beam = MC_SAFETY_BEAM_OPEN_DELAY_MS;
        }

    }
}

// This method handles global, gate-wide flags.
static void ProcessGateState(void)
{
    // Sanity check to prevent closing on safety
    if (gate_position == GATE_POSITION_STOPPED_BY_SAFETY_BEAM) {
        downcounter_close_on_timer_ms = 0;
    }
    if (IS_ALL_MOTORS_IDLE()) {
        if (downcounter_close_on_timer_ms) {
            // We have timer pending, but the gate is not moving yet.
            if (gate_position != GATE_POSITION_WAITING_FOR_TIMER_CLOSE) {
                SetGatePosition(GATE_POSITION_WAITING_FOR_TIMER_CLOSE);
            }
        } else {
            // No timer expected.
            if (gate_position != GATE_POSITION_IDLE) {
                SetGatePosition(GATE_POSITION_IDLE);
            }
        }
        if (downcounter_open_on_safety_beam > 0) {
            if (0 == --downcounter_open_on_safety_beam) {
                SetGatePosition(GATE_POSITION_OPENING_BY_SAFETY_BEAM);
                Command_RunDualGate(true, true, default_com_channel);
            }
        }
    } else {
        // Some motors are active.
        //
        // Print the current curve for calibration purposes. We print both motors at the same time
        // to minimize the serial traffic.
        //
        if (current_measurement_flag) {
            if (0 == (timer_ticks % MC_MEAS_CURRENT_PERIOD_MS)) {
                char msg[40];
                sprintf(msg, "=u:%.2fV", adc_readings_volts[MC_ADC_CHANNEL_BATTERY_VOLTAGE]);
                queue_string(msg, default_com_channel);
                for (motor_number_t mi=0; mi < NUM_MOTORS; mi++) {
                    if (IS_MOTORx_IDLE(mi)) {
                        queue_string(",IDLE", default_com_channel);
                    } else {
                        const uint32_t elapsed_time_ms = motors[mi].max_running_time_ms - motors[mi].running_time_downcounter_ms;
                        const uint8_t is_within_guard_band = (elapsed_time_ms < eeprom_shadow.overcurrent_guard_time_ms);
                        queue_char(',', default_com_channel);
                        if (is_within_guard_band)
                            queue_char('[', default_com_channel);
                        if (simulate_overcurrent_after_seconds && (motors[mi].current == MC_SIMULATED_OVERCURRENT_AMPS)) {
                            queue_string("SIMOV", default_com_channel);
                        } else {
                            sprintf(msg, "%.2fA", motors[mi].current);
                            queue_string(msg, default_com_channel);
                        }
                        if (is_within_guard_band)
                            queue_char(']', default_com_channel);
                    }
                }
                queue_char(0, default_com_channel);
            }
        }
    }
}

static const char *PinNumberToString(uint8_t pin_number)
{
    switch (pin_number) {
        case MC_INPUT_PIN_EMERGENCY_STOP:       return "EMERGENCY_STOP";
        case MC_INPUT_PIN_SAFETY_BEAM:          return "SAFETY_BEAM";
        case MC_INPUT_PIN_LOCK_STATE:           return "LOCK_STATE";
        case MC_INPUT_PIN_PUSHBUTTON_SINGLE:    return "PUSHBUTTON_SINGLE";
        case MC_INPUT_PIN_PUSHBUTTON_DUAL:      return "PUSHBUTTON_DUAL";
        case MC_INPUT_PIN_CYCLE_SINGLE:         return "CYCLE_SINGLE";
        case MC_INPUT_PIN_CYCLE_DUAL:           return "CYCLE_DUAL";
        case MC_INPUT_PIN_ZW_TRIAC:             return "Z-WAVE";
        case MC_INPUT_PIN_UNUSED1:              return "UNUSED1";
        case MC_INPUT_PIN_UNUSED2:              return "UNUSED2";
        case MC_INPUT_PIN_UNUSED3:              return "UNUSED3";
        default:                                return "?";
    }
}

static void OnInputPinStateChange(uint8_t pin_number, int channel)
{
    // For convenience of debugging...
    if (flag_print_pin_status) {
        char msg[40];
        sprintf(msg, "=u: INP%d (%s) BECOMES %d", pin_number, PinNumberToString(pin_number), input_pins[pin_number].state);
        queue_string_and_send(msg, channel);
    }

    if (MC_INPUT_PIN_EMERGENCY_STOP == pin_number) {
        if (IS_PIN_EMERGENCY_STOP()) {
            ExecuteEmergencyStop(MC_STOP_REASON_INPUT_PIN, true, channel);
        }
        // If the gate is in emergency stop we do nothing further.
        return;
    }

    if (MC_INPUT_PIN_LOCK_STATE == pin_number) {
        if (IS_PIN_LOCK_STATE()) {
            // If the gate is in lock state we do nothing further.
            return;
        }
    }

    // At this point we are allowed to operate the gate.

    if (MC_INPUT_PIN_PUSHBUTTON_SINGLE == pin_number) {
        // The pushbutton of a single cycle is pressed or released.
        // Each transition may start or reverse the operation. There
        // is no provision for an emergency stop, outside of a separate
        // button (connected to INP0.)
        if (IS_ANY_MOTOR_REVERSE()) {
            ExecuteEmergencyStop(MC_STOP_REASON_SAFETY_BEAM, true, channel);
            SetGatePosition(GATE_POSITION_STOPPED_BY_SAFETY_BEAM);
            downcounter_open_on_safety_beam = MC_SAFETY_BEAM_OPEN_DELAY_MS;
        } else {
            if (IS_PIN_PUSHBUTTON_SINGLE()) {
                // Open the gate after the button got pressed.
                if (GATE_OPENED != motors[MOTOR_1].gate_state) {
                    SetGatePosition(GATE_POSITION_OPENING_PUSHBUTTON_SINGLE);
                    ExecuteGateCycle(false);
                }
            } else {
                if (!IS_PIN_SAFETY_BEAM()) {
                    // Close the gate after the button got released.
                    if (GATE_CLOSED != motors[MOTOR_1].gate_state) {
                        SetGatePosition(GATE_POSITION_CLOSING_PUSHBUTTON_SINGLE);
                        ExecuteGateCycle(false);
                    }
                }
            }
        }
    }
    if (MC_INPUT_PIN_PUSHBUTTON_DUAL == pin_number) {
        // The pushbutton of a dual cycle is pressed or released.
        // Each transition may start or reverse the operation.
        if (IS_ANY_MOTOR_REVERSE()) {
            ExecuteEmergencyStop(MC_STOP_REASON_SAFETY_BEAM, true, channel);
            SetGatePosition(GATE_POSITION_STOPPED_BY_SAFETY_BEAM);
            downcounter_open_on_safety_beam = MC_SAFETY_BEAM_OPEN_DELAY_MS;
        } else {
            if (IS_PIN_PUSHBUTTON_DUAL()) {
                // Open the gate after the button got pressed.
                if (GATE_OPENED != motors[MOTOR_1].gate_state) {
                    SetGatePosition(GATE_POSITION_OPENING_PUSHBUTTON_DUAL);
                    ExecuteGateCycle(true);
                }
            } else {
                if (!IS_PIN_SAFETY_BEAM()) {
                    // Close the gate after the button got released.
                    if (GATE_CLOSED != motors[MOTOR_1].gate_state) {
                        SetGatePosition(GATE_POSITION_CLOSING_PUSHBUTTON_DUAL);
                        ExecuteGateCycle(true);
                    }
                }
            }
        }
    }

    if (MC_INPUT_PIN_ZW_TRIAC == pin_number) {
        // The Z-Wave input of a dual cycle is pressed or released.
        // Each transition may start or reverse the operation.
        switch (eeprom_shadow.zwave_mode) {
            case ZWAVE_MODE_STATIC:
                //
                // Static Triac output operates identically to dual-gate pushbutton.
                //
                if (IS_ANY_MOTOR_REVERSE()) {
                    ExecuteEmergencyStop(MC_STOP_REASON_ZWAVE_CANCEL, true, channel);
                    SetGatePosition(GATE_POSITION_REVERSED_BY_ZWAVE_CANCEL);
                    downcounter_open_on_safety_beam = MC_SAFETY_BEAM_OPEN_DELAY_MS;
                    inhibit_next_timer_close_on_safety_beam = true;
                } else {
                    if (IS_PIN_ZW_TRIAC()) {
                        // Open the gate after the button got pressed.
                        if (GATE_OPENED != motors[MOTOR_1].gate_state) {
                            SetGatePosition(GATE_POSITION_OPENING_ZWAVE_DUAL);
                            ExecuteGateCycle(true);
                        }
                    } else {
                        if (!IS_PIN_SAFETY_BEAM()) {
                            // Close the gate after the button got released.
                            if (GATE_CLOSED != motors[MOTOR_1].gate_state) {
                                SetGatePosition(GATE_POSITION_CLOSING_ZWAVE_DUAL);
                                ExecuteGateCycle(true);
                            }
                        }
                    }
                }
                break;
            case ZWAVE_MODE_EDGE_RISING:
                if (IS_PIN_ZW_TRIAC()) {
                    // Rising edge
                    if (IS_ALL_MOTORS_IDLE()) {
                        SetGatePosition(GATE_POSITION_CYCLE_ZWAVE_EDGE_DUAL);
                        ExecuteGateCycle(true);
                    }
                }
                break;
            case ZWAVE_MODE_EDGE_FALLING:
                if (!IS_PIN_ZW_TRIAC()) {
                    // Falling edge
                    if (IS_ALL_MOTORS_IDLE()) {
                        SetGatePosition(GATE_POSITION_CYCLE_ZWAVE_EDGE_DUAL);
                        ExecuteGateCycle(true);
                    }
                }
                break;
            case ZWAVE_MODE_EDGE_ANY:
                // Any edge, do not check which one it is.
                if (IS_ALL_MOTORS_IDLE()) {
                    SetGatePosition(GATE_POSITION_CYCLE_ZWAVE_EDGE_DUAL);
                    ExecuteGateCycle(true);
                }
                break;
            default:
                break;
        } /* switch */
    } /* if (triac event) */

    if ((MC_INPUT_PIN_CYCLE_SINGLE == pin_number) && IS_PIN_CYCLE_SINGLE()) {
        // The button of a single cycle is depressed.
        if (IS_ALL_MOTORS_IDLE()) {
            SetGatePosition(GATE_POSITION_CYCLE_PIN_SINGLE);
            ExecuteGateCycle(false);
        }
    }
    if ((MC_INPUT_PIN_CYCLE_DUAL == pin_number) && IS_PIN_CYCLE_DUAL()) {
        // The button of a dual cycle is depressed.
        if (IS_ALL_MOTORS_IDLE()) {
            SetGatePosition(GATE_POSITION_CYCLE_PIN_DUAL);
            ExecuteGateCycle(true);
        }
    }
}

static void PollInputPins(int channel)
{
    for (uint8_t k=0; k < NUM_INPUT_PINS; k++) {
        input_pin_t * const ip = input_pins + k;
        uint8_t value = gpio_get_pin_value(ip->pin_number) ? 1 : 0;
        // Feed the value into the debouncer
        if (value != ip->state) {
            if (ip->debounce_counter > 0) {
                ip->debounce_counter--;
                if (0 == ip->debounce_counter) {
                    ip->state = value;
                    OnInputPinStateChange(k, channel);
                }
            }
        } else {
            // The pin has already settled and no state change is required.
            ip->debounce_counter = MC_INPUT_PIN_DEBOUNCE_TIME_MS;
        }
    }
    if (IS_PIN_EMERGENCY_STOP() && IS_ANY_MOTOR_POWERED()) {
        ExecuteEmergencyStop(MC_STOP_REASON_INPUT_PIN, true, channel);
    }
}

static void PollVoltages(int channel)
{
    flag_battery_ok = (adc_readings_volts[MC_ADC_CHANNEL_BATTERY_VOLTAGE] >= MC_BATTERY_MINIMUM_VOLTAGE);
}

static void RunTasksOnTimerTick(int channel)
{
    check_adc();
    for (motor_number_t mi=0; mi < NUM_MOTORS; mi++) {
        SPI_ExchangeWithMotorDriver(mi);
        ProcessMotorState(mi, channel);
    }
    ProcessGateState();
    UpdateLedState();
    PollInputPins(channel);
    PollVoltages(channel);
}

static void InitializeMotor(motor_t *mp, motor_number_t mi)
{
    mp->number = mi;
    mp->state = MOTOR_STATE_IDLE;
    mp->gate_state = GATE_CLOSED;
    mp->max_running_time_ms = SECONDS_TO_MS(MC_MOTOR_MAX_RUNNING_TIME_SECS);
    mp->running_time_downcounter_ms = 0;
    mp->start_delay_ms = 0;
    mp->start_delay_ms = 0;
    mp->stop_reasons = 0;
    mp->overload_current = MC_MAX_OVERCURRENT_THRESHOLD_AMPS; // TODO: Flash
}

static void InitializeInputPins(void)
{
    uint8_t k;
    for (k=0; k < NUM_INPUT_PINS; k++) {
        input_pins[k].debounce_counter = MC_INPUT_PIN_DEBOUNCE_TIME_MS;
    }
    k = 0;
    input_pins[k++].pin_number = PIN_INP0;
    input_pins[k++].pin_number = PIN_INP1;
    input_pins[k++].pin_number = PIN_INP2;
    input_pins[k++].pin_number = PIN_INP3;
    input_pins[k++].pin_number = PIN_INP4;
    input_pins[k++].pin_number = PIN_INP5;
    input_pins[k++].pin_number = PIN_INP6;
    input_pins[k++].pin_number = PIN_INP7;
    input_pins[k++].pin_number = PIN_INP8;
    input_pins[k++].pin_number = PIN_INP9;
    input_pins[k++].pin_number = PIN_ZW_TRIAC;
}

static void QueueGatePosition(gate_position_t gs, int channel)
{
    switch (gs) {
        case GATE_POSITION_IDLE:
            queue_string("IDLE", channel);
            break;
        case GATE_POSITION_MANUAL_MOTOR_OPERATION:
            queue_string("MANUAL_MOTOR_OPERATION", channel);
            break;
        case GATE_POSITION_OPENING_BY_COMMAND_SINGLE:
            queue_string("OPENING_BY_COMMAND_SINGLE", channel);
            break;
        case GATE_POSITION_CLOSING_BY_COMMAND_SINGLE:
            queue_string("CLOSING_BY_COMMAND_SINGLE", channel);
            break;
        case GATE_POSITION_OPENING_BY_COMMAND_DUAL:
            queue_string("OPENING_BY_COMMAND_DUAL", channel);
            break;
        case GATE_POSITION_CLOSING_BY_COMMAND_DUAL:
            queue_string("CLOSING_BY_COMMAND_DUAL", channel);
            break;
        case GATE_POSITION_OPENING_PUSHBUTTON_SINGLE:
            queue_string("OPENING_PUSHBUTTON_SINGLE", channel);
            break;
        case GATE_POSITION_CLOSING_PUSHBUTTON_SINGLE:
            queue_string("CLOSING_PUSHBUTTON_SINGLE", channel);
            break;
        case GATE_POSITION_OPENING_PUSHBUTTON_DUAL:
            queue_string("OPENING_PUSHBUTTON_DUAL", channel);
            break;
        case GATE_POSITION_CLOSING_PUSHBUTTON_DUAL:
            queue_string("CLOSING_PUSHBUTTON_DUAL", channel);
            break;
        case GATE_POSITION_OPENING_ZWAVE_DUAL:
            queue_string("OPENING_ZWAVE_DUAL", channel);
            break;
        case GATE_POSITION_CLOSING_ZWAVE_DUAL:
            queue_string("CLOSING_ZWAVE_DUAL", channel);
            break;
        case GATE_POSITION_REVERSED_BY_ZWAVE_CANCEL:
            queue_string("REVERSED_BY_ZWAVE_CANCEL", channel);
            break;
        case GATE_POSITION_STOPPED_BY_SAFETY_BEAM:
            queue_string("STOPPED_BY_SAFETY_BEAM", channel);
            break;
        case GATE_POSITION_OPENING_BY_SAFETY_BEAM:
            queue_string("OPENING_BY_SAFETY_BEAM", channel);
            break;
        case GATE_POSITION_CYCLE_PIN_SINGLE:
            queue_string("CYCLE_PIN_SINGLE", channel);
            break;
        case GATE_POSITION_CYCLE_PIN_DUAL:
            queue_string("CYCLE_PIN_DUAL", channel);
            break;
        case GATE_POSITION_CYCLE_ZWAVE_EDGE_DUAL:
            queue_string("CYCLE_ZWAVE_EDGE_DUAL", channel);
            break;
        case GATE_POSITION_WAITING_FOR_TIMER_CLOSE:
            queue_string("WAITING_FOR_TIMER_CLOSE", channel);
            break;
        case GATE_POSITION_CLOSING_BY_TIMER:
            queue_string("CLOSING_BY_TIMER", channel);
            break;
        default:
            queue_string("?", channel);
            break;
    }
}

static void PrintGatePosition(int channel)
{
    queue_string("=g: ", channel);
    QueueGatePosition(gate_position, channel);
    queue_char(0, channel);
}

static void SetGatePosition(gate_position_t gpos)
{
    gate_position_t old_gpos = gate_position;
    gate_position = gpos;
    if (log_gate_status_transitions) {
        queue_string("=g: GATE ", default_com_channel);
        QueueGatePosition(old_gpos, default_com_channel);
        queue_string(" => ", default_com_channel);
        QueueGatePosition(gpos, default_com_channel);
        queue_char(0, default_com_channel);
    }
}

int main (void)
{
    uint64_t old_timer_ticks;

    timer_ticks = old_timer_ticks = 0; // in milliseconds

    Start_PLL();
    ConfigurePins();
    Configure_UARTs();
    Configure_Timer();
    Configure_ADC();
    Configure_SPI();
    InitializeInputPins();

    for (motor_number_t mi=0; mi < NUM_MOTORS; mi++) {
        InitializeMotor(motors + mi, mi);
    }

    Initialize_EEPROM();

    while (true) {
        // First, we run all tasks that should be done on timer ticks.
        if (old_timer_ticks != timer_ticks) {
            RunTasksOnTimerTick(default_com_channel);
            old_timer_ticks = timer_ticks;
        }
        // Then we run tasks that can run whenever.
        Poll_UARTs();
        poll_action_button();
    }
}
