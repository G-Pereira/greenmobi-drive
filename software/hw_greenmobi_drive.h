#ifndef HW_GREENMOBI_DRIVE_H_
#define HW_GREENMOBI_DRIVE_H_

#define HW_NAME					"GREENMOBI_DRIVE"

// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS

// Macros
#define LED_GREEN_GPIO			  GPIOA
#define LED_GREEN_PIN			    6
#define LED_RED_GPIO			    GPIOA
#define LED_RED_PIN				    5

#define LED_GREEN_ON()		    palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()		    palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			    palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			    palClearPad(LED_RED_GPIO, LED_RED_PIN)

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_EXT3
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT3			10
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOS_2		8
#define ADC_IND_TEMP_MOS_3		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)
#define V_REG					      3.44
#define VIN_R1					    56000.0
#define VIN_R2					    2200.0
#define CURRENT_AMP_GAIN		50.0
#define CURRENT_SHUNT_RES		0.0002
#define NTC_MOS_PULLUP      5100.0
#define NTC_MOS_BETA        3380.0
#define NTC_MOTOR_PULLUP    5100.0

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

#define NTC_RES_MOTOR(adc_val)	(NTC_MOTOR_PULLUP / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / NTC_MOTOR_PULLUP) / beta) + (1.0 / 298.15)) - 273.15)

#define NTC_RES(adc_val)		((4095.0 * NTC_MOS_PULLUP) / adc_val - 10000.0)
#define NTC_TEMP_MOS1()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / NTC_MOS_PULLUP) / NTC_MOS_BETA) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / NTC_MOS_PULLUP) / NTC_MOS_BETA) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS3()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / NTC_MOS_PULLUP) / NTC_MOS_BETA) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP(adc_ind)   NTC_TEMP_MOS1()

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#define CURR1_DOUBLE_SAMPLE		1
#define CURR2_DOUBLE_SAMPLE		1
#define CURR3_DOUBLE_SAMPLE		1

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		8
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		6
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

#define BMI160_SDA_GPIO			GPIOB
#define BMI160_SDA_PIN			2
#define BMI160_SCL_GPIO			GPIOA
#define BMI160_SCL_PIN			15
//#define IMU_FLIP
//#define IMU_ROT_180

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		1000.0

// Default setting overrides
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			15.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			65.0	// Maximum input voltage
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					30000.0
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		4.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			4.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-4.0	// Input current limit in Amperes (Lower)
#endif

// Setting limits
#define HW_LIM_CURRENT			-3.5, 3.5
#define HW_LIM_CURRENT_IN		-3.5, 3.5
#define HW_LIM_CURRENT_ABS		0.0, 4.0
#define HW_LIM_VIN				15.0, 65.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

// Unused pins

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// UART Peripheral
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11

#define HW_UART_GPIO_AF  HW_UART_P_GPIO_AF

#define HW_UART_DEV         HW_UART_P_DEV
#define HW_UART_TX_PORT     HW_UART_P_TX_PORT
#define HW_UART_TX_PIN      HW_UART_P_TX_PIN
#define HW_UART_RX_PORT     HW_UART_P_RX_PORT
#define HW_UART_RX_PIN      HW_UART_P_RX_PIN

// SPI
#define HW_SPI_DEV       SPID1
#define HW_SPI_GPIO_AF   GPIO_AF_SPI1
#define HW_SPI_PORT_NSS  GPIOA
#define HW_SPI_PIN_NSS   4
#define HW_SPI_PORT_SCK  GPIOB
#define HW_SPI_PIN_SCK   3
#define HW_SPI_PORT_MOSI GPIOB
#define HW_SPI_PIN_MOSI  5
#define HW_SPI_PORT_MISO GPIOB
#define HW_SPI_PIN_MISO  4

// Disable NRF/LoRa
//#define HW_HAS_NRF
//#define USE_APP_NRF
//#define USE_APP_LORA

#endif /* HW_GREENMOBI_DRIVE_H_ */

