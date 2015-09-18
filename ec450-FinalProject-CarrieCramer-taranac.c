#include "msp430g2553.h"

#define LED_PIN1 0x02 //LED powered by P1.1
#define LED_PIN2 0x04 //LED powered by P1.2

#define ADC_BIT_4 0x10 // ADC input P1.4
#define ADC_BIT_5 0x20 // ADC input P1.5

#define MOTOR_PIN1 0x40 // Motor 1 output P1.6
#define MOTOR_PIN2 0x80 // Motor 2 output P1.7

volatile int latest_result[2]; // ADC buffer

#define MAX_THRESHOLD 50 // PWM Duty Cycle Thresholds
#define MIN_THRESHOLD 45

#define NUM_SAMPLES 6 // Number of samples taken for calibration

volatile int min_sensor1 = 0; // Initialize min and max values for sensor readings
volatile int max_sensor1 = 0;
volatile int min_sensor2 = 0;
volatile int max_sensor2 = 0;

volatile unsigned int duty_cycle_count = 0; // Initialize counter for PWM (using Timer A)

volatile unsigned int speed_motor1 = 0; // Initialize duty cycles for motors 1 and 2
volatile unsigned int speed_motor2 = 0;

unsigned int result_sum1 = 0; // Initialize running sums for sensor calibration
unsigned int result_sum2 = 0;


// Function declarations
void start_conversion(void); // ADC conversion
void init_adc(void);	// Initialize ADC
volatile unsigned int duty_cycle(unsigned int adc_val, unsigned int min_sensor, unsigned int max_sensor); // Calculate duty cycles for a sensor reading
void init_timerA(void); // Initialize Timer A
void ms_delay(unsigned int ms); // Milisecond delay
void calibrate(void); // Calibrate sensors


void main() {

	WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer
	BCSCTL1 = CALBC1_8MHZ;			// 8Mhz calibration for clock
  	DCOCTL  = CALDCO_8MHZ;

  	init_adc(); // Initialize ADC

  	P1DIR |= LED_PIN1 + LED_PIN2;		// Set LED to output direction
  	P1OUT |= LED_PIN1 + LED_PIN2;		// Turn on LEDs

  	calibrate(); // Calibrate sensors

  	P1DIR |= MOTOR_PIN1 + MOTOR_PIN2;	// Set motor pins to output direction

  	init_timerA(); // Initialize Timer A

  	__enable_interrupt(); // Enable interrupts


 	while(1) { // Loop forever

 		start_conversion();   // Do an ADC conversion
 		__delay_cycles(100); // Delay for 50 cycles

 	  	speed_motor1 = duty_cycle(latest_result[0], min_sensor1, max_sensor1); // Set duty cycle for motor 1

 	  	speed_motor2 = duty_cycle(latest_result[1], min_sensor2, max_sensor2); // Set duty cycle for motor 2

 	}
}


// Perform ADC conversion
void start_conversion(){

	while (ADC10CTL1 & ADC10BUSY);    // Wait for ADC Conversion
	ADC10CTL0 |= ADC10SC;			  // Start conversion
	ADC10SA=(unsigned) latest_result; // Results stored in latest_result buffer
}

// Initialize the ADC
void init_adc(){

	ADC10CTL1= INCH_5 		// Highest input channel is A5
 			  +SHS_0 		//use ADC10SC bit to trigger sampling
 			  +ADC10DIV_4 	// ADC10 clock/5
 			  +ADC10SSEL_0 	// Clock Source=ADC10OSC
 			  +CONSEQ_1 	// multichannel, single conversion
 			  ;

 			  ADC10AE0= ADC_BIT_4 + ADC_BIT_5; // enable A4 and A5 analog input

 			  ADC10DTC1= 2;   // 2 block per transfer

 	ADC10CTL0= SREF_0	//reference voltages are Vss and Vcc
 	          +ADC10SHT_3 //64 ADC10 Clocks for sample and hold time (slowest)
 	          +ADC10ON	//turn on ADC10
 	          +ENC		//enable (but not yet start) conversions
 	          +MSC
 	          ;
}


volatile unsigned int duty_cycle(unsigned int adc_val, unsigned int min_sensor, unsigned int max_sensor)
{
	volatile unsigned int dc;

	// Ensure that the value read by the sensor is within range
	if (adc_val > max_sensor)
		adc_val = max_sensor;
	if (adc_val < min_sensor)
	 	adc_val = min_sensor;

	// Calculate the result and put it within 0 to 100% PWM Duty Cycle value
	dc = 100 - (adc_val - min_sensor) * 100 / (max_sensor - min_sensor);

	if (dc <= MIN_THRESHOLD)
		dc = 0;
	if (dc >= MAX_THRESHOLD)
		dc = MAX_THRESHOLD;

	return(dc);
}


void interrupt timerA_handler () {

  duty_cycle_count++; // Increment duty cycle counter

  // If the counter exceeds 100, a full duty cycle has completed, so reset counter to zero and turn on the motors
  if (duty_cycle_count >= 100) {
    duty_cycle_count = 0;
    P1OUT |= MOTOR_PIN1;
    P1OUT |= MOTOR_PIN2;
  }

  // If the counter reaches the duty cycle for motor 1, turn off motor 1
  if (duty_cycle_count == speed_motor1) {
    P1OUT &= ~MOTOR_PIN1;
  }

  // If the counter reaches the duty cycle for motor 2, turn off motor 2
  if (duty_cycle_count == speed_motor2) {
    P1OUT &= ~MOTOR_PIN2;
  }
}
ISR_VECTOR(timerA_handler,".int09") // declare interrupt vector


void init_timerA() {
	TACCTL0 = CCIE;				// CCR0 interrupt enabled
	TACCR0 = 99;				// Interrupt will occur every 100 cycles
	TACTL = TASSEL_2 + MC_1;    // Start Timer, use SMCLK, Up Mode
}


void ms_delay(unsigned int ms)
{
  while(ms--) {
    __delay_cycles(8000);     // Delay 8000 cycles per millisecond because the clock is 8MHz
  }
}

void calibrate()
{
	latest_result[0] = 0; // Initialize ADC buffer values to zero
	latest_result[1] = 0;

	// Calculate maximum sensor values while sensors are placed over the black line

	unsigned char i;
	result_sum1 = 0; // Initialize the running sums that will store the sensor readings
	result_sum2 = 0;

	for(i = 0; i < NUM_SAMPLES; i++) {
		start_conversion();					// Do an ADC conversion
		__delay_cycles(5000);				// Delay for 5000 cycles while the conversion completes
		result_sum1 += latest_result[0];    // Add the sensor results to the running sums
		result_sum2 += latest_result[1];
	}

	max_sensor1 = result_sum1/NUM_SAMPLES;	// Average the the sensor readings
	max_sensor2 = result_sum2/NUM_SAMPLES;


	// Flash the LED 5 times to indicate that calibration for black is complete
	// User should move the robot onto white surface during this time
	for(i = 0; i < 10; i++) {
		P1OUT ^= LED_PIN1 + LED_PIN2;
		ms_delay(1000);
	}

	ms_delay(4000); // Delay for 4 seconds


	// Calculate minimum sensor values while sensors are placed over the white surface

	result_sum1 = 0; // Set the running sums back to zero
	result_sum2 = 0;

	for(i = 0; i < NUM_SAMPLES; i++) {
		start_conversion();
		__delay_cycles(5000);
		result_sum1 += latest_result[0];
		result_sum2 += latest_result[1];
	}

	min_sensor1 = result_sum1/NUM_SAMPLES;   // Average the the sensor readings
	min_sensor2 = result_sum2/NUM_SAMPLES;

	// Flash the LEDs 5 times to indicate that calibration is complete
	// The user should place the robot onto the path
	for(i = 0; i < 10; i++) {
		P1OUT ^= LED_PIN1 + LED_PIN2;
		ms_delay(1000);
	}
}
