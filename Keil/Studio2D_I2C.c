/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

// Use PGO to reset VL53LIX via XSHUT
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                    // Activate clock for Port J
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};    // Allow time for clock to stabilize
    GPIO_PORTJ_DIR_R &= ~0x03;                                            // Make PJ1 input 
    GPIO_PORTJ_DEN_R |= 0x03;                                             // Enable digital I/O on J
    
    GPIO_PORTJ_PCTL_R &= ~0x000000F0;                                     //? Configure PJ1 as GPIO 
    GPIO_PORTJ_PUR_R |= 0x03;                                                    //    Enable weak pull up resistors
}

void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};         
	GPIO_PORTH_DIR_R = 0b00001111;       
  GPIO_PORTH_DEN_R = 0b00001111;  
	return;
}

void flashLED3(int count) { //on board LED4
	while(count--){		
	GPIO_PORTL_DATA_R ^= 0b00001000; 								
			GPIO_PORTF_DATA_R ^= 0b00000001;	
			SysTick_Wait10ms(10);	
			GPIO_PORTL_DATA_R ^= 0b00001000; 								
			GPIO_PORTF_DATA_R ^= 0b00000001;	
	}
}

void rotationControl(int direction, int step, int delay){

	//CCW
	if(direction == -1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
		}

	}
	//CW
	else if(direction == 1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(delay);

		}
	}

}


void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        
    GPIO_PORTG_DATA_R &= 0b11111110;                                 
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            
    
}




uint16_t	dev = 0x29;			
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortJ_Init();
	PortH0H1H2H3_Init();


	status = VL53L1X_GetSensorId(dev, &wordData);
// Wait for the sensor to enter ready state before continuing
while(sensorState==0){
	// Check the boot state of the VL53L1X sensor and set sensorState to the result
	status = VL53L1X_BootState(dev, &sensorState);
	// Wait for 100ms before checking the sensor state again
	SysTick_Wait10ms(10);
}

// Flash all the LEDs to indicate that the sensor is ready
FlashAllLEDs();

// Clear the interrupt status of the VL53L1X sensor
status = VL53L1X_ClearInterrupt(dev); 

// Initialize the sensor with default settings
status = VL53L1X_SensorInit(dev);

// Check if there was an error during initialization
Status_Check("SensorInit", status);

// Start the ranging measurement on the VL53L1X sensor
status = VL53L1X_StartRanging(dev);

	
	int dir = 1;
	int steps = 512;
	int delay = 1200;
	int rotate = 0;
	int count = 0;
	int depth = 0;
	int scan = 0;
	FlashLED1(1);
	
	while (1){
		// Check if button is pressed
	if((GPIO_PORTJ_DATA_R&0x01)==0){
		// Wait for button to be released
		while((GPIO_PORTJ_DATA_R&0x01)==0){SysTick_Wait10ms(10);} // Wait for release
		//Toggle rotate flag
		rotate = !rotate;
		}
	// Check if 'scan' flag is set
	if(scan == 1) {
		
		while((GPIO_PORTJ_DATA_R&0x01)==0){SysTick_Wait10ms(100);}
		// Flash PN1
		FlashLED1(1);
		
		for(int i = 0; i < 1; i++) {
			// Wait for sensor to indicate data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						// Flash PN0
						FlashLED2(1);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			// Retrieve sensor data
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					
			status = VL53L1X_GetSignalRate(dev, &SignalRate);
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			status = VL53L1X_GetSpadNb(dev, &SpadNum);

			status = VL53L1X_ClearInterrupt(dev); 
			int step = count;
			if (depth%2 == 1) { 
				step = 512-count;
			}
			sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, step, depth, SpadNum);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(50);
			}
		//Clear scan flag
		scan = 0;
   }
	 
	if (rotate == 1){
		rotationControl(dir, 1, delay);
		count++;
	}
	
	// Run every 11.25 deg
	if (count % 16 == 0 && rotate == 1 && count != 0){
		SysTick_Wait(100000); // Stabilize
		scan = 1; // Take reading
	}

	// Switch direction and increase depth
	if (count > 512){ //512 is the equivalent to 360 deg 
			rotate = 0;
			count = 0;
			dir *= -1;
			depth += 1;
		FlashLED2(depth); //Flash PN0 to indicate the amount of scans already conducted
	}
}

	VL53L1X_StopRanging(dev);
  while(1) {}
}
