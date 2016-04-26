// C/C++ libraries
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <unistd.h>

// Pi specific libraries
#include "bcm2835.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

// ADC configuration data
// #define GPIOCONFIG  0b0100000000000111 - unused
#define ADCCONFIG   0b0010000001110000  // 0x2070
#define DACCONFIG   0b0010100000001011  // 0x280B
#define GPCONFIG    0b0001100000000000  // 0x1800
#define GPIOCONFIG  0b0100000110000000  // 0x4180
#define ADCSEQUENCE 0b0001001001110000  // 0x1270
#define RESET       0b0111110110101100  // 0x7DAC
#define WORD_SIZE		2

// DAC write-out addresses
#define DAC0_WRITE  0b1000000000000000
#define DAC1_WRITE  0b1001000000000000
#define DAC2_WRITE  0b1011000000000000

// Corresponding 12-bit DAC values using 0-5V range
#define FIVE_VOLTS 4095
#define ONE_VOLT 808
#define GROUNDED 0

// Seven segement delay
#define SEGDELAY 750

// Case statement variables
#define TBD 0
#define GATE 1
#define SOURCE 2
#define DRAIN 3
#define NMOS 4
#define PMOS 5
#define MOSFET 6
#define BJT 7
#define NPN 8
#define PNP 9
#define BASE 10
#define COLLECTOR 11
#define EMITTER 12

#define SIZE 10

// Set GPIO pin locations
#define RESET_PIN RPI_BPLUS_GPIO_J8_07
#define TEST_PIN  RPI_BPLUS_GPIO_J8_11
#define T1_PIN RPI_BPLUS_GPIO_J8_13
#define T2_PIN RPI_BPLUS_GPIO_J8_15
#define T3_PIN RPI_BPLUS_GPIO_J8_16

// Various variables
#define RESISTOR 470 //resistor value, can change or be set to a variable if needed
#define VMAX 5.00 //value for testing right now, can change based upon range
#define ADCMAX 3972 //3972

// Samples and floating-point samples
#define SAMPLES 500
#define SAMPLESF 500.0

// global arrays
float volts_ct[SAMPLES]; //x-axis value storage (decimal values from 0-VMAX, to be graphed)
int volts_adc[SAMPLES]; //values to send to DAC from 0-4095
float curr[SAMPLES]; //y-axis value storage

// global counter
int xAxisCnt;

// fixed globals used for curve trace
float voltsVDS[SAMPLES];

// More globals, we love these (bad programmer, BAD!)
int mosfet[3]; //simulated MOSFET: then Type
int volts[3];
int terminal_id[3] = {0,0,0};
int buttonRead = 1;
float vgsCorrected;
float PbjtBase[6] = {5.0, 4.5, 4.0, 3.5, 3.0, 2.5};
float NbjtBase[6] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5};

int calVolts;   // Calibration level voltage

//file name
char fname[1000];

// I/O data variables
char spiOut[2];
char spiIn[2] = {0, 0};

// Create two byte-size packets from 16-bit word for transmission to 5592
void makeWord(char eightBits[], unsigned short sixteenBits)
{
	eightBits[0] = sixteenBits >> 8;
	eightBits[1] = sixteenBits & 0x00FF;
}

// Pull ground level for ADC from this function
int AD5592_calibration(void){

    // Set all voltages to ground
    int calVoltage = GROUNDED;
    int gndRead = 0;
    int gndReadMax = 0;
    int i = 0; // counter

    // ground all DACs
    spiOut[0] = (calVoltage | DAC0_WRITE) >> 8;   // Term 1
    spiOut[1] = (calVoltage | DAC0_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (calVoltage | DAC1_WRITE) >> 8;   // Term 2
    spiOut[1] = (calVoltage | DAC1_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (calVoltage | DAC2_WRITE) >> 8;   // Term 3
    spiOut[1] = (calVoltage | DAC2_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    for(i = 0; i < 90; i++){ // average ADC1 readings (30)
            makeWord(spiOut, 0b0000000000000000); //no op command, data in garbage
            bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE); // no op again, should be good ADC1 data
            spiIn[0] = spiIn[0] & 0x0F;
            gndRead = spiIn[0]; // Place result into read-out array
            gndRead = (gndRead << 8) | spiIn[1];

            // Grab maximum ground value
            if (gndRead > gndReadMax){
                gndReadMax = gndRead;
            }
        }

    // return base ground level
    return abs(gndReadMax);
}

// Cycles voltages to identify gate terminal (or lack thereof) on a MOSFET
int volt_cycle(int tone, int ttwo, int tthree){

    // variables used for ADC reads and math
    int m1, m2, m3, c1=0, c2=0, c3=0;
    int cnt = 0, ADC1cnt = 0, ADC2cnt = 0, ADC3cnt = 0;
    int ADC1read = 0, ADC2read = 0, ADC3read = 0;
    int ADC1readSum = 0, ADC2readSum = 0, ADC3readSum = 0;
    int ADC1drop = 0, ADC2drop = 0, ADC3drop = 0;

    // checks source of the incoming ADC data
    int spiInCheck = 0;

    // counters
    int mos_cnt = 0; int bjt_cnt = 0;
    int c1_cnt = 0; int c2_cnt = 0; int c3_cnt = 0;

    // voltages defined in globals, self-explanatory
    volts[0] = FIVE_VOLTS;
    volts[1] = ONE_VOLT;
    volts[2] = GROUNDED;

    for(int kk = 0; kk<29; kk++){

        c1 = 0; c2 = 0; c3 = 0;

        sort(volts, volts+3);

	do{

        ADC1cnt = 0;
        ADC1readSum = 0;
        ADC2cnt = 0;
        ADC2readSum = 0;
        ADC3cnt = 0;
        ADC3readSum = 0;

        // write data to DACs
        spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
        spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
        spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
        spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = ADCSEQUENCE >> 8;
        spiOut[1] = ADCSEQUENCE & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

                                                                ADC Reads

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

        // printf("Volts: %d %d %d\n", volts[0], volts[1], volts[2]);

        for(cnt = 0; cnt < 90; cnt++){

            makeWord(spiOut, 0b0000000000000000); //no op command, data in
            bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
            spiInCheck = (spiIn[0] >> 4) & 0x07;
            // printf("spiCheck: %d\n", spiInCheck);

            if(spiInCheck == 4){
                spiIn[0] = spiIn[0] & 0x0F;
                ADC1read = spiIn[0]; // Place result into read-out array
                ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                ADC1readSum += ADC1read;
                ADC1cnt++;
                //printf("ADC1: %d\n", ADC1read);
            }

            else if(spiInCheck == 5){
                spiIn[0] = spiIn[0] & 0x0F;
                ADC2read = spiIn[0]; // Place result into read-out array
                ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                ADC2readSum += ADC2read;
                ADC2cnt++;
            }

            else if(spiInCheck == 6){
                spiIn[0] = spiIn[0] & 0x0F;
                ADC3read = spiIn[0]; // Place result into read-out array
                ADC3read = (ADC3read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                ADC3readSum += ADC3read;
                ADC3cnt++;
                // printf("ADC3read: %d\n", ADC3read);
            }
            else{
                printf("spicheck is %d\n",spiInCheck);
                ADC1cnt++;
                ADC2cnt++;
                ADC3cnt++;
            }
        }
        // check the voltage drops across each terminal resistor, write to read variables
        ADC1read = abs(ADC1readSum / (ADC1cnt));
        ADC1drop = abs(volts[0] - ADC1read);

        ADC2read = abs(ADC2readSum / (ADC2cnt));
        ADC2drop = abs(volts[1] - ADC2read);

        ADC3read = abs(ADC3readSum / (ADC3cnt));
        ADC3drop = abs(volts[2] - ADC3read);

        // increment counters based on voltage drops -- we're looking for a terminal with no current (i.e. gate)
		if(ADC1drop < calVolts){c1++;} //note some values will have to change based on number ranges recieved by ADC/DAC

		if(ADC2drop < calVolts){c2++;}

		if(ADC3drop < calVolts){c3++;}
	} while (next_permutation(volts, volts+3)); // run through all possible voltage permutations

    // some summing variables
	c1_cnt += c1; c2_cnt += c2; c3_cnt += c3;

    // is there a gate?
    if( (c1 == 6) || (c2 == 6) || (c3 == 6)){
		mos_cnt++;
	}
	// if not, it's a BJT
	else {
		bjt_cnt++;
	}

	}
    // if there's a gate, what terminal is it located on?
    if (mos_cnt > bjt_cnt){
        if( (c1_cnt > c2_cnt) && (c1_cnt > c3_cnt) ){
            return 1;
        } else if( (c2_cnt > c1_cnt) && (c2_cnt > c3_cnt) ){
            return 2;
        } else if( (c3_cnt > c1_cnt) && (c3_cnt > c2_cnt) ){
            return 3;
        }
    }
	else {
		return 0;
	}
}

// If the device is a MOSFET, identifies PMOS vs. NMOS device
int type_finder(int nongate, int gate){

    int m1, m2;
    int cnt = 0;
    int nongateRead, nongateReadSum, nongateReadBefore, nongateReadAfter;
    int nongatecnt = 0;
    int nongateIO = 0x10 << nongate;
    int spiInCheck;

    // create new ADC sequence from nongate terminal number
    int newADCSequence = (ADCSEQUENCE & 0b0001001000000000) | nongateIO;

    // ground entire voltage array (temporary)
    volts[0] = GROUNDED;
    volts[1] = GROUNDED;
    volts[2] = GROUNDED;

    // move proper voltages to gate and random non-gate terminal
    volts[gate] = FIVE_VOLTS;
    volts[nongate] = ONE_VOLT;

    // write data to DACs
    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = newADCSequence >> 8;
    spiOut[1] = newADCSequence & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    makeWord(spiOut, 0b0000000000000000); //no op command, garbage in
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    for(cnt = 0; cnt < 30; cnt++){

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
        spiInCheck = (spiIn[0] >> 4) & 0x07;
        // printf("spiCheck: %d\n", spiInCheck);

        if(spiInCheck == nongate + 4){
            spiIn[0] = spiIn[0] & 0x0F;
            nongateRead = spiIn[0]; // Place result into read-out array
            nongateRead = (nongateRead << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
            nongateReadSum += nongateRead;
            nongatecnt++;
            //printf("ADC1: %d\n", ADC1read);
        }
    }

    // get before value (drop) of nongate terminal
    nongateReadBefore = abs(volts[nongate] - (nongateReadSum / nongatecnt));

    // ground the gate, transfer to DACs, wait 10 ms for gate capacitance to discharge
    volts[gate] = GROUNDED;

    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    delay(10);

    // null critical variables
    nongatecnt = 0;
    nongateReadSum = 0;

    // check values again
    for(cnt = 0; cnt < 30; cnt++){

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
        spiInCheck = (spiIn[0] >> 4) & 0x07;
        // printf("spiCheck: %d\n", spiInCheck);

        if(spiInCheck == nongate + 4){
            spiIn[0] = spiIn[0] & 0x0F;
            nongateRead = spiIn[0]; // Place result into read-out array
            nongateRead = (nongateRead << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
            nongateReadSum += nongateRead;
            nongatecnt++;
            //printf("ADC1: %d\n", ADC1read);
        }
    }

    nongateReadAfter = abs(volts[nongate] - (nongateReadSum / nongatecnt));

    // did the voltage increase or decrease?
    // decrease ->
    if (nongateReadBefore > nongateReadAfter){
        return NMOS;
    }
    // increase ->
    else if (nongateReadBefore < nongateReadAfter){
        return PMOS;
    }
    else{
        return TBD;
    }

}

// Operates 7-segment LED after device type, subtype, and terminals have been identified
void Sev_seg_disp(int type, int sub, int t1, int t2, int t3, int fd){

    printf("\nOutputting data to 7-segment display...\n");

	switch(type){
		case BJT:
			wiringPiI2CWrite(fd, 131); //letter b
			delay(SEGDELAY);
			wiringPiI2CWrite(fd, 225); //letter J
			delay(SEGDELAY);
			wiringPiI2CWrite(fd, 135); //letter T
			delay(SEGDELAY);
			wiringPiI2CWrite(fd, 127); //decimal
			delay(SEGDELAY);
			switch(sub){
				case NPN:
					wiringPiI2CWrite(fd, 171); //letter n
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 140); //letter P
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 171); //letter n
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 127); //decimal
					delay(SEGDELAY);
					break;
				case PNP:
					wiringPiI2CWrite(fd, 140); //letter P
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 171); //letter n
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 140); //letter P
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 127); //decimal
					delay(SEGDELAY);
					break;
				default:
					wiringPiI2CWrite(fd, 255); //display blank
					break;
			}
			switch(t1){
						case BASE:
							wiringPiI2CWrite(fd, 249); //number 1
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 131); //letter b
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case COLLECTOR:
							wiringPiI2CWrite(fd, 249); //number 1
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 198); //letter C
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case EMITTER:
							wiringPiI2CWrite(fd, 249); //number 1
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 134); //letter E
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						default:
							wiringPiI2CWrite(fd, 255); //display blank
							break;
					}
					switch(t2){
						case BASE:
							wiringPiI2CWrite(fd, 164); //number 2
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 131); //letter b
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case COLLECTOR:
							wiringPiI2CWrite(fd, 164); //number 2
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 198); //letter C
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case EMITTER:
							wiringPiI2CWrite(fd, 164); //number 2
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 134); //letter E
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						default:
							wiringPiI2CWrite(fd, 255); //display blank
							break;
					}
					switch(t3){
						case BASE:
							wiringPiI2CWrite(fd, 176); //number 3
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 131); //letter b
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case COLLECTOR:
							wiringPiI2CWrite(fd, 176); //number 3
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 198); //letter C
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case EMITTER:
							wiringPiI2CWrite(fd, 176); //number 3
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 134); //letter E
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						default:
							wiringPiI2CWrite(fd, 255); //display blank
							break;
					}
			break;
		case MOSFET:
			wiringPiI2CWrite(fd, 142); //letter F
			delay(SEGDELAY);
			wiringPiI2CWrite(fd, 134); //letter E
			delay(SEGDELAY);
			wiringPiI2CWrite(fd, 135); //letter T
			delay(SEGDELAY);
			wiringPiI2CWrite(fd, 127); //decimal
			delay(SEGDELAY);
			switch(sub){
				case NMOS:
					wiringPiI2CWrite(fd, 171); //letter n
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 127); //decimal
					delay(SEGDELAY);
					break;
				case PMOS:
					wiringPiI2CWrite(fd, 140); //letter P
					delay(SEGDELAY);
					wiringPiI2CWrite(fd, 127); //decimal
					delay(SEGDELAY);
					break;
				default:
					wiringPiI2CWrite(fd, 255); //display blank
					break;
			}
			switch(t1){
						case GATE:
							wiringPiI2CWrite(fd, 249); //number 1
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 130); //letter G
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case DRAIN:
							wiringPiI2CWrite(fd, 249); //number 1
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 161); //letter d
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case SOURCE:
							wiringPiI2CWrite(fd, 249); //number 1
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 146); //letter S
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						default:
							wiringPiI2CWrite(fd, 255); //display blank
							break;
					}
					switch(t2){
						case GATE:
							wiringPiI2CWrite(fd, 164); //number 2
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 130); //letter G
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case DRAIN:
							wiringPiI2CWrite(fd, 164); //number 2
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 161); //letter d
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case SOURCE:
							wiringPiI2CWrite(fd, 164); //number 2
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 146); //letter S
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						default:
							wiringPiI2CWrite(fd, 255); //display blank
							break;
					}
					switch(t3){
						case GATE:
							wiringPiI2CWrite(fd, 176); //number 3
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 130); //letter G
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case DRAIN:
							wiringPiI2CWrite(fd, 176); //number 3
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 161); //letter d
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						case SOURCE:
							wiringPiI2CWrite(fd, 176); //number 3
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 146); //letter S
							delay(SEGDELAY);
							wiringPiI2CWrite(fd, 127); //decimal
							delay(SEGDELAY);
							break;
						default:
							wiringPiI2CWrite(fd, 255); //display blank
							break;
					}
			break;
		default:
			wiringPiI2CWrite(fd, 255); //display blank
			break;
	}
}

// Displays component data to terminal
int display_id(int one, int two, int three, int type, int subtype){
    switch(type){
        case MOSFET:
            printf("Type: MOSFET\n");
            break;
        case BJT:
            printf("Type: BJT\n");
            break;
        default:
            printf("Type: TBD");
            break;
    }
    switch(subtype){
        case NMOS:
            printf("Subtype: NMOS\n");
            break;
        case PMOS:
            printf("Subtype: PMOS\n");
            break;
        case NPN:
            printf("Subtype: NPN\n");
            break;
        case PNP:
            printf("Subtype: PNP\n");
            break;
        default:
            printf("Subtype: TBD\n");
            break;
    }
    switch(one){ // terminal 1
        case GATE:
            printf("Terminal 1: GATE\n");
            break;
        case SOURCE:
            printf("Terminal 1: SOURCE\n");
            break;
        case DRAIN:
            printf("Terminal 1: DRAIN\n");
            break;
        case BASE:
            printf("Terminal 1: BASE\n");
            break;
        case COLLECTOR:
            printf("Terminal 1: COLLECTOR\n");
            break;
        case EMITTER:
            printf("Terminal 1: EMITTER\n");
            break;
        default:
            printf("Terminal 1: TBD\n");
            break;
    }
    switch(two){ // terminal 2
        case GATE:
            printf("Terminal 2: GATE\n");
            break;
        case SOURCE:
            printf("Terminal 2: SOURCE\n");
            break;
        case DRAIN:
            printf("Terminal 2: DRAIN\n");
            break;
        case BASE:
            printf("Terminal 2: BASE\n");
            break;
        case COLLECTOR:
            printf("Terminal 2: COLLECTOR\n");
            break;
        case EMITTER:
            printf("Terminal 2: EMITTER\n");
            break;
        default:
            printf("Terminal 2: TBD\n");
            break;
    }
    switch(three){ // terminal 3
        case GATE:
            printf("Terminal 3: GATE\n");
            break;
        case SOURCE:
            printf("Terminal 3: SOURCE\n");
            break;
        case DRAIN:
            printf("Terminal 3: DRAIN\n");
            break;
        case BASE:
            printf("Terminal 3: BASE\n");
            break;
        case COLLECTOR:
            printf("Terminal 3: COLLECTOR\n");
            break;
        case EMITTER:
            printf("Terminal 3: EMITTER\n");
            break;
        default:
            printf("Terminal 3: TBD\n");
            break;
    }
}

// Once MOSFET has been identified, differentiates the drain and source
void drain_source(int nongate_a, int nongate_b, int gate, int subtype){
    int m, tested;
    tested = nongate_a;
    int nongateIO = 0x10 << nongate_a;
    int spiInCheck;
    int cnt = 0;
    int nongateRead, nongateReadSum, nongateHold;
    int nongatecnt = 0;

    // create new ADC sequence from nongate terminal number
    int newADCSequence = (ADCSEQUENCE & 0b0001001000000000) | nongateIO;

    // move proper voltages to gate and random non-gate terminal
    volts[gate] = GROUNDED;
    volts[nongate_a] = FIVE_VOLTS;
    volts[nongate_b] = GROUNDED;

    // write data to DACs
    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = newADCSequence >> 8;
    spiOut[1] = newADCSequence & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    makeWord(spiOut, 0b0000000000000000); //no op command, garbage in
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    for(cnt = 0; cnt < 30; cnt++){

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
        spiInCheck = (spiIn[0] >> 4) & 0x07;

        // read one of the non-gate terminals
        if(spiInCheck == nongate_a + 4){
            spiIn[0] = spiIn[0] & 0x0F;
            nongateRead = spiIn[0]; // Place result into read-out array
            nongateRead = (nongateRead << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
            nongateReadSum += nongateRead;
            nongatecnt++;
        }
    }

    nongateHold = volts[nongate_a] - (nongateReadSum / nongatecnt);

    // Move voltage around, reevaluate
    // move proper voltages to gate and random non-gate terminal
    volts[gate] = GROUNDED;
    volts[nongate_b] = FIVE_VOLTS;
    volts[nongate_a] = GROUNDED;

    // write data to DACs
    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    spiOut[0] = newADCSequence >> 8;
    spiOut[1] = newADCSequence & 0x00FF;
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    makeWord(spiOut, 0b0000000000000000); //no op command, garbage in
    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

    nongateReadSum = 0;
    nongatecnt = 0;

    for(cnt = 0; cnt < 30; cnt++){

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
        spiInCheck = (spiIn[0] >> 4) & 0x07;
        // printf("spiCheck: %d\n", spiInCheck);

        // read the non-gate terminal again
        if(spiInCheck == nongate_a + 4){
            spiIn[0] = spiIn[0] & 0x0F;
            nongateRead = spiIn[0]; // Place result into read-out array
            nongateRead = (nongateRead << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
            nongateReadSum += nongateRead;
            nongatecnt++;
            //printf("ADC1: %d\n", ADC1read);
        }
    }

    // verify which terminals have been tested
    if (abs(nongateHold) > abs(volts[nongate_a] - (nongateReadSum / nongatecnt))){
        tested = nongate_a;
    }
    else{
        tested = nongate_b;
    }

    // using body diode principle, identify the source/drain terminal
    if(subtype == NMOS){
        if(tested == nongate_a){
            terminal_id[nongate_a] = SOURCE;
            terminal_id[nongate_b] = DRAIN;
        }
        else if (tested == nongate_b){
            terminal_id[nongate_b] = SOURCE;
            terminal_id[nongate_a] = DRAIN;
        }
    }
    else if(subtype == PMOS){
        if(tested == nongate_a){
            terminal_id[nongate_b] = SOURCE;
            terminal_id[nongate_a] = DRAIN;
        }
        else if (tested == nongate_b){
            terminal_id[nongate_a] = SOURCE;
            terminal_id[nongate_b] = DRAIN;
        }
    }
}

// If device is found to not be a MOSFET, it is determined to be a BJT -- this function differentiates NPN from PNP
int bjt_typer(/*int tone, int ttwo, int tthree*/){
    int c1 = 0;
    int cnt = 0;
    int terminalCheck[3] = {0,0,0};
    //terminalCheck[0] = 0; terminalCheck[1] = 0; terminalCheck[2] = 0;

    int spiInCheck, ADC1read, ADC2read, ADC3read,
    ADC1readSum = 0, ADC2readSum = 0, ADC3readSum = 0,
    ADC1cnt = 0, ADC2cnt = 0, ADC3cnt = 0,
    ADC1drop, ADC2drop, ADC3drop;
    int cathodeIO, newADCSequence;

    for(int i = 0; i < 3; i++){
        volts[0] = GROUNDED; volts[1] = GROUNDED; volts[2] = GROUNDED;
        volts[i] = ONE_VOLT;

        // write data to DACs
        spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
        spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
        spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
        spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        cathodeIO = 0x10 << i;

        // create new ADC sequence from nongate terminal number
        newADCSequence = (ADCSEQUENCE & 0b0001001000000000) | cathodeIO;

        // Using just the ith ADC
        spiOut[0] = newADCSequence >> 8;
        spiOut[1] = newADCSequence & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        ADC1readSum = 0;
        ADC1cnt = 0;

        // Take readings and averages
        for(int ii = 0; ii < 30; ii++){

            makeWord(spiOut, 0b0000000000000000); //no op command, data in
            bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
            spiInCheck = (spiIn[0] >> 4) & 0x07;
            // printf("spiCheck: %d\n", spiInCheck);

            spiIn[0] = spiIn[0] & 0x0F;
            ADC1read = spiIn[0]; // Place result into read-out array
            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
            ADC1readSum += ADC1read;
            ADC1cnt++;
            // printf("ADC1: %d\n", ADC1read);
        }
        ADC1read = abs(ADC1readSum / ADC1cnt);

        ADC1drop = abs(volts[i] - ADC1read);

        if(ADC1drop > calVolts){
            cnt++;
            terminalCheck[i] = 1;
        }
//        printf("Volts: %d %d %d\n", volts[0], volts[1], volts[2]);
//        printf("ADC: %d\n", ADC1read);
//        printf("ADC drop: %d\n\n", ADC1drop);
    }
    //cnt=2;
    // printf("Count: %d\n", cnt);
    if (cnt == 1){
        //printf("IF 1");
        for(int j = 0; j < 3; j++){
            if(terminalCheck[j] == 1){
                terminal_id[j] = BASE;
            }
        }
        return NPN;
    }

     else if (cnt == 2){
        //terminalCheck[0] = 0; terminalCheck[1] = 1; terminalCheck[2] = 1;
        //printf("Inside else if.\n");
        for(int j = 0; j < 3; j++){
            if(terminalCheck[j] != 1){
                //printf("%d is the  base.\n", j);
                terminal_id[j] = BASE;


                //break;
            }

        }
        /*if(terminalCheck[2] == 0){
            terminal_id[2] = BASE;
        } else if(terminalCheck[1] == 0){
            terminal_id[1] = BASE;
        } else {
            terminal_id[0] = BASE;
        }*/
        //printf("Right before return.");
        return PNP;
    }

    else{
        return TBD;
    }
    //printf("Function dead.");
}

// Once BJT is identified, identifies and reports the terminals using difference in beta between forward and reverse active cases
void bjt_terminal_id(int subtype, int base){

   // variables for data holds and math
   int mbase, m, b1, b2, b0;
   int newADCSequence, ADC1read, ADC1readSum, ADC1cnt;
   int ADC2read, ADC2readSum, ADC2cnt;
   int ADC3read, ADC3readSum, ADC3cnt;
    int ADC1drop, ADC2drop, ADC3drop;
    int spiInCheck;

    switch(subtype){
        case NPN:
            switch(base){ // we know where the base is already, run through the different terminals
                case 0:
                   // iteration 1
                   volts[0] = ONE_VOLT; volts[1] = FIVE_VOLTS; volts[2] = GROUNDED;

                    // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001000110000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;
                        // printf("spiCheck: %d\n", spiInCheck);

                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                            //printf("ADC1: %d\n", ADC1read);
                        }

                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    // after reading ADC values, create a collector/base and emitter/base quotient (beta)
                    ADC1drop = abs(volts[0] - ADC1read);
                    ADC2drop = abs(volts[1] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    b1 = m / (mbase+1);

                   // iteration 2
                   volts[0] = ONE_VOLT; volts[2] = FIVE_VOLTS; volts[1] = GROUNDED;
                   // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001010000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;

                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                        }

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    // after reading ADC values, create a collector/base and emitter/base quotient (beta)
                    ADC1drop = abs(volts[0] - ADC1read);
                    ADC2drop = abs(volts[2] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    b2 = m / (mbase+1);

                    if(b1 > b2){
                        terminal_id[1] = COLLECTOR; terminal_id[2] = EMITTER;
                    }
                    else {
                        terminal_id[2] = COLLECTOR; terminal_id[1] = EMITTER;
                    }
                    break;


                case 1:
                    // iteration 1
                   volts[1] = ONE_VOLT; volts[0] = FIVE_VOLTS; volts[2] = GROUNDED;

                    // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001000110000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;

                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                        }

                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    // after reading ADC values, create a collector/base and emitter/base quotient (beta)
                    ADC1drop = abs(volts[1] - ADC1read);
                    ADC2drop = abs(volts[0] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    b1 = m / (mbase+1);

                   // iteration 2
                   volts[1] = ONE_VOLT; volts[2] = FIVE_VOLTS; volts[0] = GROUNDED;
                   // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001100000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;

                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                        }

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[1] - ADC1read);
                    ADC2drop = abs(volts[2] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    b2 = m / (mbase+1);

                    // the higher of the two beta values collected is the forward active case. The collector is at the highest bias
                    if(b1 > b2){
                        terminal_id[0] = COLLECTOR; terminal_id[2] = EMITTER;
                    }
                    else {
                        terminal_id[2] = COLLECTOR; terminal_id[0] = EMITTER;
                    }
                    break;




                case 2:
                    // iteration 1
                   volts[2] = ONE_VOLT; volts[1] = FIVE_VOLTS; volts[0] = GROUNDED;

                    // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001100000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                        }

                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[2] - ADC1read);
                    ADC2drop = abs(volts[1] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    b1 = m / (mbase+1);

                   // iteration 2
                   volts[2] = ONE_VOLT; volts[0] = FIVE_VOLTS; volts[1] = GROUNDED;
                   // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001010000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                        }

                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[2] - ADC1read);
                    ADC2drop = abs(volts[0] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    b2 = m / (mbase+1);

                    if(b1 > b2){
                        terminal_id[1] = COLLECTOR; terminal_id[0] = EMITTER;
                    }
                    else {
                        terminal_id[0] = COLLECTOR; terminal_id[1] = EMITTER;
                    }
                    break;
                default:
                    break;
            }
        break;


/*----------------------------------------------------------------------------------------------------------------------------------------------------------------

                                                            PNP Case

----------------------------------------------------------------------------------------------------------------------------------------------------------------*/

        case PNP:
            int q1, q2, q3, q4;
            switch(base){
                case 0:
                   // iteration 1
                   volts[0] = GROUNDED; volts[1] = ONE_VOLT; volts[2] = GROUNDED;

                    // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001000110000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;


                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;

                        }

                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;

                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[0] - ADC1read);
                    ADC2drop = abs(volts[1] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    q1 = m / (mbase+1);


                   // iteration 2
                   volts[0] = GROUNDED; volts[2] = ONE_VOLT; volts[1] = GROUNDED;
                   // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001010000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;


                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;

                        }

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;

                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[0] - ADC1read);
                    ADC2drop = abs(volts[2] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    q2 = m / (mbase+1);

                    //printf("beta: %d\n", q2);

                    if(q1 > q2){
                        terminal_id[2] = COLLECTOR; terminal_id[1] = EMITTER;
                    }
                    else {
                        terminal_id[1] = COLLECTOR; terminal_id[2] = EMITTER;
                    }
                    break;




                case 1:

                    // iteration 1
                   volts[1] = GROUNDED; volts[0] = ONE_VOLT; volts[2] = GROUNDED;

                    // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001000110000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;


                        if(spiInCheck == 5){ //was 5
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;

                        }

                        if(spiInCheck == 4){ //was 4
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;

                        }

                    }

                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[1] - ADC1read);
                    ADC2drop = abs(volts[0] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;

                    q1 = m / (mbase+1); //MBASE WAS 0, CAUSING CRASH

                   // iteration 2
                   volts[1] = GROUNDED; volts[2] = ONE_VOLT; volts[0] = GROUNDED;
                   // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001100000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;


                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;

                        }

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[1] - ADC1read);
                    ADC2drop = abs(volts[2] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    q2 = m / (mbase+1);

                    if(q1 > q2){
                        terminal_id[0] = EMITTER; terminal_id[2] = COLLECTOR;
                    }
                    else {
                        terminal_id[0] = COLLECTOR; terminal_id[2] = EMITTER;
                    }
                    break;




                case 2:
                    // iteration 1
                   volts[2] = GROUNDED; volts[1] = ONE_VOLT; volts[0] = GROUNDED;

                    // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001100000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;


                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;

                        }

                        if(spiInCheck == 5){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;

                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    ADC1drop = abs(volts[2] - ADC1read);
                    ADC2drop = abs(volts[1] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    q1 = m / (mbase+1);

                   // iteration 2
                   volts[2] = GROUNDED; volts[0] = ONE_VOLT; volts[1] = GROUNDED;
                   // write data to DACs
                    spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
                    spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
                    spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
                    spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    // create new ADC sequence from 1V and 5V terminals
                    newADCSequence = (ADCSEQUENCE & 0b0001001001010000);

                    // Using just the base ADC
                    spiOut[0] = newADCSequence >> 8;
                    spiOut[1] = newADCSequence & 0x00FF;
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    makeWord(spiOut, 0b0000000000000000); //no op command, data in
                    bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

                    ADC1readSum = 0; ADC2readSum = 0;
                    ADC1cnt = 0; ADC2cnt = 0;

                    // Take readings and averages
                    for(int ii = 0; ii < 60; ii++){

                        makeWord(spiOut, 0b0000000000000000); //no op command, data in
                        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
                        spiInCheck = (spiIn[0] >> 4) & 0x07;
                        // printf("spiCheck: %d\n", spiInCheck);

                        if(spiInCheck == 6){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC1read = spiIn[0]; // Place result into read-out array
                            ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC1readSum += ADC1read;
                            ADC1cnt++;
                            //printf("ADC1: %d\n", ADC1read);
                        }

                        if(spiInCheck == 4){
                            spiIn[0] = spiIn[0] & 0x0F;
                            ADC2read = spiIn[0]; // Place result into read-out array
                            ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                            ADC2readSum += ADC2read;
                            ADC2cnt++;
                        }

                    }
                    ADC1read = abs(ADC1readSum / ADC1cnt);
                    ADC2read = abs(ADC2readSum / ADC2cnt);

                    // after reading ADC values, create a collector/base and emitter/base quotient (beta)
                    ADC1drop = abs(volts[2] - ADC1read);
                    ADC2drop = abs(volts[0] - ADC2read);
                    mbase = ADC1drop;
                    m = ADC2drop;
                    q2 = m / (mbase+1);

                    if(q1 > q2){
                        terminal_id[0] = COLLECTOR; terminal_id[1] = EMITTER;
                    }
                    else {
                        terminal_id[1] = COLLECTOR; terminal_id[0] = EMITTER;
                    }
                    break;
                default:
                    break;
            }
            break;



            }
}

// Hard reset of the DAC/ADC
void AD5592_reset(void){

        // set the reset pin
        bcm2835_gpio_fsel(RESET_PIN, BCM2835_GPIO_FSEL_OUTP);
		bcm2835_gpio_set_pud(RESET_PIN, BCM2835_GPIO_PUD_UP);

        // drop and raise the reset pin level
		bcm2835_gpio_clr(RESET_PIN);
        delay(1);
        bcm2835_gpio_set(RESET_PIN);
        delay(1);

        return;
}

// Config data sent to DAC/ADC
void AD5592_config(void){

	// Array containing 5592 configuration data declared above
	int config[] =
		{
			ADCCONFIG,
			DACCONFIG,
			GPCONFIG,
			GPIOCONFIG,
			ADCSEQUENCE
		};

		// counter variable
		int j =  0;

		// Set number of elements in config array
        int configIteration = sizeof(config)/sizeof(config[0]);

		// Transmit configuration data to 5592
        for(j = 0; j < configIteration; j++)
		{
            spiOut[0] = config[j] >> 8;
            spiOut[1] = config[j] & 0x00FF;
			bcm2835_spi_transfern(spiOut, WORD_SIZE);
		}

		return;
}

// initialize the SPI
void SPI_init(void){

        // Initialize SPI protocol
		if(!bcm2835_init())return;
		bcm2835_spi_begin();
		bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // MSB first
		bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                 // Mode 1
		bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);  // 64 = 3.9 MHz
		bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                    // Chip select 0 line
		bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // Chip select polarity active low

		return;
}

// function generates the X values of the arrays for both the graph and the DAC input
void voltage_ranger(){
	// set the output curve tracer voltage range using samples and ADC maximum value (5V)
	int i;
	float step_size = VMAX / SAMPLESF;
	float step_size_adc = ADCMAX / SAMPLESF;
	for(i=0;i<=SAMPLES-1;i++){
		volts_ct[i] = (i)*step_size;
		volts_adc[i] = (i)*step_size_adc;
	}
}

// output curve trace data to file
void print_csv(float vgs, int type, int subtype, int t1, int t2, int t3){
    FILE *ofp;
    char str[][15] = {"TBD","GATE","SOURCE","DRAIN","NMOS","PMOS","MOSFET","BJT","NPN","PNP","BASE","COLLECTOR","EMITTER"};

    // char outputFilename[] = "curve.csv";

    // set when to write and when to append file
	if( (subtype == NPN && vgs == NbjtBase[0]) || (subtype == PNP && vgs == PbjtBase[0]) || (subtype == NMOS && (vgs == 0)) || (subtype == PMOS && (vgs == 5)) ){
        ofp = fopen(fname, "w"); // write

        switch(type){
            case BJT:
                fprintf(ofp, "Type: %s,Subtype: %s,\n", str[type],str[subtype]);
                fprintf(ofp, "Terminal 1: %s,Terminal 2: %s,Terminal 3: %s\n", str[terminal_id[0]], str[terminal_id[1]], str[terminal_id[2]]);
                fprintf(ofp, "$V_{BE}$,$V_{CE}$,$I_C$\n");
                break;
            case MOSFET:
                fprintf(ofp, "Type: %s,Subtype: %s,\n", str[type],str[subtype]);
                fprintf(ofp, "Terminal 1: %s,Terminal 2: %s,Terminal 3: %s\n", str[terminal_id[0]], str[terminal_id[1]], str[terminal_id[2]]);
                fprintf(ofp, "$V_{GS}$,$V_{DS}$,$I_D$\n");
                break;
            default:
                break;
        }
    }
    else{
        ofp = fopen(fname, "a"); // append
    }
	int i;

	for(i=0;i<=SAMPLES-1;i++){
		fprintf(ofp, "%f,%f,%f\n", vgs, voltsVDS[i], curr[i]);
	}
	fclose(ofp);
}

// extension of adcdac_return function
void adcdac_returnExt(int gateBase, int srcEmitter, int drainCollector, int* ADC1drop, int subtype){

        int ADC1read, ADC1readSum, ADC1cnt, spiInCheck;
        int ADC2read, ADC2readSum, ADC2cnt;
        int diffCnt; int diffHoldSum; int diff;

        // write out to DACs
        spiOut[0] = (volts[0] | DAC0_WRITE) >> 8;   // Term 1
        spiOut[1] = (volts[0] | DAC0_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = (volts[1] | DAC1_WRITE) >> 8;   // Term 2
        spiOut[1] = (volts[1] | DAC1_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = (volts[2] | DAC2_WRITE) >> 8;   // Term 3
        spiOut[1] = (volts[2] | DAC2_WRITE) & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        spiOut[0] = ADCSEQUENCE >> 8;
        spiOut[1] = ADCSEQUENCE & 0x00FF;
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        makeWord(spiOut, 0b0000000000000000); //no op command, data in
        bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);

        ADC1readSum = 0; ADC1cnt = 0;
        ADC2readSum = 0; ADC2cnt = 0;
        diffHoldSum = 0; diffCnt = 0;

        for(int ii = 0; ii < 199; ii++){
            makeWord(spiOut, 0b0000000000000000); //no op command, data in
            bcm2835_spi_transfernb(spiOut, spiIn, WORD_SIZE);
            spiInCheck = (spiIn[0] >> 4) & 0x07;

            // take readings from source and drain of device
            if(spiInCheck == srcEmitter + 4){
                spiIn[0] = spiIn[0] & 0x0F;
                ADC1read = spiIn[0]; // Place result into read-out array
                ADC1read = (ADC1read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                ADC1readSum += ADC1read;
                ADC1cnt++;
            }

            if(spiInCheck == drainCollector + 4){
                spiIn[0] = spiIn[0] & 0x0F;
                ADC2read = spiIn[0]; // Place result into read-out array
                ADC2read = (ADC2read << 8) | spiIn[1];    // Change spiIn 0 to most significant bits, OR with spiIn 1 to create word
                ADC2readSum += ADC2read;
                ADC2cnt++;
            }
            //diff = ADC2read - ADC1read;
            //diffHoldSum += diff;
            //diffCnt++;

        }
            // output collector/drain current readings
            ADC1read = ADC1readSum / ADC1cnt;
            ADC2read = ADC2readSum / ADC2cnt;
            *ADC1drop = volts[drainCollector] - ADC2read;

            if (subtype == NPN || subtype == NMOS){
                voltsVDS[xAxisCnt] = (ADC2read - ADC1read); // was (ADC2read - ADC1read)
            }
            else if (subtype == PNP || subtype == PMOS){
                voltsVDS[xAxisCnt] = (ADC1read - ADC2read);
            }
            voltsVDS[xAxisCnt] =  (voltsVDS[xAxisCnt] / ADCMAX) * VMAX;
}

// adcdac_return function grabs the current values from the device for the curve trace and outputs it to the curve trace file function
int adcdac_return(int vds, float vgs, int t1, int t2, int t3, int subtype){

    int ADC1drop, ADC2drop;

    if( (subtype == NMOS) || (subtype == NPN) ){    // NMOS/NPN first

        // run through each possible iteration and output required curve trace test voltages to device under test
        if( (t1 == GATE) || (t1 == BASE) ){
            if( (t2 == SOURCE) || (t2 == EMITTER) ){
                volts[0] = (vgs*ADCMAX)/VMAX; volts[1] = GROUNDED; volts[2] = vds; // set proper output voltages
                adcdac_returnExt(0, 1, 2, &ADC1drop, subtype); // send them to the extension function
                return ADC1drop; // return the drop
            }
            else {
                volts[0] = (vgs*ADCMAX)/VMAX; volts[2] = GROUNDED; volts[1] = vds;
                adcdac_returnExt(0, 2, 1, &ADC1drop, subtype);
                return ADC1drop;
            }
        }

        else if( (t2 == GATE) || (t2 == BASE) ){
            if( (t1 == SOURCE) || (t1 == EMITTER) ){
                volts[0] = GROUNDED; volts[1] = (vgs*ADCMAX)/VMAX; volts[2] = vds;
                adcdac_returnExt(1, 0, 2, &ADC1drop, subtype);
                return ADC1drop;
            }
            else{
                volts[2] = GROUNDED; volts[1] = (vgs*ADCMAX)/VMAX; volts[0] = vds;
                adcdac_returnExt(1, 2, 0, &ADC1drop, subtype);
                return ADC1drop;
            }
        }

        else if( (t1 == SOURCE) || (t1 == EMITTER) ){
            volts[0] = GROUNDED; volts[1] = vds; volts[2] = (vgs*ADCMAX)/VMAX;
            adcdac_returnExt(2, 0, 1, &ADC1drop, subtype);
            return ADC1drop;
        }
        else{
                volts[1] = GROUNDED; volts[0] = vds; volts[2] = (vgs*ADCMAX)/VMAX;
                adcdac_returnExt(2, 1, 0, &ADC1drop, subtype);
                return ADC1drop;
        }
    }

    if( (subtype == PMOS) || (subtype == PNP) ){ // PMOS/PNP next

        if( (t1 == GATE) || (t1 == BASE) ){
            if( (t2 == DRAIN) || (t2 == COLLECTOR) ){
                volts[0] = (vgs*ADCMAX)/VMAX; volts[2] = FIVE_VOLTS; volts[1] = vds;
                adcdac_returnExt(0, 2, 1, &ADC1drop, subtype);
                return ADC1drop;
            }
            else {
                volts[0] = (vgs*ADCMAX)/VMAX; volts[1] = FIVE_VOLTS; volts[2] = vds;
                adcdac_returnExt(0, 1, 2, &ADC1drop, subtype);
                return ADC1drop;
            }
        }

        else if( (t2 == GATE) || (t2 == BASE) ){
            if( (t1 == DRAIN) || (t1 == COLLECTOR) ){
                volts[2] = FIVE_VOLTS; volts[1] = (vgs*ADCMAX)/VMAX; volts[0] = vds;
                adcdac_returnExt(1, 2, 0, &ADC1drop, subtype);
                return ADC1drop;
            }
            else{
                volts[2] = vds; volts[1] = (vgs*ADCMAX)/VMAX; volts[0] = FIVE_VOLTS;
                adcdac_returnExt(1, 0, 2, &ADC1drop, subtype);
                return ADC1drop;
            }
        }

        else if ( (t3 == GATE) || (t3 == BASE)){
        if( (t1 == DRAIN) || (t1 == COLLECTOR) ){
            volts[1] = FIVE_VOLTS; volts[0] = vds; volts[2] = (vgs*ADCMAX)/VMAX;
            adcdac_returnExt(2, 1, 0, &ADC1drop, subtype);
            return ADC1drop;
        }
        else{
            volts[0] = FIVE_VOLTS; volts[1] = vds; volts[2] = (vgs*ADCMAX)/VMAX;
            adcdac_returnExt(2, 0, 1, &ADC1drop, subtype);
            return ADC1drop;
        }
        }
    }
}

// evaluates the current range of the device
void current_ranger(int type, int subtype,int t1,int t2, int t3){
	int i,j,k,dac,iter;
	double result; double avr;
	for(k=0;k<=5;k++){
        xAxisCnt = 0; // counter used for voltage range
        for(i=0;i<=SAMPLES-1;i++){

            avr=0.0;
            //for(j=1;j<=30;j++){
                if (type == BJT){
                    if (subtype == NPN){
                        vgsCorrected = NbjtBase[k]; // set the proper range for the NPN
                    }
                    else if (subtype == PNP){
                        vgsCorrected = PbjtBase[k]; // set the proper range for the PNP
                    }
                }
                else if (type == MOSFET) {
                    if(subtype == NMOS){
                        vgsCorrected = k;
                    }
                    if(subtype == PMOS){    // Runs Vgs in reverse for plot
                        vgsCorrected = 5-k;
                    }
                }

                dac = adcdac_return(volts_adc[i], vgsCorrected, t1, t2, t3, subtype);
                result = (double)(dac) / (float)(ADCMAX) * (float)(VMAX);

                //avr = avr + result;
            //}
            //avr = avr / 30.0;   // average of 30 readings
            xAxisCnt++;
                curr[i] = result / float(RESISTOR); // result was avr

                if (subtype == PNP || subtype == PMOS){ // if the device is a PMOS or PNP, change the current to negative to flip the axis
                    curr[i] = -curr[i];
                }

    // eliminate first 10 points of data (ADC noise)
    if (subtype == PNP || subtype == PMOS){
        for (int kk = 0; kk < 10; kk++){
            curr[kk] = curr[10];
        }
    }
    }

    print_csv(vgsCorrected,type,subtype,t1,t2,t3); // k or vgsCorrected
    }
    char usb_copy[1000];
    sprintf(usb_copy, "echo \"raspberry\" | sudo -S cp %s /media/pi/usbdrive/%s", fname, fname);
    system(usb_copy);
    // system("echo \"raspberry\" | sudo -S cp curve.csv /media/pi/usbdrive/curve.csv");
	system("echo \"raspberry\" | sudo -S umount /dev/sda1");
	system("echo \"raspberry\" | sudo -S rm -r /media/pi/usbdrive");
}

// main functions
int main(){
	int type=0, subtype=0, fd, fcount=1;;

	char str[][15] = {"TBD","GATE","SOURCE","DRAIN","NMOS","PMOS","MOSFET","BJT","NPN","PNP","BASE","COLLECTOR","EMITTER"};

	// establish GPIO and I2C protocols
	wiringPiSetup();
	fd = wiringPiI2CSetup(0x20);

    // establish SPI protocols
	SPI_init();

	while(1){

        printf("Press test button when ready...\n\n");

        bcm2835_gpio_fsel(TEST_PIN, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(TEST_PIN, BCM2835_GPIO_PUD_UP);

        while(buttonRead == 1){ // test button
            buttonRead = bcm2835_gpio_lev(TEST_PIN);
        }

        buttonRead = 1;

        AD5592_reset();
        AD5592_config();
        fcount=1;

        // the meat and potatoes
        calVolts = AD5592_calibration();
        switch(volt_cycle(mosfet[0], mosfet[1], mosfet[2])){	//This will determine terminal identity, type, and subtype.
            case 1:
                terminal_id[0] = GATE;
                type = MOSFET;
                subtype = type_finder(1, 0);
                drain_source(1,2,0,subtype);
                break;
            case 2:
                terminal_id[1] = GATE;
                type = MOSFET;
                subtype = type_finder(0, 1);
                drain_source(0,2,1,subtype);
                break;
            case 3:
                terminal_id[2] = GATE;
                type = MOSFET;
                subtype = type_finder(0, 2);
                drain_source(1,0,2,subtype);
                break;
            default:
                type = BJT;
                subtype = bjt_typer(/*mosfet[0], mosfet[1], mosfet[2]*/);
                if(terminal_id[0]==BASE){
                    bjt_terminal_id(subtype, 0);
                } else if (terminal_id[1]==BASE){
                    bjt_terminal_id(subtype, 1);
                } else if (terminal_id[2]==BASE){
                    bjt_terminal_id(subtype, 2);
                }
                break;
        } // error check
        if((type == TBD)||(subtype == TBD)||(terminal_id[0] == TBD)||(terminal_id[1] == TBD)||(terminal_id[2] == TBD)){
            printf("Identification Error.  Check device and try again.\n");
        }
        else{
        display_id(terminal_id[0], terminal_id[1], terminal_id[2], type, subtype);
        if(fd==-1){
            printf("Can't setup the 7segment display.\n");
            return -1;
        } else {
                Sev_seg_disp(type, subtype, terminal_id[0], terminal_id[1], terminal_id[2], fd);    //Type, Subtype, Terminals 1, 2, 3
        }
        printf("\nGenerating Curves...\n\n");
        // system("echo \"raspberry\" | sudo -S umount /dev/sda1");
        system("echo \"raspberry\" | sudo -S mkdir /media/pi/usbdrive/ 2> /dev/null");
        system("echo \"raspberry\" | sudo -S mount --source /dev/sda1 --target /media/pi/usbdrive/");
        sprintf(fname, "%s_%s_%d.csv", str[type], str[subtype], fcount);
        FILE *csvfile;
        while (access(fname, F_OK) != -1){
            fcount++;
            sprintf(fname, "%s_%s_%d.csv", str[type], str[subtype],fcount);
        }

        // curve_switch(terminal_id[0], terminal_id[1], terminal_id[2]);

        voltage_ranger();
        current_ranger(type, subtype,terminal_id[0], terminal_id[1], terminal_id[2]);
        char python_run[1000];
        sprintf(python_run, "python /home/pi/TransistorID/curve.py %s", fname);
        system(python_run);
        //system("python /home/pi/TransistorID/curve.py");}
    }
    }
	return 0;
}
