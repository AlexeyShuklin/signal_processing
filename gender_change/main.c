#define AIC3204_I2C_ADDR 0x18
#include "C5515.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "stdio.h"

#define W_LEN 1024
#define TEMP_BUF_LEN 512
#define NOSCALE 0
#define EFFECT_FLAG 1
#define NO_EFFECT_FLAG 0
#define IER0 (*(unsigned int *)0x00)
#define N 128 //FFT length
#define IVPD (*(unsigned int*)0x000049)
#define IVPH (*(unsigned int*)0x00004A)

#define Xmit 0x20
#define Rcv 0x08


/**********Функции для вычисления Фурье************/
#define rfft(x,nx)\
    (\
         cfft_SCALE(x,nx/2),\
         cbrev(x,x,nx/2),\
         unpack(x,nx)\
    )
#define rifft(x,nx)\
    (\
        unpacki(x, nx),\
        cifft_NOSCALE(x,nx/2),\
        cbrev(x,x,nx/2)\
    )
/***************************************************/

extern unsigned int VECSTART;


Int16 *x, *y; 			// input signal and FFT buffer pointers
//
//
//
Int16 b1[W_LEN] = {0};
Int16 b2[W_LEN] = {0};
Int16 b3[W_LEN] = {0};
Int16 b4[W_LEN] = {0};

Int16 b5[W_LEN] = {0};
Int16 b6[W_LEN] = {0};
Int16 b7[W_LEN] = {0};
Int16 b8[W_LEN] = {0};

Int16* buf_in_left_1 = b1;
Int16* buf_in_right_1 = b2;
Int16* buf_out_left_1 = b3;
Int16* buf_out_right_1 = b4;

Int16* buf_in_left_2 = b5;
Int16* buf_in_right_2 = b6;
Int16* buf_out_left_2 = b7;
Int16* buf_out_right_2 = b8;

Int16* change;

Int16 buf_index = 0;



Int16 effect_flag = NO_EFFECT_FLAG;


/*************Функции для вычисления Фурье**********/
extern void unpack32(Int32 *x, Int16 nx);
extern void cfft32_SCALE(Int32 *x, Int16 nx);
extern void cbrev32(Int32 *x, Int32 *y, Int16 nx);
extern void cifft32_SCALE (Int32 *x,  Int16 nx);
extern void unpacki32(Int16 *x, Int16 nx);
extern Int16 sqrt_16(Int16 *x, Int16 *r, Int16 nx);
/***************************************************/

extern Int16 aic3204_mic();
extern Int16 aic3204_stereo_in1();




/* Обработчик прерывания по предаче */
interrupt void i2s2_tx_isr() // interrupt service routine
{
	I2S2_W0_MSW_W = buf_out_left_2[buf_index];
	I2S2_W1_MSW_W = buf_out_right_2[buf_index];

	while ((Rcv & I2S2_IR) == 0);
	buf_in_left_2[buf_index] = I2S2_W0_MSW_R;
	buf_in_right_2[buf_index] = I2S2_W1_MSW_R;

	buf_index++;

	if (buf_index >= W_LEN) {
		change = buf_in_left_2;
		buf_in_left_2 = buf_in_left_1;
		buf_in_left_1 = change;

		change = buf_out_left_2;
		buf_out_left_2 = buf_out_left_1;
		buf_out_left_1 = change;

		change = buf_in_right_2;
		buf_in_right_2 = buf_in_right_1;
		buf_in_right_1 = change;

		change = buf_out_right_2;
		buf_out_right_2 = buf_out_right_1;
		buf_out_right_1 = change;

		buf_index = 0;
		effect_flag = EFFECT_FLAG;
	}
}

static inline void effect(Int16 *buf_in, Int16 *buf_out) {
	Int16 i, j;
	Int16 temp_buf_1[TEMP_BUF_LEN];
	Int16 temp_buf_2[TEMP_BUF_LEN];
	rfft(buf_in, W_LEN);
	j = 0;
	for (i = 0; i < TEMP_BUF_LEN; i++) {
		Int32 current_el_1 = (Int32)buf_in[j];
		j++;
		Int32 current_el_2 = (Int32)buf_in[j];
		j++;
		temp_buf_1[i] = (Int16)((current_el_1 * current_el_1 + current_el_2 * current_el_2) >> 15);
	}
	sqrt_16(temp_buf_1, temp_buf_2, TEMP_BUF_LEN);

	for (i = 0, j = 0; i < W_LEN; i += 2, j++) {
		buf_out[i] = temp_buf_1[j];
		buf_out[i + 1] = 0;
	}
	rifft(buf_out, W_LEN);
}


void main(void) {
	/* Initialize BSL */
	c5515_init();
	/* Configure Parallel Port */
	SYS_EXBUSSEL &= ~0x7000;   					//
	SYS_EXBUSSEL |= 0x1000;						// Configure Parallel Port for I2S2
	/* Configure Serial Port */
	SYS_EXBUSSEL &= ~0x0C00;   					//
	SYS_EXBUSSEL |= 0x0400;						// Serial Port mode 1 (I2S1 and GP[11:10]).
	c5515_GPIO_init();
	c5515_GPIO_setDirection(GPIO10, GPIO_OUT);
	c5515_GPIO_setOutput(GPIO10, 1);    		// Take AIC3201 chip out of reset
	I2C_init();                    				// Initialize I2C

	asm(" BSET INTM");
	unsigned long int vector;
	vector = (unsigned long int)&VECSTART;
	vector = vector >> 8;
	IVPD = (unsigned short)vector;
	IVPH = (unsigned short)vector;
	IER0 |= 0x4000;
	asm(" BCLR INTM");

	/* I2S settings */
	I2S2_SRGR = 0x0015;
	I2S2_ICMR = 0x0028;    						// Enable interrupts
	I2S2_CR = 0x8012;							// 16-bit word, Master, enable I2C
//	I2S2_INTFL = 0x003C;
//	I2S2_INTMASK = 0x0024;


	aic3204_stereo_in1();

	while(1) {

		if (effect_flag == EFFECT_FLAG) {
			effect(buf_in_left_1, buf_out_left_1);
			effect(buf_in_right_1, buf_out_right_1);
			effect_flag = NO_EFFECT_FLAG;
		}

	}

/* Ioee??aai I2S */
I2S0_CR = 0x00;

c5515_GPIO_setOutput(GPIO26, 0);

SW_BREAKPOINT;
}
