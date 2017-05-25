#ifndef I2S_H_
#define I2S_H_

/* ------------------------------------------------------------------------ *
 *  I2S Module                                                              *
 * ------------------------------------------------------------------------ */
#define I2S0_CR            *(volatile ioport Uint16*)(0x2800)
#define I2S0_SRGR          *(volatile ioport Uint16*)(0x2804)
#define I2S0_W0_LSW_W      *(volatile ioport Uint16*)(0x2808)
#define I2S0_W0_MSW_W      *(volatile ioport Uint16*)(0x2809)
#define I2S0_W1_LSW_W      *(volatile ioport Uint16*)(0x280C)
#define I2S0_W1_MSW_W      *(volatile ioport Uint16*)(0x280D)
#define I2S0_IR            *(volatile ioport Uint16*)(0x2810)
#define I2S0_ICMR          *(volatile ioport Uint16*)(0x2814)
#define I2S0_INTFL         *(volatile ioport Uint16*)(0x2818)
#define I2S0_INTMASK       *(volatile ioport Uint16*)(0x281C)
#define I2S0_W0_LSW_R      *(volatile ioport Uint16*)(0x2828)
#define I2S0_W0_MSW_R      *(volatile ioport Uint16*)(0x2829)
#define I2S0_W1_LSW_R      *(volatile ioport Uint16*)(0x282C)
#define I2S0_W1_MSW_R      *(volatile ioport Uint16*)(0x282D)
/* I2S2 */
#define I2S2_CR            *(volatile ioport Uint16*)(0x2A00)
#define I2S2_SRGR          *(volatile ioport Uint16*)(0x2A04)
#define I2S2_W0_LSW_W      *(volatile ioport Uint16*)(0x2A08)
#define I2S2_W0_MSW_W      *(volatile ioport Uint16*)(0x2A09)
#define I2S2_W1_LSW_W      *(volatile ioport Uint16*)(0x2A0C)
#define I2S2_W1_MSW_W      *(volatile ioport Uint16*)(0x2A0D)
#define I2S2_IR            *(volatile ioport Uint16*)(0x2A10)
#define I2S2_ICMR          *(volatile ioport Uint16*)(0x2A14)
#define I2S2_INTFL         *(volatile ioport Uint16*)(0x2A18)
#define I2S2_INTMASK       *(volatile ioport Uint16*)(0x2A1C)
#define I2S2_W0_LSW_R      *(volatile ioport Uint16*)(0x2A28)
#define I2S2_W0_MSW_R      *(volatile ioport Uint16*)(0x2A29)
#define I2S2_W1_LSW_R      *(volatile ioport Uint16*)(0x2A2C)
#define I2S2_W1_MSW_R      *(volatile ioport Uint16*)(0x2A2D)



#endif /* I2S_H_ */
