		.sect "vectors"
        .global _VECSTART
        .global _i2s2_tx_isr
        ;.global _i2s2_rx_isr
        .global _Reset
        .ref _c_int00
      .def nmi, int0, int1, int2, int3, int4, int5, int6
      .def int7, int8, int9, int10, int11, int12, int13
      .def int14, int15, int16, int17, int18, int19, int20
      .def int21, int22, int23, int24, int25, int26, int27
      .def int28, int29

_VECSTART:
_Reset  .ivec _c_int00, USE_RETA
nmi     .ivec no_isr
int0    .ivec no_isr
int1    .ivec no_isr
int2    .ivec no_isr
int3    .ivec no_isr
int4    .ivec no_isr
int5    .ivec no_isr
int6    .ivec no_isr
int7    .ivec no_isr
int8    .ivec no_isr
int9    .ivec no_isr
int10   .ivec no_isr
int11   .ivec no_isr
int12   .ivec _i2s2_tx_isr
int13   .ivec no_isr
int14   .ivec no_isr
int15   .ivec no_isr
int16   .ivec no_isr
int17   .ivec no_isr
int18   .ivec no_isr
int19   .ivec no_isr
int20   .ivec no_isr
int21   .ivec no_isr
int22   .ivec no_isr
int23   .ivec no_isr
int24   .ivec no_isr
int25   .ivec no_isr
int26   .ivec no_isr
int27   .ivec no_isr
int28   .ivec no_isr
int29   .ivec no_isr

        .text
        .def no_isr
no_isr:
        b #no_isr

