-stack 0x2000
-sysstack 0x1000
-heap 0x2000
-c
-u _Reset

MEMORY
{
PAGE 0:
MMR (RWIX): origin = 0x000000, length = 0x0000c0
DARAM0 (RWIX): origin = 0x0000c0, length = 0x00ff40
SARAM0 (RWIX): origin = 0x010000, length = 0x010000
SARAM1 (RWIX): origin = 0x020000, length = 0x020000
SARAM2 (RWIX): origin = 0x040000, length = 0x00FE00
VECS (RWIX): origin = 0x04FE00, length = 0x000200
PDROM (RIX): origin = 0xff8000, length = 0x008000
PAGE 2:
IOPORT (RWI) : origin = 0x000000, length = 0x020000
}

SECTIONS
{
.text >> SARAM1|SARAM2|SARAM0
.stack > DARAM0
.sysstack > DARAM0
.data >> DARAM0|SARAM0|SARAM1
.bss >> DARAM0|SARAM0|SARAM1
.const >> DARAM0|SARAM0|SARAM1
.sysmem > DARAM0|SARAM0|SARAM1
.switch > SARAM2
.cinit > SARAM2
.pinit > SARAM2
.cio > SARAM2
.args > SARAM2
vectors > VECS
.ioport > IOPORT PAGE 2
}