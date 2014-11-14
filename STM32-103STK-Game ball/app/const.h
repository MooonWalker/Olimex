

//CONSTANTS
#define   NOP             0xff
#define   R_REGISTER      0x00
#define   W_REGISTER      0x01
#define   R_RX_PAYLOAD    0x61
#define   W_TX_PAYLOAD    0xA0
#define   FLUSH_TX        0xE1
#define   FLUSH_RX        0xE2
#define   REUSE_TX_PL     0xE3

#define   RX_ADDR_P0      0x0A
#define   RX_ADDR_P1      0x0B
#define   TX_ADDR         0x10
#define   RX_PW_P0        0x11
#define   RX_PW_P1        0x12
#define   FIFO_STATUS     0x17

#define   MAX_RT          0x10

#define   CONFIG_REG_ADDR 0x00
#define   STATUS_ADDR     0x07

#define   FLUSH_TX        0xE1
#define   TX_FULL         0x01
#define   RX_DR           0x40
#define   TX_DS           0x20

// #define   IRQ             IO0PIN&BIT3

// #define   BUT1            IO0PIN&BIT15
// #define   BUT2            IO0PIN&BIT16


#define   RX_TX_TIME      100000
#define   BUT_TIME        10000

/*
#define   LED1_ON         IO0CLR |= BIT10
#define   LED1_OFF        IO0SET |= BIT10
#define   LED2_ON         IO0CLR |= BIT11
#define   LED2_OFF        IO0SET |= BIT11

#define   BUZZ_TIME       500
#define   BUZZ1_HIGH      IO0SET_bit.P0_12 = 1;
#define   BUZZ1_LOW       IO0CLR_bit.P0_12 = 1;
#define   BUZZ2_HIGH      IO0SET_bit.P0_13 = 1;
#define   BUZZ2_LOW       IO0CLR_bit.P0_13 = 1;
*/



