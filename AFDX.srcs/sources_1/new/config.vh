// Copyright (c) 2018, Marcelo Samsoniuk
`define __3STAGE__

// RV32I vs RV32E:
//
// The difference between the RV32I and RV32E regarding the logic space is
// minimal in typical applications with modern 5 or 6 input LUT based FPGAs,
// but the RV32E is better with old 4 input LUT based FPGAs.
//`define __RV32E__


`ifdef __COPROCESSOR__


`endif


// CSR support
//
// enable this to use CSR registers...  INTERRUPT and EBREAK use this in
// order to read some special exception registers.  Also, THREADS use this in
// order to identify the core number.
`define __CSR__

`ifdef __CSR__

// Performance Counters
//
// The Performance Counters are a set of 64-bit registers that counts the
//number of clocks and number of instructions executed, so is possible
//measure the core performance.
`define __CSR_ESSENTIAL__


`define __INTERRUPT__

// ebreak support
//
// ebreak enable live debug w/ gdb, with break points, single-step, etc...
// it basically consists in a single instruction that replace the normal
// instruction, so an exception will be triggered, which is like an interrupt,
// but with no real interrupt source.
`define __EBREAK__

`endif

// MUL support
//
// experimental and partial M-extension support, MUL/MULH/MULHSU/MULHU
// only...  in case of DIV/REM, trap and emulate by software is the current
// option!
//`define __MEXT__


`define __PERFMETER__


`define __RESETPC__ 32'd0

////////////////////////////////////////////////////////////////////////////////
// darksocv configuration:
////////////////////////////////////////////////////////////////////////////////


`define __HARVARD__


`ifndef __HARVARD__
    `define __LUTCACHE__
    `define __CDEPTH__ 6
    `define __ICACHE__
    `define __DCACHE__
    `define __RMW_CYCLE__
`endif


`define MLEN 13 // 13: 8kB for darkshell
                // 15: 32kB, for coremark


// bram wait states
//
// to simulate high latency memories, is possible set the number of wait-states
// for bram here! case not configured, wait-states defaults to 1.
//`define __WAITSTATE__ 3

// UART speed is set in bits per second, typically 115200 bps:
//`define __UARTSPEED__ 115200

// UART queue:
//
// Optional RX/TX queue for communication oriented applications. The concept
// foreseen 256 bytes for TX and RX, in a way that frames up to 128 bytes can
// be easily exchanged via UART. the queue size is defined as 2**N:
//`define __UARTQUEUE__ 8 // not working well, need check...

////////////////////////////////////////////////////////////////////////////////
// board definition:
////////////////////////////////////////////////////////////////////////////////

// The board is automatically defined in the xst/xise files via Makefile or
// ISE. Case it is not the case, please define you board name here:
//`define AVNET_MICROBOARD_LX9
//`define XILINX_AC701_A200
//`define QMTECH_SDRAM_LX16

// the following defines are automatically defined:

`ifdef __ICARUS__
    `define SIMULATION 1
`endif

`ifdef XILINX_ISIM
    `define SIMULATION 2
`endif

`ifdef MODEL_TECH
    `define SIMULATION 3
`endif

`ifdef XILINX_SIMULATOR
    `define SIMULATION 4
`endif

// the board definition is done on the tool, otherwise we assume simulation

`ifdef AVNET_MICROBOARD_LX9
    `define BOARD_ID 1
    //`define BOARD_CK 100000000
    //`define BOARD_CK 66666666
    //`define BOARD_CK 40000000
    // example of DCM logic:
    `define BOARD_CK_REF 100000000
    `define BOARD_CK_MUL 6
    `ifdef __3STAGE__
        `define BOARD_CK_DIV 6 // 3-stage, 1-ws, 9=66MHz 6=100MHz
    `else
        `define BOARD_CK_DIV 9 // 2-stage, 1-ws, 9=66MHz 6=100MHz
    `endif
    `define XILINX6CLK 1
`endif

`ifdef XILINX_AC701_A200
    `define BOARD_ID 2
    //`define BOARD_CK 90000000
    `define BOARD_CK_REF 90000000
    `define BOARD_CK_MUL 4
    `define BOARD_CK_DIV 2
`endif

`ifdef QMTECH_SDRAM_LX16
    `define BOARD_ID 3
    `define BOARD_CK_REF 50000000
    `define BOARD_CK_MUL 4
    `define BOARD_CK_DIV 2
    `define INVRES 1
    `define XILINX6CLK 1
`endif

`ifdef QMTECH_SPARTAN7_S15
    `define BOARD_ID 4
    `define BOARD_CK_REF 50000000
    `define BOARD_CK_MUL 20
    `define BOARD_CK_DIV 10
    `define XILINX7CLK 1
    `define VIVADO 1
    `define INVRES 1
`endif

`ifdef LATTICE_BREVIA2_XP2
    `define BOARD_ID 5
    `define BOARD_CK 50000000
    `define INVRES 1
`endif

`ifdef LATTICE_ECP5_COLORLIGHTI9
    `define LATTICE_ECP5_PLL_REF25MHZ 1
    `define BOARD_ID 14
    `define BOARD_CK 125_000_000 // cause we use a pll with 25MHz ref clks
    `define INVRES 1
`endif

`ifdef LATTICE_ECP5_COLORLIGHTI5
    `define LATTICE_ECP5_PLL_REF25MHZ 1
    `define BOARD_ID 15
    `define BOARD_CK 125_000_000 // cause we use a pll with 25MHz ref clks
    `define INVRES 1
`endif

`ifdef LATTICE_ECP5_ULX3S
    `define LATTICE_ECP5_PLL_REF25MHZ 1
    `define BOARD_ID 16
    `define BOARD_CK 125_000_000 // cause we use a pll with 25MHz ref clks
    `define INVRES 1
`endif

`ifdef LATTICE_ICE40_BREAKOUT_HX8K
    `define BOARD_ID 17
    `define BOARD_CK 65_000_000 // cause we use a pll with 25MHz ref clks
    `define INVRES 1
`endif


`ifdef PISWORDS_RS485_LX9
    `define BOARD_ID 6
    `define BOARD_CK_REF 50000000
    `define BOARD_CK_MUL 4
    `define BOARD_CK_DIV 2
    `define INVRES 1
    `define XILINX6CLK 1
`endif

`ifdef DIGILENT_SPARTAN3_S200
    `define BOARD_ID 7
    `define BOARD_CK 50000000
    `define __RMW_CYCLE__
`endif

`ifdef ALIEXPRESS_HPC40GBE_K420
    `define BOARD_ID 8
    //`define BOARD_CK 200000000
    `define BOARD_CK_REF 100000000
    `define BOARD_CK_MUL 12
    `define BOARD_CK_DIV 5
    `define XILINX7CLK 1
    `define INVRES 1
`endif

`ifdef QMTECH_ARTIX7_A35
    `define BOARD_ID 9
    `define BOARD_CK_REF 50000000
    `define BOARD_CK_MUL 20
    `define BOARD_CK_DIV 10
    `define XILINX7CLK 1
    `define VIVADO 1
    `define INVRES 1
`endif

`ifdef ALIEXPRESS_HPC40GBE_XKCU040
    `define BOARD_ID 10
    //`define BOARD_CK 200000000
    `define BOARD_CK_REF 100000000
    `define BOARD_CK_MUL 8  // x8/2 = 400MHZ (overclock!)
    `define BOARD_CK_DIV 2  // vivado reco. = 250MHz
    `define XILINX7CLK 1
    `define INVRES 1
`endif

`ifdef PAPILIO_DUO_LOGICSTART
    `define BOARD_ID 11
    `define BOARD_CK_REF 32000000
    `define BOARD_CK_MUL 2
    `define BOARD_CK_DIV 2
    `define XILINX6CLK 1
`endif

`ifdef QMTECH_KINTEX7_K325
    `define BOARD_ID 12
    `define BOARD_CK_REF 50000000
    `define BOARD_CK_MUL 20
    `define BOARD_CK_DIV 4
    `define XILINX7CLK 1
    `define INVRES 1
`endif

`ifdef SCARAB_MINISPARTAN6_PLUS_LX9
    `define BOARD_ID 13
    `define BOARD_CK_REF 50000000
    `define BOARD_CK_MUL 4
    `define BOARD_CK_DIV 2
    // `define INVRES 0
    `define XILINX6CLK 1
`endif

`ifdef QMTECH_CYCLONE10_CL016
    `define BOARD_ID 17
    `define BOARD_CK 50000000
	 `define INVRES 1
	 `define MIFBRAM 1
    `define __RMW_CYCLE__
`endif

`ifdef PISSWORDS_CH34X_LX16
    `define BOARD_ID 0 // 18
    `ifdef __3STAGE__
        `define BOARD_CK_REF 50000000
        `define BOARD_CK_MUL 2
        `define BOARD_CK_DIV 2
        `define XILINX6CLK 1
    `else
        `define BOARD_CK 50000000
    `endif
    `define INVRES 1

`endif

`ifdef MAX1000_MAX10
    `define BOARD_ID 19
    `define BOARD_CK 32000000
`endif

`ifdef DE10NANO_CYCLONEV_MISTER
    `define BOARD_ID 20
    `define BOARD_CK 50000000
`endif

`ifndef BOARD_ID
    `define BOARD_ID 0
    `define BOARD_CK 100000000
    //`define __SDRAM__ 1
`endif

`ifdef BOARD_CK_REF
    `define BOARD_CK (`BOARD_CK_REF * `BOARD_CK_MUL / `BOARD_CK_DIV)
`endif

// darkuart baudrate automtically calculated according to board clock:

`ifndef __UARTSPEED__
  `define __UARTSPEED__ 115200
`endif

`define  __BAUD__ ((`BOARD_CK/`__UARTSPEED__))

// register number depends of CPU type RV32[EI] and number of threads

`ifdef __THREADS__

    `ifdef __RV32E__
        `define RLEN 16*(2**`__THREADS__)
    `else
        `define RLEN 32*(2**`__THREADS__)
    `endif

    `define __CSR__
`else
    `ifdef __RV32E__
        `define RLEN 16
    `else
        `define RLEN 32
    `endif
`endif

`ifdef __INTERRUPT__
    `define __CSR__
`endif

`ifdef __EBREAK__
    `define __CSR__
`endif
