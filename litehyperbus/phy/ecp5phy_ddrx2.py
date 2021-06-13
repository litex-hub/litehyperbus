#
# This file is part of LiteHyperBus
#
# Copyright (c) 2020 Gregory Davill <greg.davill@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from migen.genlib.cdc import MultiReg

# HyperBusPHY --------------------------------------------------------------------------------------

class HyperBusPHY(Module):
    """HyperBusPHY x2 for ECP5

    Provides I/O support for a 32bit datapath from HyperRAM x2 module
    - Uses ECP5 primitives IDDRX2F / ODDRX2F / DELAYF
      - Not technically supported under diamond outside of DQS modes
      - Only available on Left/Right I/O banks

    - PLL required to produce 2*sys_clk, then use CLKDIVF to create sys_clk

    - Clocks
     - hr2x    : 2* sys_freq - I/O clock
     - hr      : sys_freq    - core clock
     - hr2x_90 : 2* sys_freq - phase shifted clock output to HyperRAM
     - hr_90   : sys_freq    - phase shifted clock for SCLK

    """
    def add_tristate(self, pad):
        t = TSTriple(len(pad))
        self.specials += t.get_tristate(pad)
        return t

    def __init__(self, pads):
        def io_bus(n):
            return Record([("oe", 1),("i", n),("o", n)])

        # # #

        self.clk_enable = Signal()
        self.cs         = Signal()
        self.dq         = io_bus(32)
        self.rwds       = io_bus(4)

        # IO Delay shifting
        self.dly_io  = Record([("loadn", 1),("move", 1),("direction", 1)])
        self.dly_clk = Record([("loadn", 1),("move", 1),("direction", 1)])

        dq   = self.add_tristate(pads.dq)   if not hasattr(pads.dq, "oe")   else pads.dq
        rwds = self.add_tristate(pads.rwds) if not hasattr(pads.rwds, "oe") else pads.rwds

        # Shift non DDR signals to match the FF's inside DDR modules.
        self.specials += MultiReg(self.cs,      pads.cs_n, n=3)
        self.specials += MultiReg(self.rwds.oe, rwds.oe,   n=3)
        self.specials += MultiReg(self.dq.oe,   dq.oe,     n=3)

        # Mask off clock when no CS
        clk_en = Signal()
        self.comb += clk_en.eq(self.clk_enable & ~self.cs)

        # Clk_out
        if hasattr(pads, "clk"):
            clk = Signal()
            self.specials += [
                Instance("ODDRX2F",
                    i_SCLK = ClockSignal("hr_90"),
                    i_ECLK = ClockSignal("hr2x_90"),
                    i_RST  = ResetSignal("hr"),
                    i_D3   = clk_en,
                    i_D2   = 0,
                    i_D1   = clk_en,
                    i_D0   = 0,
                    o_Q    = clk
                ),
                Instance("DELAYF",
                    p_DEL_MODE  = "USER_DEFINED",
                    p_DEL_VALUE = 0, # (25ps per tap)
                    i_A         = clk,
                    i_LOADN     = self.dly_clk.loadn,
                    i_MOVE      = self.dly_clk.move,
                    i_DIRECTION = self.dly_clk.direction,
                    o_Z         = pads.clk
                )
            ]

        else:
            clkp = Signal()
            clkn = Signal()
            self.specials += [
                Instance("ODDRX2F",
                    i_SCLK = ClockSignal("hr_90"),
                    i_ECLK = ClockSignal("hr2x_90"),
                    i_RST  = ResetSignal("hr"),
                    i_D3   = clk_en,
                    i_D2   = 0,
                    i_D1   = clk_en,
                    i_D0   = 0,
                    o_Q    = clkp
                ),
                Instance("DELAYF",
                    p_DEL_MODE  = "USER_DEFINED",
                    p_DEL_VALUE = 0, # (25ps per tap)
                    i_A         = clkp,
                    i_LOADN     = self.dly_clk.loadn,
                    i_MOVE      = self.dly_clk.move,
                    i_DIRECTION = self.dly_clk.direction,
                    o_Z         = pads.clk_p
                )
            ]

            self.specials += [
                Instance("ODDRX2F",
                    i_SCLK = ClockSignal("hr_90"),
                    i_ECLK = ClockSignal("hr2x_90"),
                    i_RST  = ResetSignal("hr"),
                    i_D3   = ~clk_en,
                    i_D2   = 1,
                    i_D1   = ~clk_en,
                    i_D0   = 1,
                    o_Q    = clkn
                ),
                Instance("DELAYF",
                    p_DEL_MODE  = "USER_DEFINED",
                    p_DEL_VALUE = 0, # (25ps per tap)
                    i_A         = clkn,
                    i_LOADN     = self.dly_clk.loadn,
                    i_MOVE      = self.dly_clk.move,
                    i_DIRECTION = self.dly_clk.direction,
                    o_Z         = pads.clk_n
                )
            ]

        # DQ_out
        for i in range(8):
            self.specials += Instance("ODDRX2F",
                i_SCLK = ClockSignal("hr"),
                i_ECLK = ClockSignal("hr2x"),
                i_RST  = ResetSignal("hr"),
                i_D3   = self.dq.o[i],
                i_D2   = self.dq.o[8+i],
                i_D1   = self.dq.o[16+i],
                i_D0   = self.dq.o[24+i],
                o_Q    = dq.o[i]
            )


        # DQ_in
        for i in range(8):
            dq_in = Signal()
            self.specials += [
                Instance("IDDRX2F",
                    i_SCLK = ClockSignal("hr"),
                    i_ECLK = ClockSignal("hr2x"),
                    i_RST  = ResetSignal("hr"),
                    i_D    = dq_in,
                    o_Q3   = self.dq.i[i],
                    o_Q2   = self.dq.i[i+8],
                    o_Q1   = self.dq.i[i+16],
                    o_Q0   = self.dq.i[i+24]
                ),
                Instance("DELAYF",
                    p_DEL_MODE  = "USER_DEFINED",
                    p_DEL_VALUE = 0, # (25ps per tap)
                    i_A         = dq.i[i],
                    i_LOADN     = self.dly_io.loadn,
                    i_MOVE      = self.dly_io.move,
                    i_DIRECTION = self.dly_io.direction,
                    o_Z         = dq_in
                )
            ]

        # RWDS_out
        self.specials += [
            Instance("ODDRX2F",
                i_SCLK = ClockSignal("hr"),
                i_ECLK = ClockSignal("hr2x"),
                i_RST  = ResetSignal("hr"),
                i_D3   = self.rwds.o[0],
                i_D2   = self.rwds.o[1],
                i_D1   = self.rwds.o[2],
                i_D0   = self.rwds.o[3],
                o_Q    = rwds.o
            )
        ]

        # RWDS_in
        rwds_in = Signal()
        self.specials += [
            Instance("IDDRX2F",
                i_SCLK = ClockSignal("hr"),
                i_ECLK = ClockSignal("hr2x"),
                i_RST  = ResetSignal("hr"),
                i_D    = rwds_in,
                o_Q3   = self.rwds.i[0],
                o_Q2   = self.rwds.i[1],
                o_Q1   = self.rwds.i[2],
                o_Q0   = self.rwds.i[3]
            ),
            Instance("DELAYF",
                p_DEL_MODE  = "USER_DEFINED",
                p_DEL_VALUE = 0, # (25ps per tap)
                i_A         = rwds.i,
                i_LOADN     = self.dly_io.loadn,
                i_MOVE      = self.dly_io.move,
                i_DIRECTION = self.dly_io.direction,
                o_Z         = rwds_in
            )
        ]
