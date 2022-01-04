#
# This file is part of LiteHyperBus
#
# Copyright (c) 2019 Antti Lukats <antti.lukats@gmail.com>
# Copyright (c) 2019-2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2021 Franck Jullien <franck.jullien@collshade.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.misc import timeline

from litex.build.io import DifferentialOutput

from litex.soc.interconnect import wishbone

# HyperRAM -----------------------------------------------------------------------------------------

class HyperRAM(Module):
    """HyperRAM

    Provides a very simple/minimal HyperRAM core that should work with all FPGA/HyperRam chips:
    - FPGA vendor agnostic.
    - no setup/chip configuration (use default latency).

    This core favors portability and ease of use over performance.
    """
    def __init__(self, pads, latency=6):
        self.pads = pads
        self.bus  = bus = wishbone.Interface()

        # # #

        clk       = Signal()
        clk_phase = Signal(2)
        cs        = Signal()
        ca        = Signal(48)
        ca_active = Signal()
        sr        = Signal(48)
        dq        = self.add_tristate(pads.dq)   if not hasattr(pads.dq,   "oe") else pads.dq
        rwds      = self.add_tristate(pads.rwds) if not hasattr(pads.rwds, "oe") else pads.rwds
        dw        = len(pads.dq)                 if not hasattr(pads.dq,   "oe") else len(pads.dq.o)

        assert dw in [8, 16]

        # Drive rst_n, cs_n, clk from internal signals ---------------------------------------------
        if hasattr(pads, "rst_n"):
            self.comb += pads.rst_n.eq(1)
        self.comb += pads.cs_n[0].eq(~cs)
        assert len(pads.cs_n) <= 2
        if len(pads.cs_n) == 2:
            self.comb += pads.cs_n[1].eq(1)
        if hasattr(pads, "clk"):
            self.comb += pads.clk.eq(clk)
        else:
            self.specials += DifferentialOutput(clk, pads.clk_p, pads.clk_n)

        # Clock Generation (sys_clk/4) -------------------------------------------------------------
        self.sync += clk_phase.eq(clk_phase + 1)
        cases = {}
        cases[1] = clk.eq(cs) # Set pads clk on 90° (if cs is set)
        cases[3] = clk.eq(0)  # Clear pads clk on 270°
        self.sync += Case(clk_phase, cases)

        # Data Shift Register (for write and read) -------------------------------------------------
        dqi = Signal(dw)
        self.sync += dqi.eq(dq.i)  # Sample on 90° and 270°
        self.sync += [
                If((clk_phase == 0) | (clk_phase == 2), # Shift on 0° and 180°
                    # During Command-Address, only D[7:0] are used
                    If(ca_active,
                        sr.eq(Cat(dqi[:8], sr[:-8]))
                    ).Else(
                        sr.eq(Cat(dqi, sr[:-dw]))
                    )
                )
        ]
        self.comb += [
            bus.dat_r.eq(sr),      # To Wisbone
            If(ca_active,
                dq.o.eq(sr[-8:]),  # To HyperRAM, 8-bits mode
            ).Else(
                dq.o.eq(sr[-dw:]), # To HyperRAM, 16-bits mode
            )
        ]

        # Command generation -----------------------------------------------------------------------
        self.comb += [
            ca[47].eq(~self.bus.we),          # R/W#
            ca[45].eq(1),                     # Burst Type (Linear)
        ]

        if dw == 8:
            self.comb += [
                ca[16:45].eq(self.bus.adr[2:]), # Row & Upper Column Address
                ca[1:3].eq(self.bus.adr[0:]),   # Lower Column Address
                ca[0].eq(0),                    # Lower Column Address
            ]
        else:
            self.comb += [
                ca[16:45].eq(self.bus.adr[3:]), # Row & Upper Column Address
                ca[1:3].eq(self.bus.adr[1:]),   # Lower Column Address
                ca[0].eq(self.bus.adr[0]),      # Lower Column Address
            ]

        # Latency count starts from the middle of the command (it's where -4 comes from).
        # In fixed latency mode (default), latency is 2*Latency count.
        # Because we have 4 sys clocks per ram clock:
        lat = (latency * 8) - 4

        # Sequencer --------------------------------------------------------------------------------

        # Command.
        # --------
        dt_seq = [
            # DT,  Action
            (3,    []),
            (12,   [cs.eq(1), dq.oe.eq(1), sr.eq(ca), ca_active.eq(1)]), # Command: 6 clk
            (lat,  [dq.oe.eq(0), ca_active.eq(0)]),                      # Latency
        ]

        # Write/Read.
        # -----------
        rwdso = Signal(2)
        self.comb += rwds.o.eq(rwdso)
        if dw == 8:
            dt_seq += [
                (2,    [dq.oe.eq(self.bus.we),         # Write/Read data byte: 2 clk
                        sr[:16].eq(0),
                        sr[16:].eq(self.bus.dat_w),
                        rwds.oe.eq(self.bus.we),
                        rwdso[0].eq(~bus.sel[3])]),
                (2,    [rwdso[0].eq(~bus.sel[2])]),    # Write/Read data byte: 2 clk
                (2,    [rwdso[0].eq(~bus.sel[1])]),    # Write/Read data byte: 2 clk
                (2,    [rwdso[0].eq(~bus.sel[0])]),    # Write/Read data byte: 2 clk
            ]
        if dw == 16:
            dt_seq += [
                (2,    [dq.oe.eq(self.bus.we),         # Write/Read data byte: 2 clk
                        sr[:16].eq(0),
                        sr[16:].eq(self.bus.dat_w),
                        rwds.oe.eq(self.bus.we),
                        rwdso[1].eq(~bus.sel[3]),
                        rwdso[0].eq(~bus.sel[2])]),
                (2,    [rwdso[1].eq(~bus.sel[1]),
                        rwdso[0].eq(~bus.sel[0])]),    # Write/Read data byte: 2 clk
            ]

        # End.
        # ----
        dt_seq += [
            (2,    [cs.eq(0), rwds.oe.eq(0), dq.oe.eq(0)]),
            (1,    [bus.ack.eq(1)]),
            (1,    [bus.ack.eq(0)]),
            (0,    [])
        ]

        # Convert delta-time sequencer to time sequencer
        t_seq = []
        t_seq_start = (clk_phase == 1)
        t = 0
        for dt, a in dt_seq:
            t_seq.append((t, a))
            t += dt
        self.sync += timeline(bus.cyc & bus.stb & t_seq_start, t_seq)

    def add_tristate(self, pad):
        t = TSTriple(len(pad))
        self.specials += t.get_tristate(pad)
        return t
