#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2018 hcab14@gmail.com.
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

import numpy as np
from gnuradio import blocks
from gnuradio import gr
from gnuradio import filter
import pmt
from align_streams import align_streams

class rotator_array(gr.basic_block):
    """
    This is a pure message handling gr.basic_block containing an array of blocks.rotator_cc
    The phase increment of these blocks is set in the message handler
    """
    def __init__(self, num_streams, fs, df):
        gr.basic_block.__init__(
            self,
            name="rotator_array",
            in_sig=None,
            out_sig=None)
        self._num_streams = num_streams
        self._fs = fs
        self._df = df
        self._rotators = [blocks.rotator_cc(0) for _ in range(num_streams)]
        self._port_fs  = pmt.intern('fs')
        self.message_port_register_in(self._port_fs)
        self.set_msg_handler(self._port_fs, self.msg_handler_rotator)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def get_rotator_block(self, i):
        return self._rotators[i]

    def msg_handler_rotator(self, msg_in):
        fs = pmt.to_python(pmt.cdar(msg_in))
        n  = self._num_streams
        phase_increments = 2*np.pi * self._df/self._fs * (self._fs/fs-1.0) * np.arange(-(n//2), (n//2)+1)
        print('phase_increments=', phase_increments)
        for (i,d) in enumerate(phase_increments):
            self._rotators[i].set_phase_inc(d)

def gcd(a, b):
    return gcd(b, a % b) if b else a

class coh_stream_synth(gr.hier_block2):
    """
    Block for coherently combining KiwiSDR IQ streams into a single stream with larger bandwidth
    """
    def __init__(self, num_streams, fs_in, delta_f_in):
        gr.hier_block2.__init__(self,
                                "coh_stream_synth",
                                gr.io_signature(num_streams,   num_streams,   gr.sizeof_gr_complex),
                                gr.io_signature(num_streams+1, num_streams+1, gr.sizeof_gr_complex))
        fsr    = delta_f_in
        factor = gcd(fsr, fs_in)
        interp = fsr/factor
        decim  = fs_in/factor

        self._align_streams = align_streams(num_streams)
        self._rotators      = rotator_array(num_streams, fs_in, delta_f_in)

        self._taps_resampler = taps_resampler = filter.firdes.low_pass(1, 1, 0.50/decim, 0.01/decim)
        self._rational_resamplers = [filter.rational_resampler_ccc(interpolation=interp,
                                                                   decimation=decim,
                                                                   ## taps=None,
                                                                   ## fractional_bw=0.48) for _ in range(num_streams)]
                                                                   taps=(taps_resampler),
                                                                   fractional_bw=None) for _ in range(num_streams)]
        self._taps_pfb = taps_pfb = filter.firdes.low_pass_2(1+num_streams, (1+num_streams)*fsr,
                                                             0.50*fsr, 0.01*fsr, 80, 5)
        self._pfb_synthesizer = filter.pfb_synthesizer_ccf(1+num_streams, (taps_pfb), False)
        channel_map = [num_streams]
        channel_map.extend(range(num_streams))
        self._pfb_synthesizer.set_channel_map(channel_map)
        for i in range(num_streams):
            self.connect((self, i),
                         (self._align_streams, i),
                         (self._rotators.get_rotator_block(i), 0),
                         (self._rational_resamplers[i], 0),
                         (self._pfb_synthesizer, i))
            self.connect((self._rational_resamplers[i], 0),
                         (self, 1+i))
            # self.connect((self._rotators.get_rotator_block(i), 0),
            #              (self, 1+i))
        self.connect((self._pfb_synthesizer, 0), (self, 0))

        self.msg_connect((self._align_streams.get_find_offsets(), "fs"), (self._rotators, "fs"))

