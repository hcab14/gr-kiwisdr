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

class coh_rotator(gr.sync_block):
    """
    Coherently rotate num_streams of IQ samples
    Phase increments are set via message parsing
    """
    def __init__(self, num_streams, fs, df):
        gr.sync_block.__init__(
            self,
            name="coh_rotator",
            in_sig  = num_streams*(np.complex64,),
            out_sig = num_streams*(np.complex64,))
        self._num_streams = num_streams
        self._fs = fs
        self._df = df
        self._exp_phase     = np.ones(num_streams, dtype=np.complex64)
        self._exp_phase_inc = np.ones(num_streams, dtype=np.complex64)
        self._port_fs = pmt.intern('fs')
        self.message_port_register_in(self._port_fs)
        self.set_msg_handler(self._port_fs, self.msg_handler_rotator)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def work(self, input_items, output_items):
        n = len(input_items[0])
        exp_phases = np.zeros(n, dtype=np.complex64)
        for i in range(self._num_streams):
            exp_phases[:] = self._exp_phase_inc[i]
            exp_phases    = np.cumprod(exp_phases[:])
            exp_phases   *= self._exp_phase[i]
            output_items[i][:]  = input_items[i]
            ## we need a better way for making sure the streams are aligned (coherence)
            if self.nitems_written(i) > 4096:
                output_items[i][:] *= exp_phases
                self._exp_phase[i]  = exp_phases[-1]
        ## normalize
        self._exp_phase /= np.abs(self._exp_phase)
        return n

    def msg_handler_rotator(self, msg_in):
        fs = pmt.to_python(pmt.cdar(msg_in))
        n  = self._num_streams
        m  = (n-1)/2
        phase_inc = 2*np.pi * self._df/float(self._fs) * (self._fs/fs-1.0)
        self._exp_phase_inc = np.exp(1j*phase_inc * np.linspace(-m,m,n),
                                     dtype=np.complex64)

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
        self._rotators      = coh_rotator(num_streams, fs_in, delta_f_in)

        self._taps_resampler = taps_resampler = filter.firdes.low_pass(gain             = interp,
                                                                       sampling_freq    = 1.0,
                                                                       cutoff_freq      = 0.50/decim,
                                                                       transition_width = 0.01/decim,
                                                                       window           = filter.firdes.WIN_HAMMING)
        self._rational_resamplers = [filter.rational_resampler_ccc(interpolation = interp,
                                                                   decimation    = decim,
                                                                   taps          = (taps_resampler),
                                                                   fractional_bw = None) for _ in range(num_streams)]
        self._taps_pfb = taps_pfb = filter.firdes.low_pass_2(gain             = (1+num_streams),
                                                             sampling_freq    = float((1+num_streams)*fsr),
                                                             cutoff_freq      = 0.495*fsr,
                                                             transition_width = 0.01*fsr,
                                                             attenuation_dB   = 80.0,
                                                             window           = filter.firdes.WIN_BLACKMAN_HARRIS)

        self._pfb_synthesizer = filter.pfb_synthesizer_ccf(numchans = 1+num_streams,
                                                           taps     = (taps_pfb),
                                                           twox     = False)
        channel_map = [num_streams]
        channel_map.extend(range(num_streams))
        self._pfb_synthesizer.set_channel_map(channel_map)
        for i in range(num_streams):
            self.connect((self, i),
                         (self._align_streams, i),
                         (self._rotators, i),
                         (self._rational_resamplers[i], 0),
                         (self._pfb_synthesizer, i))
            self.connect((self._rational_resamplers[i], 0),
                         (self, 1+i))
            # self.connect((self._rotators.get_rotator_block(i), 0),
            #              (self, 1+i))
        self.connect((self._pfb_synthesizer, 0), (self, 0))

        self.msg_connect((self._align_streams.get_find_offsets(), "fs"), (self._rotators, "fs"))

