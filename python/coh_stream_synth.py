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

class phase_estimator(gr.sync_block):
    """
    Assumes that the input vector contains random IQ samples, all with the same constant phase.
    Output: vlen*interp same complex numbers with norm 1 having this phase
    """
    def __init__(self, vlen, interp, z0):
        gr.sync_block.__init__(
            self,
            name='phase_estimator',
            in_sig  = [(np.complex64, vlen)],
            out_sig = [(np.complex64, vlen*interp)])
        self._z0 = z0

    def work(self, input_items, output_items):
        in0 = input_items[0]
        z   = np.sum(in0[np.abs(in0) > 1e-5])
        az  = np.abs(z)
        z   = self._z0*(z/az if az != 0 else 1)
        z = -1
        output_items[0][:] = z
        return len(input_items[0])

class phase_offset_corrector(gr.hier_block2):
    """
    Corrects phase offsets between IQ data streams, assuming that there is no phase drift.
    For this the relative phases between streams are obtained from overlapping regions of the spectra.
    """
    def __init__(self, num_streams, samp_rate, delta_f):
        gr.hier_block2.__init__(
            self,
            'phase_offset_corrector',
            gr.io_signature(  num_streams,     num_streams,   gr.sizeof_gr_complex),
            gr.io_signature(2*num_streams-1, 2*num_streams-1, gr.sizeof_gr_complex))

        ## for now we can handle only exactly three streams
        assert(num_streams == 3)

        ## the overlapping parts of the spectra are processed at a lower sampling rate
        decim = int(np.floor(samp_rate/float(samp_rate-delta_f)))
        decim = 1
        self._taps = taps = filter.firdes.low_pass(1, samp_rate, 0.45*(samp_rate-delta_f), samp_rate*0.1)
        self._xlAplus   = filter.freq_xlating_fir_filter_ccf(decim, (taps), +delta_f/2.0, samp_rate)
        self._xlAminus  = filter.freq_xlating_fir_filter_ccf(decim, (taps), -delta_f/2.0, samp_rate)
        self._xlBplus   = filter.freq_xlating_fir_filter_ccf(decim, (taps), +delta_f/2.0, samp_rate)
        self._xlBminus  = filter.freq_xlating_fir_filter_ccf(decim, (taps), -delta_f/2.0, samp_rate)
        self._mult_ccA1 = blocks.multiply_conjugate_cc(1)
        self._mult_ccB1 = blocks.multiply_conjugate_cc(1)
        self._mult_ccA2 = blocks.multiply_conjugate_cc(1)
        self._mult_ccB2 = blocks.multiply_conjugate_cc(1)

        vlen = int(np.floor(samp_rate/decim/10))
        self._s2vA = blocks.stream_to_vector(gr.sizeof_gr_complex, vlen)
        self._s2vB = blocks.stream_to_vector(gr.sizeof_gr_complex, vlen)
        self._v2sA = blocks.vector_to_stream(gr.sizeof_gr_complex, vlen*decim)
        self._v2sB = blocks.vector_to_stream(gr.sizeof_gr_complex, vlen*decim)

        ## phase offsets depend on the combination of resampling and pfb_synth filter
        self._pA = phase_estimator(vlen, decim, 1.0)
        self._pB = phase_estimator(vlen, decim, 1.0)

        self._kludge = blocks.skiphead(gr.sizeof_gr_complex, 0)

        ## [0,+delta_f]
        self.connect((self, 2),
                     (self._xlAminus),
                     (self._mult_ccA1, 0))
        self.connect((self, 1),
                     (self._xlAplus),
                     (self._mult_ccA1, 1))
        self.connect((self._mult_ccA1),
                     (self._s2vA),
                     (self._pA),
                     (self._v2sA),
                     (self._mult_ccA2, 1))
        self.connect((self, 2),
                     (self._mult_ccA2, 0))
        self.connect((self._mult_ccA2),
                     (self, 2))

        ## 0 (pass-through)
        self.connect((self, 1),
                     (self._kludge),
                     (self, 1))

        ## [-delta_f,0]
        self.connect((self, 0),
                     (self._xlBplus),
                     (self._mult_ccB1, 0))
        self.connect((self, 1),
                     (self._xlBminus),
                     (self._mult_ccB1, 1))
        self.connect((self._mult_ccB1),
                     (self._s2vB),
                     (self._pB),
                     (self._v2sB),
                     (self._mult_ccB2, 1))
        self.connect((self, 0),
                     (self._mult_ccB2, 0))
        self.connect((self._mult_ccB2),
                     (self, 0))

        ## phase offsets
        self.connect((self._v2sA), (self, 3))
        self.connect((self._v2sB), (self, 4))

class rotator_proxy(gr.basic_block):
    """
    Pure message handling block containing an array of rotator_cc blocks
    Phase increments are set via message parsing
    """
    def __init__(self, num_streams, fs, df):
        gr.basic_block.__init__(
            self,
            name    = 'rotator_proxy',
            in_sig  = None,
            out_sig = None)
        self._num_streams = num_streams
        self._fs = fs
        self._df = df
        self._rotators = [blocks.rotator_cc(0) for _ in range(num_streams)]
        self._is_coh = False
        self._port_fs = pmt.intern('fs')
        self.message_port_register_in(self._port_fs)
        self.set_msg_handler(self._port_fs, self.msg_handler_rotator)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def get(self, i):
        return self._rotators[i]

    def msg_handler_rotator(self, msg_in):
        fs = pmt.to_python(pmt.cdar(msg_in))
        n  = self._num_streams
        m  = (n-1)/2
        phase_inc = 2*np.pi * self._df * (1.0/fs-1.0/self._fs) * np.linspace(-m,m,n)
        for i,p in enumerate(phase_inc):
            self._rotators[i].set_phase_inc(p)

def gcd(a, b):
    return gcd(b, a % b) if b else a

class coh_stream_synth(gr.hier_block2):
    """
    Block for coherently combining three KiwiSDR IQ streams into a single stream with larger bandwidth.
Parameters:
    fs_in      ... sample frequency of input streams in#0..2
    delta_f_in ... frequency spacing of the input streams
Inputs and outputs
    in#0..2        ... input IQ streams samples at fs_in
    out#0          ... combined IQ stream sampled at 4*delta_f_in
    in_resamp#0..2 ... resampled and coh. aligned input stream sampled at delta_f_in (optional)
    phase_off#0    ... phase offset between stream#0 and stream#1 (optional)
    phase_off#1    ... phase offset between stream#1 and stream#2 (optional)
    """
    def __init__(self, num_streams, fs_in, delta_f_in):
        gr.hier_block2.__init__(
            self,
            'coh_stream_synth',
            gr.io_signature(num_streams,   num_streams,   gr.sizeof_gr_complex),
            gr.io_signature(num_streams*2, num_streams*2, gr.sizeof_gr_complex))
        assert(num_streams == 3)
        fsr    = delta_f_in
        factor = gcd(fsr, fs_in)
        interp = fsr/factor
        decim  = fs_in/factor

        self._align_streams = align_streams(num_streams)
        self._rotators      = rotator_proxy(num_streams, fs_in, delta_f_in)

        self._phase_offset_corrector = phase_offset_corrector(num_streams, fs_in, delta_f_in)

        self._taps_resampler = taps_resampler = filter.firdes.low_pass_2(gain             = decim,
                                                                         sampling_freq    = 2.0,
                                                                         cutoff_freq      = 0.5/decim,
                                                                         transition_width = 0.4/decim,
                                                                         attenuation_dB   = 80.0,
                                                                         window           = filter.firdes.WIN_BLACKMAN_HARRIS)#filter.firdes.WIN_HAMMING)#
        self._rational_resamplers = [filter.rational_resampler_ccc(interpolation = 2*interp,
                                                                   decimation    = decim,
                                                                   taps          = (taps_resampler),
                                                                   fractional_bw = None) for _ in range(num_streams)]
        self._taps_pfb = taps_pfb = filter.firdes.low_pass_2(gain             = (1+num_streams),
                                                             sampling_freq    = float((1+num_streams)*fsr),
                                                             cutoff_freq      = 0.5*fsr,
                                                             transition_width = 0.2*fsr,
                                                             attenuation_dB   = 80.0,
                                                             window           = filter.firdes.WIN_BLACKMAN_HARRIS)

        self._pfb_synthesizer = filter.pfb_synthesizer_ccf(numchans = 1+num_streams,
                                                           taps     = (taps_pfb),
                                                           twox     = True)
        channel_map = [num_streams]
        channel_map.extend(range(num_streams))
        channel_map = [7,0,1]
        self._pfb_synthesizer.set_channel_map(channel_map)

        ## for testing
        nskip = 1
        self._skip = [blocks.skiphead(gr.sizeof_gr_complex, nskip),
                      blocks.skiphead(gr.sizeof_gr_complex, nskip),
                      blocks.skiphead(gr.sizeof_gr_complex, nskip)]
        for i in range(num_streams):
            self.connect((self, i),
                         (self._align_streams, i),
                         (self._skip[i]),
                         (self._rotators.get(i)),
                         (self._phase_offset_corrector, i),
                         (self._rational_resamplers[i], 0),
                         (self._pfb_synthesizer, i))
            self.connect((self._rational_resamplers[i]),
                         (self, 1+i))
        for i in range(num_streams-1):
            self.connect((self._phase_offset_corrector, num_streams+i),
                         (self, num_streams+1+i))
        self.connect((self._pfb_synthesizer, 0), (self, 0))

        self.msg_connect((self._align_streams.get_find_offsets(), 'fs'), (self._rotators, 'fs'))

