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

from gnuradio import gr
from gnuradio import gru
from gnuradio import filter
from gnuradio import blocks
from gnuradio import analog

import pmt

from kiwisdr import align_streams


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
        z   = np.sum(in0)
        az  = np.abs(z)
        z   = self._z0*(z/az if az != 0 else 1)
        output_items[0][:] = z
        return len(in0)

class phase_offset_corrector(gr.hier_block2):
    """
    Corrects phase offsets between IQ data streams, assuming that there is no phase drift.
    For this the relative phases between streams are obtained from overlapping regions of the spectra.
    """
    def __init__(self, samp_rate):
        gr.hier_block2.__init__(
            self,
            'phase_offset_corrector',
            gr.io_signature(2, 2, gr.sizeof_gr_complex),
            gr.io_signature(2, 2, gr.sizeof_gr_complex))

        ## the overlapping parts of the spectra are processed at a lower sampling rate
        decim   = 5
        delta_f = float(samp_rate)/4.0
        self._taps = taps = filter.firdes.low_pass(1, samp_rate, 0.05*samp_rate, 0.02*samp_rate)
        self._xlplus   = filter.freq_xlating_fir_filter_ccf(decim, (taps), +delta_f, samp_rate)
        self._xlminus  = filter.freq_xlating_fir_filter_ccf(decim, (taps), -delta_f, samp_rate)
        self._mult_ccA = blocks.multiply_conjugate_cc(1)
        self._mult_ccB = blocks.multiply_conjugate_cc(1)

        vlen = int(np.power(2, np.ceil(np.log2(0.1*samp_rate/decim))))
        self._s2v = blocks.stream_to_vector(gr.sizeof_gr_complex, vlen)
        self._v2s = blocks.vector_to_stream(gr.sizeof_gr_complex, vlen*decim)

        ## phase offsets depend on the combination of resampling and pfb_synth filter
        self._pB = phase_estimator(vlen, decim, np.exp(2j*np.pi*0.5)) ## phase 1 for num_streams==4, 0.5 for num_streams==6

        ## output = conj([conj(#0) * #1]) * #1
        self.connect((self, 0),
                     (self._xlplus),
                     (self._mult_ccA, 1))
        self.connect((self, 1),
                     (self._xlminus),
                     (self._mult_ccA, 0))
        self.connect((self._mult_ccA),
                     (self._s2v),
                     (self._pB),
                     (self._v2s),
                     (self._mult_ccB, 1))
        self.connect((self, 1),
                     (self._mult_ccB, 0))
        self.connect((self._mult_ccB),
                     (self, 0))

        ## phase offsets
        self.connect((self._v2s), (self, 1))

class rotator_proxy(gr.sync_block):
    """
    gr.sync_block (pass-through) containing an array of rotator_cc blocks
    Phase increments are set via message parsing
    """
    def __init__(self, num_streams, fs, df):
        gr.sync_block.__init__(
            self,
            name    = 'rotator_proxy',
            in_sig  = num_streams*(np.complex64,),
            out_sig = num_streams*(np.complex64,))
        self._num_streams = num_streams
        self._fs = fs
        self._df = df
        self._rotators = [blocks.rotator_cc(0) for _ in range(num_streams)]
        self._got_message = False
        self._port_fs = pmt.intern('fs')
        self.message_port_register_in(self._port_fs)
        self.set_msg_handler(self._port_fs, self.msg_handler_rotator)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def get(self, i):
        return self._rotators[i]

    def work(self, input_items, output_items):
        ## make sure a message is received before the 1st sample is passed through
        if not self._got_message:
            return 0
        for i in range(self._num_streams):
            output_items[i][:] = input_items[i]
        return len(input_items[0])

    def msg_handler_rotator(self, msg_in):
        msg = pmt.to_python(msg_in)
        fs = msg['fs']
        n  = self._num_streams
        m  = (n-1)//2
        off = np.arange(n)-m
        for i in range(n):
            phase_inc = 2*np.pi * self._df * (1.0/fs[i]-1.0/self._fs) * off[i]
            self._rotators[i].set_phase_inc(phase_inc)
        self._got_message = True


class final_processing(gr.hier_block2):
    """
    TODO
    """
    def __init__(self, num_streams, delta_f_in):
        gr.hier_block2.__init__(
            self,
            'final_processing',
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(1, 1, gr.sizeof_gr_complex))

        blk_last = (self, 0)
        if (num_streams%2) == 0:
            self._mult_conj = blocks.multiply_conjugate_cc(1)
            self._carrier   = analog.sig_source_c(sampling_freq = 8*delta_f_in,
                                                  waveform      = analog.GR_COS_WAVE,
                                                  wave_freq     = delta_f_in/2.0,
                                                  ampl          = 1.0,
                                                  offset        = 0.0)
            self.connect(blk_last,
                         (self._mult_conj, 0))
            self.connect((self._carrier),
                         (self._mult_conj, 1))
            blk_last = self._mult_conj

        if num_streams <= 10:
            self._final_resamp = filter.rational_resampler_ccf(1,2)
            self.connect(blk_last, (self._final_resamp))
            blk_last = self._final_resamp

        self.connect(blk_last, (self, 0))

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
            gr.io_signature(  num_streams,   num_streams, gr.sizeof_gr_complex),
            gr.io_signature(2*num_streams, 2*num_streams, gr.sizeof_gr_complex))
        assert(num_streams >= 2 and num_streams <= 6)
        fsr    = delta_f_in
        factor = gru.gcd(fsr, fs_in)
        interp = fsr/factor
        decim  = fs_in/factor

        self._align_streams = align_streams(num_streams, True)
        self._rotators      = rotator_proxy(num_streams, fs_in, delta_f_in)

        self._taps_resampler = taps_resampler = filter.firdes.low_pass_2(gain             = 2*interp,
                                                                         sampling_freq    = 2.0,
                                                                         cutoff_freq      = 0.5/decim,
                                                                         transition_width = 0.4/decim,
                                                                         attenuation_dB   = 80.0,
                                                                         window           = filter.firdes.WIN_BLACKMAN_HARRIS)
        self._rational_resamplers = [filter.rational_resampler_ccc(interpolation = 2*interp,
                                                                   decimation    = decim,
                                                                   taps          = (taps_resampler),
                                                                   fractional_bw = None) for _ in range(num_streams)]
        self._taps_pfb = taps_pfb = filter.firdes.low_pass_2(gain             = 4,
                                                             sampling_freq    = 4*fsr,
                                                             cutoff_freq      = 0.5*fsr,
                                                             transition_width = 0.2*fsr,
                                                             attenuation_dB   = 80.0,
                                                             window           = filter.firdes.WIN_BLACKMAN_HARRIS)

        self._pfb_synthesizer = filter.pfb_synthesizer_ccf(numchans = 6,
                                                           taps     = (taps_pfb),
                                                           twox     = True)
        channel_map = []
        if num_streams == 2:
            channel_map = [0,1]
        if num_streams == 3:
            channel_map = [7,0,1]
        if num_streams == 4:
            channel_map = [7,0,1,2]
        if num_streams == 5:
            channel_map = [10,11,0,1,2]
        if num_streams == 6:
            channel_map = [6,7,0,1,2,3]
        self._pfb_synthesizer.set_channel_map(channel_map)

        self._poc = [phase_offset_corrector(2*delta_f_in) for _ in range(1,num_streams)]

        for i in range(num_streams):
            self.connect((self, i),
                         (self._align_streams, i),
                         (self._rotators, i),
                         (self._rotators.get(i)),
                         (self._rational_resamplers[i]))

        ## the phase offset corrector are cascading:
        ## #0 ---------------------- 0
        self.connect((self._rational_resamplers[0]),
                     (self._pfb_synthesizer, 0))
        ## (#0,#1) -> poc[0] ------- 1
        self.connect((self._rational_resamplers[0]),
                     (self._poc[0], 0))
        self.connect((self._rational_resamplers[1]),
                     (self._poc[0], 1))
        self.connect((self._poc[0], 0),
                     (self._pfb_synthesizer, 1))
        ## (poc[0],#2) -> poc[1] --- 2
        ## (poc[1],#3) -> poc[2] --- 3
        ## ...
        for i in range(2,num_streams):
            self.connect((self._poc[i-2], 0),
                         (self._poc[i-1], 0))
            self.connect((self._rational_resamplers[i]),
                         (self._poc[i-1], 1))
            self.connect((self._poc[i-1], 0),
                         (self._pfb_synthesizer, i))

        ## resampled coherent input streams + rel. phase offsets
        self.connect((self._rational_resamplers[0]),
                     (self, 1))
        for i in range(1,num_streams):
            self.connect((self._poc[i-1], 0), (self, 1+i))
            self.connect((self._poc[i-1], 1), (self, num_streams+i))

        ## combined IQ output stream
        ## out: 4*delta_f_in if num_streams <= 3 else 8*delta_f_in
        self._final_processing = final_processing(num_streams, delta_f_in)
        self.connect((self._pfb_synthesizer, 0),
                     (self._final_processing),
                     (self, 0))

        ##self.msg_connect((self._align_streams.get_find_offsets(), 'fs'), (self._rotators, 'fs'))
        self.msg_connect((self._align_streams, 'fs'), (self._rotators, 'fs'))

