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

import weakref
import numpy as np
from gnuradio import blocks
from gnuradio import gr
from gnuradio import filter
from gnuradio import analog
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
        self._counter = 0
        self._z = 1

    def work(self, input_items, output_items):
        in0 = input_items[0]
        ##z   = np.sum(in0[np.abs(in0) > 1e-3])
        z   = np.sum(in0)
        az  = np.abs(z)
        #if self._counter < 100:
        self._z = self._z0*(z/az if az != 0 else 1)
        self._counter += 1
        print('XXX z=', np.angle(self._z), self._z0)
        output_items[0][:] = self._z
        return len(input_items[0])

class phase_offset_corrector(gr.hier_block2):
    """
    Corrects phase offsets between IQ data streams, assuming that there is no phase drift.
    For this the relative phases between streams are obtained from overlapping regions of the spectra.
    """
    def __init__(self, num_streams, samp_rate, delta_f, rs, skip):
        gr.hier_block2.__init__(
            self,
            'phase_offset_corrector',
            gr.io_signature(2*num_streams,   2*num_streams,   gr.sizeof_gr_complex),
            gr.io_signature(2, 2, gr.sizeof_gr_complex))
            # gr.io_signature(2*num_streams-1, 2*num_streams-1, gr.sizeof_gr_complex))

        ## for now we can handle only exactly three streams
        assert(num_streams == 3)

        ## the overlapping parts of the spectra are processed at a lower sampling rate
        decim = int(np.floor(samp_rate/float(samp_rate-delta_f)))
        ##decim = 1
        self._taps = taps = filter.firdes.low_pass(1, samp_rate, 0.4*(samp_rate-delta_f), 0.01*(samp_rate-delta_f))
        self._xlAplus   = filter.fir_filter_ccf(decimation=decim, taps=(taps)) ## +df/2
        self._xlAminus  = filter.fir_filter_ccf(decimation=decim, taps=(taps)) ## -df/2
        self._xlBplus   = filter.fir_filter_ccf(decimation=decim, taps=(taps)) ## +df/2
        self._xlBminus  = filter.fir_filter_ccf(decimation=decim, taps=(taps)) ## -df/2
        self._mult_xlAminus = blocks.multiply_cc(1)
        self._mult_xlAplus  = blocks.multiply_conjugate_cc(1)
        self._mult_xlBminus = blocks.multiply_cc(1)
        self._mult_xlBplus  = blocks.multiply_conjugate_cc(1)
        self._mult_ccA1 = blocks.multiply_conjugate_cc(1)
        self._mult_ccB1 = blocks.multiply_conjugate_cc(1)
        self._mult_ccA2 = blocks.multiply_conjugate_cc(1)
        self._mult_ccB2 = blocks.multiply_conjugate_cc(1)

        vlen_dt = 0.1 ## 0.1 second blocks
        vlen = int(np.power(2,np.round(np.log2(vlen_dt*samp_rate/decim))))
        vlen=1024*8
        self._s2vA = blocks.stream_to_vector(gr.sizeof_gr_complex, vlen)
        self._s2vB = blocks.stream_to_vector(gr.sizeof_gr_complex, vlen)
        self._v2sA = blocks.vector_to_stream(gr.sizeof_gr_complex, vlen*decim)
        self._v2sB = blocks.vector_to_stream(gr.sizeof_gr_complex, vlen*decim)

        ## why are the needed phase offsets exp(1j*pi)?
        xxA = delta_f/samp_rate/2
        xxB = delta_f/samp_rate/2
        xxA = 0
        xxB = 0
        self._pA = phase_estimator(vlen, decim, np.exp(-1j*np.pi*xxA))
        self._pB = phase_estimator(vlen, decim, np.exp(+1j*np.pi*xxB))

        self._kludge = blocks.skiphead(gr.sizeof_gr_complex, 0)

        self.connect((self, 5),  ## conj(ref#3)
                     (self._mult_xlAminus, 1))
        self.connect((self, 3),  ## ref#2
                     (self._mult_xlAplus, 1))
        self.connect((self, 3),  ## conj(ref#2)
                     (self._mult_xlBminus, 1))
        self.connect((self, 1),  ## ref#1
                     (self._mult_xlBplus, 1))

        ## [0,+delta_f]
        self.connect((self, 4),
                     (self._mult_xlAminus, 0),
                     (self._xlAminus),
                     #(rs[3]),
                     #(skip[3]),
                     (self._mult_ccA1, 0))
        self.connect((self, 2),
                     (self._mult_xlAplus, 0),
                     (self._xlAplus),
                     #(rs[2]),
                     #(skip[2]),
                     (self._mult_ccA1, 1))
        self.connect((self._mult_ccA1),
                     (self._s2vA),
                     (self._pA),
                     (self._v2sA))
        #              (self._mult_ccA2, 1))
        # self.connect((self, 4),
        #              (self._mult_ccA2, 0))
        # self.connect((self._mult_ccA2),
        #              (self, 2))

        ## 0 (pass-through)
        # self.connect((self, 2),
        #              (self._kludge),
        #              (self, 1))

        ## [-delta_f,0]
        self.connect((self, 0),
                     (self._mult_xlBplus, 0),
                     (self._xlBplus),
                     #(rs[1]),
                     #(skip[1]),
                     (self._mult_ccB1, 0))
        self.connect((self, 2),
                     (self._mult_xlBminus, 0),
                     (self._xlBminus),
                     #(rs[0]),
                     #(skip[0]),
                     (self._mult_ccB1, 1))
        self.connect((self._mult_ccB1),
                     (self._s2vB),
                     (self._pB),
                     (self._v2sB))
        #              (self._mult_ccB2, 1))
        # self.connect((self, 0),
        #              (self._mult_ccB2, 0))
        # self.connect((self._mult_ccB2),
        #              (self, 0))

        ## phase offsets
        self.connect((self._v2sA), (self, 0))
        self.connect((self._v2sB), (self, 1))

class resampler_proxy(gr.basic_block):
    """
    Pure message handling block containing an array of rotator_cc blocks
    Phase increments are set via message parsing
    """
    def __init__(self, rs, fs):
        gr.basic_block.__init__(
            self,
            name    = 'resampler_proxy',
            in_sig  = None,
            out_sig = None)
        self._rs = rs
        self._fs = float(fs)
        self._port_fs = pmt.intern('fs')
        self.message_port_register_in(self._port_fs)
        self.set_msg_handler(self._port_fs, self.msg_handler)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def msg_handler(self, msg_in):
        fs = pmt.to_python(pmt.cdar(msg_in))
        #fs = 3*[20250]
        print('fs=', fs)
        for i,r in enumerate(self._rs):
            r.set_rate(self._fs/fs[i//2])

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
    def __init__(self, num_streams, fs_in, delta_f_in, skip):
        gr.hier_block2.__init__(
            self,
            'coh_stream_synth',
            gr.io_signature(num_streams*2, num_streams*2, gr.sizeof_gr_complex),
            gr.io_signature(num_streams*2, num_streams*2, gr.sizeof_gr_complex))
        assert(num_streams == 3)
        fsr    = delta_f_in
        factor = gcd(fsr, fs_in)
        interp = fsr/factor
        decim  = fs_in/factor

        self._align_streams = align_streams(num_streams)

        self._taps_resampler = taps_resampler = filter.firdes.low_pass(gain             = decim+0*interp,
                                                                       sampling_freq    = 1.0,
                                                                       cutoff_freq      = 0.5/decim,
                                                                       transition_width = 0.01/decim,
                                                                       window           = filter.firdes.WIN_HAMMING)
        self._rational_resamplers = [filter.rational_resampler_ccf(interpolation = interp,
                                                                   decimation    = decim,
                                                                   taps          = (taps_resampler),
                                                                   fractional_bw = None) for _ in range(num_streams)]
        self._rational_resamplers2 = [filter.rational_resampler_ccf(interpolation = interp,
                                                                    decimation    = decim,
                                                                    taps          = (taps_resampler),
                                                                    fractional_bw = None) for _ in range(4)]
        filter_size_resampler = 32
        self._taps_r = taps_r = filter.firdes.low_pass(gain             = filter_size_resampler,
                                                       sampling_freq    = filter_size_resampler*fs_in,
                                                       cutoff_freq      = 0.495*fs_in,
                                                       transition_width = 0.01*fs_in,
                                                       window           = filter.firdes.WIN_HAMMING)
        self._resamplers = [filter.pfb_arb_resampler_ccf(rate=1.0,#20250.0/20251.0096,
                                                         taps=taps_r,
                                                         filter_size=filter_size_resampler) for _ in range(2*num_streams)]
        self._resampler_proxy = resampler_proxy(self._resamplers, fs_in)


        self._taps_pfb = taps_pfb = filter.firdes.low_pass_2(gain             = (1+num_streams),
                                                             sampling_freq    = float((1+num_streams)*fsr),
                                                             cutoff_freq      = 0.5*fsr, ## 0.495
                                                             transition_width = 0.01*fsr,
                                                             attenuation_dB   = 80.0,
                                                             window           = filter.firdes.WIN_BLACKMAN_HARRIS)

        self._pfb_synthesizer = filter.pfb_synthesizer_ccf(numchans = 1+num_streams,
                                                           taps     = (taps_pfb),
                                                           twox     = False)
        channel_map = [num_streams]
        channel_map.extend(range(num_streams))
        self._pfb_synthesizer.set_channel_map(channel_map)
        #help(self._pfb_synthesizer)
        #self._pfb_synthesizer.print_taps()
        #print('rel_rate=', self._pfb_synthesizer.relative_rate())
        #return
        self._null = [blocks.null_sink(gr.sizeof_gr_complex*1) for _ in range(3)]
 #       self._null_2 = [blocks.null_sink(gr.sizeof_gr_complex*1) for _ in range(3)]
 #       self._null_s = [blocks.null_source(gr.sizeof_gr_complex*1) for _ in range(2)]

        self._skip = [blocks.skiphead(gr.sizeof_gr_complex, skip) for _ in range(2*num_streams)]
        self._skip2 = [blocks.skiphead(gr.sizeof_gr_complex, skip) for _ in range(4)]

        self._phase_offset_corrector = phase_offset_corrector(num_streams, fs_in, delta_f_in,
                                                              self._rational_resamplers2, self._skip2)

        self._mc = 3*[[]]
        xx = np.exp(-2j*np.pi*0.11)  #skip=0
        #xx = np.exp(+2j*np.pi*0.12) #skip=1
        self._mc[0] = blocks.multiply_const_cc(xx)
        self._mc[1] = blocks.multiply_const_cc(1)
        self._mc[2] = blocks.multiply_const_cc(np.conj(xx))

        #dp = 0;
        #self._resamplers[0].set_phase(0*dp)
        #self._resamplers[1].set_phase(1*dp)
        #self._resamplers[2].set_phase(2*dp)
        print('phases:', [self._resamplers[i].phase() for i in range(2*num_streams)])
        #for i in range(2*num_streams):
        #    self._resamplers[i].set_phase(0.0)

        #self._carrier = analog.sig_source_c(fs_in, analog.GR_COS_WAVE, delta_f_in/2.0, 1.0)

        for i in range(num_streams):
            ## in#i
            self.connect((self, 2*i),
                         (self._align_streams, 2*i),
                         (self._skip[2*i]),
                         (self._resamplers[2*i]))

            ## ref#i
            self.connect((self, 2*i+1),
                         (self._align_streams, 2*i+1),
                         (self._skip[2*i+1]),
                         (self._resamplers[2*i+1]),
                         (self._null[i]))
            #(self._phase_offset_corrector, 2*i+1))

            self.connect(#(self._align_streams, 2*i),
                #(self._skip[2*i]),
                (self._resamplers[2*i]),
                         (self._phase_offset_corrector, 2*i))
            self.connect(#(self._align_streams, 2*i+1),
                #(self._skip[2*i+1]),
                (self._resamplers[2*i+1]),
                #(self._carrier),
                (self._phase_offset_corrector, 2*i+1))

            # self.connect((self._phase_offset_corrector, i),
            #              (self._null_2[i]))

            #self.connect((self._resamplers[2*i+1]),
            #             (self._phase_offset_corrector, 2*i+1))

            ## resample
            self.connect(#(self._phase_offset_corrector, i),
                (self._resamplers[2*i]),
                (self._rational_resamplers[i]))

        for i in range(num_streams):
            self.connect((self._rational_resamplers[i]),
                         (self._mc[i]),
                         (self._pfb_synthesizer, i))
            self.connect((self._rational_resamplers[i]),
                         (self, 1+i))
        for i in range(num_streams-1):
            self.connect(#(self._null_s[i]),
                         (self._phase_offset_corrector, num_streams*0+i),
                         (self, num_streams+1+i))
        self.connect((self._pfb_synthesizer, 0), (self, 0))

        self.msg_connect((self._align_streams.get_find_offsets(), 'fs'), (self._resampler_proxy, 'fs'))

