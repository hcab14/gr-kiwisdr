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
import pmt

class find_offsets(gr.sync_block):
    """
    This block determines offsets (number of samples) between a number of streams using rx_time tags
    Note that it assumes that the offsets are close to integer multiples of samples
    """
    def __init__(self, num_streams):
        gr.sync_block.__init__(
            self,
            name    = 'find_offsets',
            in_sig  = num_streams*(np.complex64,),
            out_sig = num_streams*(np.complex64,))
        self._num_streams = num_streams
        self._fs          = 12e3*np.zeros(num_streams, dtype=np.float64) ## is set from 'rx_rate' tags
        self._tags        = num_streams*[[]]
        self._port_delay  = pmt.intern('delay')
        self._port_fs     = pmt.intern('fs')
        self._pmt_align   = pmt.intern('align')
        self.message_port_register_out(self._port_delay)
        self.message_port_register_out(self._port_fs)
        self.set_tag_propagation_policy(gr.TPP_ONE_TO_ONE)

    def work(self, input_items, output_items):
        n = len(input_items[0])

        ## (1) 'rx_rate' tags
        tags_ok = [False]*self._num_streams
        for i in range(self._num_streams):
            tags = self.get_tags_in_window(i, 0, n, pmt.intern('rx_rate'))
            if len(tags) != 0:
                self._fs[i] = np.mean(np.array([pmt.to_python(t.value) for t in tags], dtype=np.float64))
                tags_ok[i] = True
        if all(tags_ok):
            msg_out = pmt.make_dict()
            msg_out = pmt.dict_add(msg_out, pmt.intern('fs'), pmt.to_pmt(self._fs))
            self.message_port_pub(self._port_fs, msg_out)

        ## (2) 'rx_time' tags
        tags_ok = [False]*self._num_streams
        f       = lambda x : np.float64(x[0])+x[1]
        for i in range(self._num_streams):
            for tag in self.get_tags_in_window(i, 0, n, pmt.intern('rx_time')):
                self._tags[i].extend([{'samp_num' : tag.offset,
                                       'gnss_sec' : f(pmt.to_python(tag.value))}])
            tags_ok[i] = (self._tags[i] != [])

        ## (3) compute delays for aligning the streams
        while all(tags_ok):
            ## compute differences w.r.t 1st in number of samples
            fd = lambda x,y,fsx,fsy: x['gnss_sec']-y['gnss_sec'] - (x['samp_num']/fsx-y['samp_num']/fsy)
            ds = np.array([fd(self._tags[i][0], self._tags[0][0], self._fs[i], self._fs[0])
                           for i in range(1,self._num_streams)], dtype=np.double) * self._fs[1:]
            ## remove the processed tags and update tags_ok
            for i in range(self._num_streams):
                self._tags[i] =  self._tags[i][1:]
                tags_ok[i]    = (self._tags[i] != [])
            ## compute offsets avoiding negative delays
            offsets = np.zeros(self._num_streams, dtype=np.int)
            offsets[0]  = 0
            offsets[1:] = np.round(ds)
            if np.max(offsets)<40000 and np.max(np.abs(offsets[1:]-ds)) < 0.2:
                ## publish the offsets to the message port
                msg_out  = pmt.make_dict()
                offsets -= np.min(offsets)
                msg_out  = pmt.dict_add(msg_out, pmt.intern('delays'), pmt.to_pmt([x for x in offsets]))
                self.message_port_pub(self._port_delay, msg_out)
            else:
                gr.log.warn('ds={} fs={}'.format(ds, self._fs))

        ## (4) pass through all data
        for i in range(self._num_streams):
            output_items[i][:] = input_items[i]

        return n

class delay_proxy(gr.sync_block):
    """
    Pure message handling block containing an array of delay blocks.
    Delays are set via message parsing.
    """
    def __init__(self, num_streams):
        gr.sync_block.__init__(
            self,
            name="delay_proxy",
            in_sig  = num_streams*(np.complex64,),
            out_sig = num_streams*(np.complex64,))
        self._num_streams = num_streams
        self._delays      = [blocks.delay(gr.sizeof_gr_complex, 0) for _ in range(num_streams)]
        self._port_delay  = pmt.intern('delay')
        self._got_message = False
        self.message_port_register_in(self._port_delay)
        self.set_msg_handler(self._port_delay, self.msg_handler_delay)

    def get(self, i):
        return self._delays[i]

    def work(self, input_items, output_items):
        if not self._got_message:
            return 0
        n = len(output_items[0])
        for i in range(self._num_streams):
            output_items[i][:] = input_items[i]

        return n
    def msg_handler_delay(self, msg_in):
        msg = pmt.to_python(msg_in)
        delays = msg['delays']
        for i,d in enumerate(delays):
            self._delays[i].set_dly(d)
        self._got_message = True

class align_streams(gr.hier_block2):
    """
    Block for aligning KiwiSDR IQ streams with GNSS timestamps
    Note that all used IQ streams have to be from the same KiwiSDR
    """
    def __init__(self, num_streams):
        gr.hier_block2.__init__(self,
                                "align_streams",
                                gr.io_signature(num_streams, num_streams, gr.sizeof_gr_complex),
                                gr.io_signature(num_streams, num_streams, gr.sizeof_gr_complex))
        self._find_offsets = find_offsets(num_streams)
        self._delays       = delay_proxy(num_streams)
        for i in range(num_streams):
            self.connect((self, i),
                         (self._find_offsets, i),
                         (self._delays, i),
                         (self._delays.get(i)),
                         (self, i))

        self.msg_connect((self._find_offsets, 'delay'), (self._delays, 'delay'))

    def get_find_offsets(self):
        return self._find_offsets
