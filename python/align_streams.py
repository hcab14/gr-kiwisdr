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
    docstring for block find_offsets
    """
    def __init__(self, num_streams):
        gr.sync_block.__init__(self,
                               name    = "find_offsets",
                               in_sig  = gr.py_io_signature(1, -1, num_streams*(np.complex64,)),
                               out_sig = gr.py_io_signature(1, -1, num_streams*(np.complex64,)))
        self._num_streams = num_streams
        self._tags = [[] for _ in range(num_streams)]
        self._fs   = 12001*np.ones(num_streams, dtype=np.double)
        self._offsets = np.zeros(num_streams, dtype=np.int)
        self._tags_new = [False for _ in range(num_streams)]
        self._port_delay = pmt.intern('delay')
        self.message_port_register_out(self._port_delay)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def work(self, input_items, output_items):
        n = len(output_items[0])
        for i in range(self._num_streams):
            output_items[i][:] = input_items[i]

        tags=[[] for _ in range(self._num_streams)]
        f = lambda x : x[0]+x[1]
        for i in range(self._num_streams):
            ## tags[i][0] => abs sample#
            ## tags[i[[1] => GNSS timestamp
            tags[i] = [(x.offset, f(pmt.to_python(x.value)))
                       for x in self.get_tags_in_window(i, 0, n)
                       if pmt.to_python(x.key) == 'rx_time']
            if tags[i] != []:
                if self._tags[i] != []:
                    self._fs[i] = (self._tags[i][0] - tags[i][0][0])/(self._tags[i][1] - tags[i][0][1])
                self._tags[i] = tags[i][0]
                self._tags_new[i] = True
        if all(self._tags_new):
            #print('tags_new:', self._tags)
            ## compute differences w.r.t 1st
            fd = lambda x,y,fsx,fsy:(x[0]/fsx-y[0]/fsy) - (x[1]-y[1])
            dd = np.array([fd(self._tags[i], self._tags[0], self._fs[i], self._fs[0])
                           for i in range(1,self._num_streams)])
            dds = dd * self._fs[1:]
            self._offsets[0]  = 0
            self._offsets[1:] = np.round(dds)
            self._offsets    -= np.min(self._offsets)
            print('dd= ', dd, dds, np.round(dds), (dds-np.round(dds))/self._fs[1:]*1e9, self._offsets)
            self._tags_new = [False for _ in range(self._num_streams)]

            ## set offsets to delay taps
            msg_out = pmt.make_dict()
            msg_out = pmt.dict_add(msg_out, pmt.intern('delays'), pmt.to_pmt([x for x in self._offsets]))
            #print(msg_out)
            self.message_port_pub(self._port_delay, msg_out)

        #print(tags)
        return n

class delay_array(gr.basic_block):
    """
    docstring for block find_offsets
    """
    def __init__(self, num_streams):
        gr.basic_block.__init__(
            self,
            name="delay_array",
            in_sig=None,
            out_sig=None)
        self._num_streams = num_streams
        self._delays = [blocks.delay(gr.sizeof_gr_complex, 0) for _ in range(num_streams)]

        self._port_delay = pmt.intern('delay')
        self.message_port_register_in(self._port_delay)
        self.set_msg_handler(self._port_delay, self.msg_handler_delay)
        self.set_tag_propagation_policy(gr.TPP_DONT)

    def get_dly_block(self, i):
        return self._delays[i]

    def msg_handler_delay(self, msg_in):
        #print('-------------------- msg_handler_delay --------------------')
        #print('msg_in= ', msg_in)
        delays = pmt.to_python(pmt.cdar(msg_in))
        print('delays=', delays)
        for (i,d) in enumerate(delays):
            print('delay',i,self._delays[i].dly(), d)
            self._delays[i].set_dly(d)

class align_streams(gr.hier_block2):
    """
    docstring for block physical_layer_driver
    """
    def __init__(self, num_streams):
        print '__init__', num_streams
        gr.hier_block2.__init__(self,
                                "align_streams",
                                gr.io_signature(num_streams, num_streams, gr.sizeof_gr_complex),
                                gr.io_signature(num_streams, num_streams, gr.sizeof_gr_complex))
        self._find_offsets = find_offsets(num_streams)
        self._delays       = delay_array(num_streams)
        for i in range(num_streams):
            self.connect((self, i), (self._find_offsets, i), (self._delays.get_dly_block(i), 0), (self, i))

        self.msg_connect((self._find_offsets, 'delay'), (self._delays, 'delay'))

