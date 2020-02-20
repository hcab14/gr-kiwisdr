/* -*- c++ -*- */
/*
 * Copyright 2019 Christoph Mayer hcab14@gmail.com.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_KIWISDR_ALIGN_STREAMS_IMPL_H
#define INCLUDED_KIWISDR_ALIGN_STREAMS_IMPL_H

#include <deque>
#include <vector>
#include <gnuradio/tags.h>

#include <kiwisdr/align_streams.h>

namespace gr {
namespace kiwisdr {

struct rx_time_tag {
  rx_time_tag(uint64_t n=0, double s=0)
    : samp_num(n)
    , gnss_sec(s){}
  uint64_t samp_num;
  double gnss_sec;
} ;

class align_streams_impl : public align_streams
{
public:
  align_streams_impl(unsigned num_streams,
                     bool from_same_kiwi);
  virtual ~align_streams_impl();

  virtual int work(int noutput_items,
                   gr_vector_const_void_star& input_items,
                   gr_vector_void_star& output_items);

  virtual bool start();
  virtual bool stop();

private:
  unsigned _num_streams;
  bool     _from_same_kiwi;
  std::vector<double> _fs;
  std::vector<std::deque<rx_time_tag>> _tags;
  std::vector<int> _delays;
};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_ALIGN_STREAMS_IMPL_H */
