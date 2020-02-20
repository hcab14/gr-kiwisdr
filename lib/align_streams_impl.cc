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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <cstdlib>
#include <unistd.h>

#include <boost/format.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/logger.h>
#include <gnuradio/tags.h>
#include <volk/volk.h>
#include <pmt/pmt.h>
#include <numeric>

#include "align_streams_impl.h"

namespace gr {
namespace kiwisdr {

align_streams::sptr
align_streams::make(unsigned num_streams,
              bool from_same_kiwi)
{
  return gnuradio::get_initial_sptr
    (new align_streams_impl(num_streams, from_same_kiwi));
}

// private constructor
align_streams_impl::align_streams_impl(unsigned num_streams,
                                       bool from_same_kiwi)
  : gr::sync_block("kiwisdr",
                   gr::io_signature::make(num_streams, num_streams, sizeof(gr_complex)),
                   gr::io_signature::make(num_streams, num_streams, sizeof(gr_complex)))
  , _num_streams(num_streams)
  , _from_same_kiwi(from_same_kiwi)
  , _fs(num_streams)
  , _tags(num_streams)
  , _delays(num_streams)
{
  set_tag_propagation_policy(TPP_DONT);
  message_port_register_out(pmt::intern("fs"));
}

// virtual destructor.
align_streams_impl::~align_streams_impl() {}


int align_streams_impl::work(int noutput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
  gr::thread::scoped_lock lock(d_setlock);
  gr_complex* out = (gr_complex*)(output_items[0]);

  std::vector<std::vector<tag_t>> rx_rate_tags(_num_streams);
  std::vector<std::vector<tag_t>> rx_time_tags(_num_streams);
  for (unsigned i=0; i<_num_streams; ++i) {
    get_tags_in_window(rx_rate_tags[i], i, 0, noutput_items, pmt::intern("rx_rate"));
    if (not rx_rate_tags[i].empty()) {
      _fs[i] = std::accumulate(std::begin(rx_rate_tags[i]),
                               std::end(rx_rate_tags[i]),
                               0.0,
                               [](double acc, tag_t const& t)
                                 {return acc + pmt::to_double(t.value); })/rx_rate_tags[i].size();
    }
    get_tags_in_window(rx_time_tags[i], i, 0, noutput_items, pmt::intern("rx_time"));
    for (auto const& t : rx_time_tags[i]) {
      _tags[i].push_back(rx_time_tag(t.offset,
                                     pmt::to_uint64(pmt::tuple_ref(t.value, 0)) +
                                     pmt::to_double(pmt::tuple_ref(t.value, 1))));
    }
  }

  auto const all_tags_ok =
    [&]() { return std::all_of(std::begin(_tags),
                               std::end(_tags),
                               [](std::deque<rx_time_tag> const& v) { return not v.empty(); }); };
  if (all_tags_ok()) {
    pmt::pmt_t msg = pmt::make_dict();
    msg = pmt::dict_add(msg, pmt::intern("fs"),
                        pmt::init_f64vector(_num_streams, _fs.data()));
    message_port_pub(pmt::intern("fs"), msg);
  }

  std::vector<double> ds(_num_streams, 0.0);
  while (all_tags_ok()) {
    ds[0] = 0.0;
    auto const t0 = _tags[0].front();
    for (unsigned i=1; i<_num_streams; ++i) {
      auto const ti = _tags[i].front();
      ds[i] = _fs[i] *
        ((ti.gnss_sec        - t0.gnss_sec) -
         (ti.samp_num/_fs[i] - t0.samp_num/_fs[0]) +
         ( _delays[i]/_fs[i] -  _delays[0]/_fs[0]));

      std::cout << "offsets[" << i << "] " << ds[i] << " "
                << _fs[0] << " " << _fs[i] << " "
                << (t0.gnss_sec - ti.gnss_sec) << " "
                << (t0.samp_num/_fs[0] - ti.samp_num/_fs[i]) << " "
                << (_delays[0]/_fs[0] - _delays[i]/_fs[i]) << std::endl;
      _tags[i].pop_front();
    }
    _tags[0].pop_front();

    std::vector<int> offsets(_num_streams, 0);
    std::transform(std::begin(ds), std::end(ds),
                   std::begin(offsets),
                   [](double x) { return std::lround(x); });

    int max_offset            =          offsets[0];
    int max_abs_offset        = std::abs(offsets[0]);
    double max_abs_rem_offset = std::abs(offsets[0]-ds[0]);
    for (unsigned i=1; i<_num_streams; ++i) {
      std::cout << "i= " << i << " " << offsets[i] << " " << ds[i] << std::endl;
      max_offset         = std::max(max_offset,                  offsets[i]);
      max_abs_offset     = std::max(max_abs_offset,     std::abs(offsets[i]));
      max_abs_rem_offset = std::max(max_abs_rem_offset, std::abs(offsets[i]-ds[i]));
    }

    if (max_abs_offset < 5*_fs[0] && (max_abs_rem_offset < 0.2 || not _from_same_kiwi)) {
      for (auto& off : offsets)
        off -= max_offset;
      int max_minus_offsets = -offsets[0];
      for (unsigned i=1; i<_num_streams; ++i) {
        max_minus_offsets = std::max(max_minus_offsets, -offsets[i]);
      }
      std::cout << "offsets: ";
      for (int i=0; i<_num_streams; ++i)
        std::cout << offsets[i] << " ";
      std::cout << "(" << max_minus_offsets << ")"<< std::endl;

      if (max_minus_offsets > 0) {
        for (unsigned i=0; i<_num_streams; ++i) {
          int const to_consume = std::min(-offsets[i], noutput_items);
          consume(i, to_consume);
          _delays[i] += to_consume;
          GR_LOG_INFO(d_logger, str(boost::format("to_consume[%d] = %d delays[%d]=%d") % i % to_consume %i %_delays[i]));
          _tags[i].clear();
        }
        usleep(100);
        return 0;
      }
    } else {
      GR_LOG_INFO(d_logger, str(boost::format("max_abs_offset = %f > %f || max_abs_rem_offset = %f > 0.2")
                                % max_abs_offset
                                % (5*_fs[0])
                                % max_abs_rem_offset));
    }
  }
  for (unsigned i=0; i<_num_streams; ++i) {
    for (auto const& t : rx_rate_tags[i])
      add_item_tag(i, t.offset, t.key, t.value, t.srcid);
    for (auto const& t : rx_time_tags[i]) {
      pmt::pmt_t const v =
        pmt::make_tuple(pmt::tuple_ref(t.value, 0),
                        pmt::from_double(pmt::to_double(pmt::tuple_ref(t.value, 1)) + _delays[i]/_fs[i]));
      std::cout << t.srcid << " "<< t.offset << " "; pmt::print(v);
      add_item_tag(i, t.offset, t.key, v, t.srcid);
    }
    gr_complex const *in = static_cast<gr_complex const*>(input_items[i]);
    gr_complex *out = static_cast<gr_complex*>(output_items[i]);
    std::copy(in, in+noutput_items, out);
  }
  return noutput_items;
}

bool align_streams_impl::start() {
  GR_LOG_DEBUG(d_logger, "align_streams_impl::start");
  gr::thread::scoped_lock lock(d_setlock);
  std::fill_n(std::begin(_fs),      _num_streams, -1.0);
  std::fill_n(std::begin(_delays),  _num_streams, 0);
  for (auto & t: _tags)
    t.clear();
  return true;
}
bool align_streams_impl::stop() {
  GR_LOG_DEBUG(d_logger, "align_streams_impl::stop");
  gr::thread::scoped_lock lock(d_setlock);
  return true;
}

} /* namespace kiwisdr */
} /* namespace gr */
