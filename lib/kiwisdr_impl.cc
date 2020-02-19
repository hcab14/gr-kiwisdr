/* -*- c++ -*- */
/*
 * Copyright 2018 Christoph Mayer hcab14@gmail.com.
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

#include <boost/format.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/logger.h>
#include <gnuradio/tags.h>
#include <volk/volk.h>

#include "kiwisdr_impl.h"

namespace gr {
namespace kiwisdr {

kiwisdr::sptr
kiwisdr::make(std::string const &host,
              std::string const &port,
              double freq_kHz,
              int low_cut_Hz,
              int high_cut_Hz)
{
  return gnuradio::get_initial_sptr
    (new kiwisdr_impl(host, port, freq_kHz, low_cut_Hz, high_cut_Hz));
}

// private constructor
kiwisdr_impl::kiwisdr_impl(std::string const &host,
                           std::string const &port,
                           double freq_kHz,
                           int low_cut_Hz,
                           int high_cut_Hz)
  : gr::block("kiwisdr",
              gr::io_signature::make (0, 0, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float)))
  , _ws_client_ptr()
  , _msg()
  , _host(host)
  , _port(port)
  , _rx_parameters{freq_kHz, low_cut_Hz, high_cut_Hz}
  , _last_snd_header()
  , _last_gnss_timestamp()
  , _last_gnss_time(0)
  , _sample_rate(12e3)
  , _sample_rate_counter(0)
  , _rate_tag_ok(false)
  , _gnss_tag_done(false)
  , _id(pmt::mp(_host+":"+_port))
{
    GR_LOG_DECLARE_LOGPTR(d_logger);
    GR_LOG_ASSIGN_LOGPTR(d_logger, "kiwisdr@"+_host+":"+_port);
}

// virtual destructor.
kiwisdr_impl::~kiwisdr_impl() {}


int kiwisdr_impl::general_work(int noutput_items,
                               gr_vector_int& ninput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star& output_items)
{
  gr::thread::scoped_lock lock(d_setlock);
  int produced_output_items = 0;

  if (!_ws_client_ptr) {
    usleep(10);
    return produced_output_items;
  }

  gr::thread::scoped_lock lock2(_ws_client_ptr->mutex());
  if (!_ws_client_ptr->get_cond().timed_wait(lock2, boost::posix_time::milliseconds(10))) // timeout
    return produced_output_items;

  gr_complex* out = (gr_complex*)(output_items[0]);

  snd_info_header       snd_info;
  gnss_timestamp_header gnss_timestamp;

  std::size_t const full_header_length  = sizeof(snd_info) + sizeof(gnss_timestamp);
  assert(snd_buffer.size() >= full_header_length);

  while (!_ws_client_ptr->data_queue().empty() && produced_output_items < noutput_items) {
    std::vector<std::uint8_t> const& snd_buffer = _ws_client_ptr->data_queue().front();
    assert((snd_buffer.size() - header_length) % 2 == 0);

    std::size_t const num_floats = (snd_buffer.size() - full_header_length) >> 1;
    assert(num_float % 2 == 0);

    std::size_t const num_complex_samples = num_floats >> 1;
    // for now we do not handle the following case
    assert(num_complex_samples < noutput_items);

    // do not overflow the output
    if (produced_output_items + ((snd_buffer.size()-full_header_length)>>2) >= noutput_items)
      break;

    // (1) decode snd_info_header
    std::memcpy(&snd_info, &snd_buffer[0], sizeof(snd_info));
    int header_length = sizeof(snd_info);

    if (snd_info.seq() - _last_snd_header.seq() != 1)
      GR_LOG_WARN(d_logger, "dropped packet");

    _last_snd_header = snd_info;

    // (2) decode gnss timestamp header (IQ mode only)
    std::memcpy(&gnss_timestamp, &snd_buffer[sizeof(snd_info_header)], sizeof(gnss_timestamp));
    header_length += sizeof(gnss_timestamp_header);
    //     insert a stream tag for each new (=not interpolated) gps timestamp
    if (gnss_timestamp.last_gps_solution() - _last_gnss_timestamp.last_gps_solution() < 0 &&
        !_gnss_tag_done && _sample_rate_counter > 0) {
      GR_LOG_DEBUG(d_logger,(boost::format("SND: seq= %5d RSSi=%5.1f gpssec=%16.9f (%3d)")
                             % snd_info.seq()
                             % snd_info.rssi()
                             % gnss_timestamp.as_double()
                             % gnss_timestamp.last_gps_solution()));
      double const gnss_time = gnss_timestamp.as_double();
      double delta_t = gnss_time - _last_gnss_time;
      delta_t += (delta_t < 0)*7*24*3600; // GNSS week rollover
      _sample_rate = _sample_rate_counter/delta_t;
      if (_sample_rate > 10e3) {
        add_item_tag(0, nitems_written(0)+produced_output_items,
                     RATE_KEY, pmt::from_double(_sample_rate), _id);
        _rate_tag_ok = true;
      }
      if (_rate_tag_ok) {
        // taken from gr-uhd/lib/usrp_source_impl.cc
        pmt::pmt_t const val = pmt::make_tuple(pmt::from_uint64(gnss_timestamp.gpssec()),
                                               pmt::from_double(1e-9*gnss_timestamp.gpsnsec()));
        add_item_tag(0, nitems_written(0)+produced_output_items,
                     TIME_KEY, val, _id);
      }
      _last_gnss_time = gnss_time;
      _sample_rate_counter = 0;
    }
    _gnss_tag_done        = (gnss_timestamp.last_gps_solution() == 0);
    _last_gnss_timestamp  = gnss_timestamp;
    _sample_rate_counter += num_complex_samples;

    if (_rate_tag_ok) {
      //    insert a tag with the RSSI value (dB)
      add_item_tag(0, nitems_written(0)+produced_output_items,
                   RSSI_KEY, pmt::mp(snd_info.rssi()), _id);
      // (2) big-endian -> little endian conversion
      volk_16u_byteswap((uint16_t*)(&snd_buffer[header_length]), num_floats);

      // (3) uint16_t -> float conversion
      volk_16i_s32f_convert_32f((float*)(out), (int16_t const*)(&snd_buffer[header_length]), float((1<<15)-1), num_floats);

      produced_output_items += num_complex_samples;
      out                   += num_complex_samples;
    }
    _ws_client_ptr->data_queue().pop();
  }
  // Tell runtime system how many output items we produced.
  return produced_output_items;
}

bool kiwisdr_impl::start() {
  GR_LOG_DEBUG(d_logger, "kiwisdr_impl::start");
  gr::thread::scoped_lock lock(d_setlock);

  _rate_tag_ok = false;

  if (_ws_client_ptr)
    return false;

  _ws_client_ptr = std::make_shared<kiwi_ws_client>();
  _ws_client_ptr->connect(_host, _port, _rx_parameters);

  return true;
}
bool kiwisdr_impl::stop() {
  GR_LOG_DEBUG(d_logger, "kiwisdr_impl::stop");
  gr::thread::scoped_lock lock(d_setlock);
  if (!_ws_client_ptr)
    return false;

  _ws_client_ptr->disconnect();
  _ws_client_ptr = nullptr;
  return true;
}

void kiwisdr_impl::set_rx_parameters(double freq_kHz,
                                     int low_cut_Hz,
                                     int high_cut_Hz) {

  gr::thread::scoped_lock lock(d_setlock);
  _rx_parameters.freq_kHz    = freq_kHz;
  _rx_parameters.low_cut_Hz  = low_cut_Hz;
  _rx_parameters.high_cut_Hz = high_cut_Hz;
  if (_ws_client_ptr) {
    _ws_client_ptr->set_rx_parameters(_rx_parameters);
  }
}

} /* namespace kiwisdr */
} /* namespace gr */
