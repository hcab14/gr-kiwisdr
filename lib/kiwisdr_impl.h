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

#ifndef INCLUDED_KIWISDR_KIWISDR_IMPL_H
#define INCLUDED_KIWISDR_KIWISDR_IMPL_H

#include <kiwisdr/kiwisdr.h>

#include "kiwi_rx_parameters.h"
#include "kiwi_ws_client.h"

namespace gr {
namespace kiwisdr {

class snd_info_header {
public:
  snd_info_header()
    : _flags()
    , _seq()
    , _smeter{0,0} {
    static_assert(sizeof(snd_info_header) == 7,
                  "snd_info_header has wrong packed size");
  }
  uint8_t  flags() const { return _flags; }
  uint32_t seq()  const { return _seq; }
  float    rssi() const { return 0.1f*((uint16_t(_smeter[0]) << 8) +
                                        uint16_t(_smeter[1])) - 127.0f; }
private:
  uint8_t  _flags;
  uint32_t _seq;
  uint8_t  _smeter[2];
} __attribute__((__packed__)) ;

class gnss_timestamp_header {
public:
  gnss_timestamp_header()
    : _last_gps_solution()
    , _dummy()
    , _gpssec()
    , _gpsnsec() {
    static_assert(sizeof(gnss_timestamp_header) == 10,
                  "gnss_timestamp_header has wrong packed size");
  }
  int      last_gps_solution() const { return _last_gps_solution; }
  uint32_t gpssec()            const { return _gpssec; }
  uint32_t gpsnsec()           const { return _gpsnsec; }
  double   as_double()         const { return _gpssec + 1e-9*_gpsnsec; }
private:
  uint8_t  _last_gps_solution;
  uint8_t  _dummy;
  uint32_t _gpssec;
  uint32_t _gpsnsec;
} __attribute__((__packed__)) ;

class kiwisdr_impl : public kiwisdr
{
private:
  std::shared_ptr<kiwi_ws_client> _ws_client_ptr;

  std::map<std::string, std::string> _msg;

  std::string _host;
  std::string _port;
  kiwi_rx_parameters _rx_parameters;

  snd_info_header       _last_snd_header;
  gnss_timestamp_header _last_gnss_timestamp;

public:
  kiwisdr_impl(std::string const &host,
               std::string const &port,
               double freq_kHz,
               int low_cut_Hz,
               int high_cut_Hz);
  virtual ~kiwisdr_impl();

  // Where all the action really happens
  int general_work(int noutput_items,
                   gr_vector_int& ninput_items,
                   gr_vector_const_void_star& input_items,
                   gr_vector_void_star& output_items);

  virtual bool start();
  virtual bool stop();

  virtual std::string get_client_public_ip() const { return _msg.at("client_public_ip"); }
  virtual int         get_rx_chans()         const { return std::stoi(_msg.at("rx_chans")); }
  virtual int         get_chan_no_pwd()      const { return std::stoi(_msg.at("chan_no_pwd")); }
  virtual bool        is_password_ok()       const { return _msg.at("badp") == "0"; }
  virtual std::string get_version()          const { return _msg.at("version_maj")+"."+_msg.at("version_min"); }
  virtual std::string get_cfg()              const { return _msg.at("load_cfg"); }
  virtual double      get_audio_rate()       const { return std::stod(_msg.at("audio_rate")); }
  virtual double      get_sample_rate()      const { return std::stod(_msg.at("sample_rate")); }
  virtual bool        is_audio_initialized() const { return _msg.at("audio_init") == "1"; }
  virtual double      get_center_freq()      const { return std::stod(_msg.at("center_freq")); }
  virtual double      get_bandwidth()        const { return std::stod(_msg.at("bandwidth")); }
  virtual double      get_adc_clk_nom()      const { return std::stod(_msg.at("adc_clk_nom")); }

  // change rx parameters
  virtual void set_rx_parameters(double freq_kHz,
                                 int low_cut_Hz,
                                 int high_cut_Hz);

private:
};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_KIWISDR_IMPL_H */
