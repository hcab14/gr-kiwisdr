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

#define BOOST_ASIO_ENABLE_HANDLER_TRACKING

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/format.hpp>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <deque>

#include <gnuradio/thread/thread.h>

#include <kiwisdr/kiwisdr.h>


using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

namespace gr {
namespace kiwisdr {

class snd_info_header {
public:
  snd_info_header()
    : _flags(0)
    , _seq(0)
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
    : _last_gps_solution(0)
    , _dummy(0)
    , _gpssec(0)
    , _gpsnsec(0) {
    static_assert(sizeof(gnss_timestamp_header) == 10,
                  "gnss_timestamp_header has wrong packed size");
  }
  int      last_gps_solution() const { return _last_gps_solution; }
  uint32_t gpssec()            const { return _gpssec; }
  uint32_t gpsnsec()           const { return _gpsnsec; }

private:
  uint8_t  _last_gps_solution;
  uint8_t  _dummy;
  uint32_t _gpssec;
  uint32_t _gpsnsec;
} __attribute__((__packed__)) ;


class kiwisdr_impl : public kiwisdr
{
private:
  boost::asio::io_context         _ioc;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> _work;
  boost::asio::io_context::strand _strand;
  tcp::resolver                   _resolver;
  websocket::stream<tcp::socket>  _ws;

  gr::thread::condition_variable _ws_cond_wait;
  gr::thread::mutex              _ws_mutex;
  gr::thread::thread             _ws_thread;

  boost::beast::flat_buffer      _ws_buffer;
  std::vector<std::uint8_t>      _snd_buffer;

  std::deque<std::string>       _ws_write_queue;

  bool _connected;
  int  _keepalive_counter;

  std::string _host;
  std::string _port;

  std::string _client_type;
  std::string _password;

  std::map<std::string, std::string> _msg;
  // double _audio_rate;
  // double _sample_rate;

  std::map<const std::string, boost::format> _fmt;

  struct rx_parameters {
    std::string modulation;
    int         low_cut_Hz;
    int         high_cut_Hz;
    double      freq_kHz;
  } ;
  rx_parameters _rx_parameters;

  bool _iq_mode;

public:
  kiwisdr_impl(const std::string& host,
               const std::string& port);
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
  virtual void set_rx_parameters(const std::string& mode,
                                 int low_cut_Hz,
                                 int high_cut_Hz,
                                 double freq_kHz);

private:
  boost::shared_ptr<kiwisdr_impl> impl_shared_from_this() {
    return boost::dynamic_pointer_cast<kiwisdr_impl>(shared_from_this());
  }
  virtual bool connect(const std::string& host,
                       const std::string& port);

  void disconnect();

  // returns /{unix time seconds}/what
  std::string make_uri(const std::string& what) const;

  // async resolve
  void async_resolve();

  // callback for async resolve
  void on_resolve(const boost::system::error_code& ec,
                  const tcp::resolver::results_type& results);

  // callback for async connect
  void on_connect(const boost::system::error_code& ec);

  // callback for async handshake
  void on_handshake(const boost::system::error_code& ec);

  // async read
  void ws_async_read();

  // consume _ws_buffer and convert to std::string
  // len==-1 -> consume all bytes in the buffer
  std::string consume_buffer_as_string(int len=-1);

  // callback for ws_async_read
  void on_read(const boost::system::error_code& ec,
               std::size_t bytes_transferred);

  // decodes and processes kiwisdr messages
  void on_message(const std::string& payload);

  // sync send message
  void ws_write(const std::string& msg);

  // async send message
  void ws_async_write(const std::string& msg);

  // callback for ws_async_write
  void on_write(const boost::system::error_code& ec,
                std::size_t bytes_transferred);

  // async close
  void ws_async_close();

  // callback for ws_async_close
  void on_close(const boost::system::error_code& ec);

};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_KIWISDR_IMPL_H */
