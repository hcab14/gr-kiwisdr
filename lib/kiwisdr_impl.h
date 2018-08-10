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

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
// avoid the warning message to switch from Boost.Coroutine to Boost.Coroutine2
#define BOOST_COROUTINES_NO_DEPRECATION_WARNING
#include <boost/asio/spawn.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <chrono>
#include <cstdlib>
#include <map>
#include <memory>

#include <gnuradio/thread/thread.h>

#include <kiwisdr/kiwisdr.h>


using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

namespace gr {
namespace kiwisdr {

class kiwisdr_impl : public kiwisdr
{
private:
  boost::asio::io_context        _ioc;
  websocket::stream<tcp::socket> _ws;

  gr::thread::condition_variable _ws_cond_wait;
  gr::thread::mutex              _ws_mutex;
  gr::thread::thread             _ws_thread;

  boost::beast::flat_buffer      _ws_buffer;

  bool _connected;

  std::string _host;
  std::string _port;

  std::string _client_type;
  std::string _password;

  std::map<const std::string, boost::format> _fmt;

  struct mod_par {
    int         low_cut_Hz;
    int         high_cut_Hz;
    double      freq_kHz;
  };
  std::map<const std::string, mod_par> _mod_info;

  std::string _current_mod;

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

  virtual bool connect(const std::string& host,
                       const std::string& port);

  void disconnect();

  std::string make_uri(const std::string& what) const;

  void run_io_context() { std::cout << "run_io_context\n"; _ioc.run(); }

  void start_read();
  void on_read(boost::system::error_code ec,
               std::size_t bytes_transferred);

  // sync send message
  void ws_write(const std::string& msg);

};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_KIWISDR_IMPL_H */
