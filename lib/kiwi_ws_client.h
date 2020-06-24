/* -*- c++ -*- */
/*
 * Copyright 2018-2020 Christoph Mayer hcab14@gmail.com.
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

#ifndef INCLUDED_KIWISDR_KIWI_WS_CLIENT_H
#define INCLUDED_KIWISDR_KIWI_WS_CLIENT_H

//#define BOOST_ASIO_ENABLE_HANDLER_TRACKING

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/format.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <vector>
#include <deque>
#include <queue>

#include <zmq.hpp>

#include <gnuradio/thread/thread.h>

#include "kiwi_rx_parameters.h"

using tcp = boost::asio::ip::tcp;
namespace websocket = boost::beast::websocket;

namespace gr {
namespace kiwisdr {

class kiwi_ws_client : public std::enable_shared_from_this<kiwi_ws_client> {
public:
  kiwi_ws_client(std::weak_ptr<zmq::socket_t> pub);
  virtual ~kiwi_ws_client();

  typedef std::queue<std::vector<std::uint8_t> > ws_data_queue_type;

  // change rx parameters
  void set_rx_parameters(kiwi_rx_parameters const&);

  bool connect(std::string const &host,
               std::string const &port,
               kiwi_rx_parameters const&);

  void disconnect();

  gr::thread::mutex &mutex() { return _mutex; }

protected:
  // returns /{unix time seconds}/what
  std::string make_uri(std::string const &what) const;

  // async resolve
  void async_resolve();

  // callback for async resolve
  void on_resolve(boost::system::error_code const &ec,
                  tcp::resolver::results_type const &results);

  // callback for async connect
  void on_connect(boost::system::error_code const &ec);

  // callback for async handshake
  void on_handshake(const boost::system::error_code& ec);

  // async read
  void ws_async_read();

  // consume _ws_buffer and convert to std::string
  // len==-1 -> consume all bytes in the buffer
  std::string consume_buffer_as_string(int len=-1);

  // callback for ws_async_read
  void on_read(boost::system::error_code const &ec,
               std::size_t bytes_transferred);

  // decodes and processes kiwisdr messages
  void on_message(std::string const &payload);

  // sync send message
  void ws_write(std::string const &msg);

  // async send message
  void ws_async_write(std::string const &msg);

  // callback for ws_async_write
  void on_write(boost::system::error_code const &ec,
                std::size_t bytes_transferred);

  // async close
  void ws_async_close();

  // // callback for ws_async_close
  // void on_close(boost::system::error_code const &ec);

private:
  boost::asio::io_context          _ioc;
  gr::thread::mutex                _mutex;
  boost::asio::io_context::strand _strand;
  boost::asio::deadline_timer     _deadline_timer_read;
  boost::asio::deadline_timer     _deadline_timer_close;
  tcp::resolver                   _resolver;
  websocket::stream<tcp::socket>  _ws;
  gr::thread::thread              _ws_thread;
  boost::beast::flat_buffer       _ws_buffer;
  std::deque<std::string>         _ws_write_queue;
  std::atomic<bool>               _connected;
  std::atomic<bool>               _disconnecting;
  size_t                          _keepalive_counter;

  std::string _host;
  std::string _port;

  kiwi_rx_parameters _rx_parameters;

  std::string _client_type;
  std::string _password;

  std::map<std::string, std::string> _msg;

  std::map<std::string const, boost::format> _fmt;

  std::weak_ptr<zmq::socket_t> _zmq_pub;
} ;

} // namespace kiwisdr
} // namespace gr

#endif // INCLUDED_KIWISDR_KIWI_WS_CLIENT_H

