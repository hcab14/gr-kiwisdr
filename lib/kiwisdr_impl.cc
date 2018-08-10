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

#include <boost/asio/placeholders.hpp>

#include <gnuradio/io_signature.h>

#include "kiwisdr_impl.h"

namespace gr {
namespace kiwisdr {

kiwisdr::sptr
kiwisdr::make(const std::string& host,
              const std::string& port)
{
  return gnuradio::get_initial_sptr
    (new kiwisdr_impl(host, port));
}

/*
 * The private constructor
 */
kiwisdr_impl::kiwisdr_impl(const std::string& host,
                           const std::string& port)
  : gr::block("kiwisdr",
              gr::io_signature::make(0, 0, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
  , _ioc()
  , _ws(_ioc)
  , _ws_cond_wait()
  , _ws_mutex()
  , _ws_thread()
  , _ws_buffer()
  , _connected(false)
  , _host(host)
  , _port(port)
  , _client_type("kiwi")
  , _password()
  , _fmt{{"auth",        boost::format("SET auth t=%s p=%s")},
         {"ident_user",  boost::format("SET auth t=%s p=%s")},
         {"geo",         boost::format("SET geo=%s")},
         {"inactivity_timeout", boost::format("SET OVERRIDE inactivity_timeout=%d")},
         {"keepalive",   boost::format("SET keepalive")},
         {"mod",         boost::format("mod=%s low_cut=%d high_cut=%d freq=%.3f")},
         {"agc",         boost::format("SET agc=%d hang=%d thresh=%d slope=%d decay=%d manGain=%d")},
         {"squelch",     boost::format("SET squelch=%d max=%d")},
         {"lms_autonotch", boost::format("SET lms_autonotch=%d")},
         {"AR",          boost::format("SET AR OK in=%d out=%d")},
         {"genattn",     boost::format("SET genattn=%d")},
         {"gen",         boost::format("SET gen=%d mix=%d")},
         {"zoom",        boost::format("SET zoom=%d start=%f")},
         {"maxdb",       boost::format("SET maxdb=%d mindb=%d")},
         {"compression", boost::format("SET compression=%d")},
         {"wf_comp",     boost::format("SET wf_comp=%d")},
         {"wf_speed",    boost::format("SET wf_speed=%d")}},
  _mod_info{{"am",  {-2500, 2500, 10e3}},
            {"iq",  {-6000, 6000, 10e3}}},
  _current_mod("iq")
{
  std::cout << "args= " << host << ", " << port << std::endl;
}

/*
 * Our virtual destructor.
 */
kiwisdr_impl::~kiwisdr_impl()
{
  std::cout << "kiwisdr_impl::~kiwisdr_impl\n";
  if (_connected)
    disconnect();
}

int
kiwisdr_impl::general_work(int noutput_items,
                           gr_vector_int& ninput_items,
                           gr_vector_const_void_star& input_items,
                           gr_vector_void_star& output_items)
{
  // <+OTYPE+> *out = (<+OTYPE+> *) output_items[0];

  // Do <+signal processing+>

  noutput_items = 0;

  // Tell runtime system how many output items we produced.
  return noutput_items;
}

void kiwisdr_impl::ws_write(const std::string& msg) {
  _ws.write(boost::asio::buffer(msg));
}

bool kiwisdr_impl::connect(const std::string& host,
                           const std::string& port) {
  _host = host;
  _port = port;
  std::cout << "connect: " << host << " " << port << "\n";

  // make a websocket stream object (NOTE: may be superfluous as this is done in the constructor)
  _ws = websocket::stream<tcp::socket>(_ioc);

  // resolve host,port
  tcp::resolver resolver{_ioc};
  auto const results = resolver.resolve(_host, _port);
  boost::asio::connect(_ws.next_layer(), results.begin(), results.end());

  // websocket handshake
  const std::string uri = make_uri("SND");
  std::cout << "connect: host='" << _host << "' uri='" << uri << "'\n";
  _ws.handshake(_host, uri);//make_uri("SND"));

  ws_write(str(_fmt.at("auth") % _client_type % _password));

  start_read();
  _ws_thread = gr::thread::thread(boost::bind(&kiwisdr_impl::run_io_context, this));
  _connected = true;

  std::cout << "connected\n";
  return true;}

void kiwisdr_impl::disconnect() {
  std::cout << "disconnect\n";
  gr::thread::scoped_lock lock(d_setlock);

  if (!_connected)
    return;

  std::cout << "disconnect 1\n";
//  _ws.close(websocket::close_code::normal); // ?? where
  std::cout << "disconnect 2\n";
  _ioc.reset();
  std::cout << "disconnect 3\n";
  _ioc.stop();
  std::cout << "disconnect 4\n";
  _ws_thread.join();
  std::cout << "disconnect 5\n";

  _connected = false;
}

std::string kiwisdr_impl::make_uri(const std::string& what) const {
  auto const tnow = std::chrono::system_clock::now();
  const time_t t0 = std::chrono::system_clock::to_time_t(tnow);
  return "/" + boost::lexical_cast<std::string>(t0) + "/" + what;
}

void kiwisdr_impl::start_read() {
  std::cout << "start_read\n";
  _ws.async_read(_ws_buffer,
                 boost::bind(&kiwisdr_impl::on_read,
                             this, // shared_from_this() returns sptr of kiwisdr instead of kiwisdr_impl!
                             boost::asio::placeholders::error,
                             boost::asio::placeholders::bytes_transferred));
}

inline std::string to_string(boost::beast::flat_buffer const& buffer)
{
  return std::string(boost::asio::buffer_cast<char const*>(
                       boost::beast::buffers_front(buffer.data())),
                     boost::asio::buffer_size(buffer.data()));
}

void kiwisdr_impl::on_read(boost::system::error_code ec,
                           std::size_t bytes_transferred) {
  std::cout << "on_read: bytes_transferred= " << bytes_transferred << " ec= " << ec
            << " '" << to_string(_ws_buffer) << "'\n";
  _ws_buffer.consume(bytes_transferred);
  start_read();
}
bool kiwisdr_impl::start() {
  if (!_connected)
    connect(_host, _port);
  auto const tnow = std::chrono::system_clock::now();
  const time_t t0 = std::chrono::system_clock::to_time_t(tnow);
  std::cout << "start " << t0 << "\n";
  return true;
}
bool kiwisdr_impl::stop() {
  std::cout << "stop\n";
  disconnect();
  return true;
}

} /* namespace kiwisdr */
} /* namespace gr */
