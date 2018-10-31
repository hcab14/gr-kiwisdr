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

#include <regex>

#include "kiwi_ws_client.h"

namespace gr {
namespace kiwisdr {

kiwi_ws_client::kiwi_ws_client()
  : _ioc{1}
  , _mutex()
  , _strand(_ioc)
  , _deadline_timer_read(_ioc)
  , _deadline_timer_close(_ioc)
  , _resolver(_ioc)
  , _ws(_ioc)
  , _ws_cond_wait_data()
  , _ws_thread()
  , _ws_buffer()
  , _ws_data_queue()
  , _ws_write_queue()
  , _connected(false)
  , _disconnecting(false)
  , _keepalive_counter(0)
  , _host()
  , _port()
  , _client_type("kiwi")
  , _password()
  , _rx_parameters{10e3, -6000, 6000}
  , _msg()
  , _fmt{{"auth",        boost::format("SET auth t=%s p=%s")},
         {"ident_user",  boost::format("SET ident_user=%s")},
         {"geo",         boost::format("SET geo=%s")},
         {"inactivity_timeout", boost::format("SET OVERRIDE inactivity_timeout=%d")},
         {"keepalive",   boost::format("SET keepalive")},
         {"mod",         boost::format("SET mod=%s low_cut=%d high_cut=%d freq=%.3f")},
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
         {"wf_speed",    boost::format("SET wf_speed=%d")}}
{
  // empty
}

kiwi_ws_client::~kiwi_ws_client() {
  if (_connected)
    disconnect();
}

void kiwi_ws_client::set_rx_parameters(kiwi_rx_parameters const& rx_parameters) {
  _rx_parameters = rx_parameters;
  boost::asio::post
    (_strand,
     std::bind(
       &kiwi_ws_client::ws_async_write,
       shared_from_this(),
       str(_fmt["mod"]
           % "iq"
           % _rx_parameters.low_cut_Hz
           % _rx_parameters.high_cut_Hz
           % _rx_parameters.freq_kHz)));
}

bool kiwi_ws_client::connect(std::string const &host,
                             std::string const &port,
                             kiwi_rx_parameters const& rx_parameters) {
  _host = host;
  _port = port;
  _rx_parameters = rx_parameters;

  boost::asio::post(_strand,
                    std::bind
                    (&kiwi_ws_client::async_resolve,
                     shared_from_this()));

  auto self = shared_from_this();
  _ws_thread = gr::thread::thread([this, self]() { _ioc.run(); });
  return true;
}

void kiwi_ws_client::disconnect() {
  if (!_connected) {
    _disconnecting = true;
    auto self = shared_from_this();
    boost::asio::post
      (_strand, [this, self]() {
        _deadline_timer_read.expires_at(boost::date_time::neg_infin);
        _deadline_timer_close.expires_at(boost::date_time::neg_infin);
        boost::system::error_code ec_ignore{};
        _ws.lowest_layer().cancel(ec_ignore);
        _ws.lowest_layer().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec_ignore);
        _ws.lowest_layer().close(ec_ignore);
        _resolver.cancel();
      });
  } else {
    _connected = false;
    ws_async_close();
  }
  _ws_thread.join();
}

std::string kiwi_ws_client::make_uri(std::string const &what) const {
  auto const tnow = std::chrono::system_clock::now();
  time_t const t0 = std::chrono::system_clock::to_time_t(tnow);
  return "/" + std::to_string(t0) + "/" + what;
}

void kiwi_ws_client::async_resolve() {
  if (_disconnecting)
    return;
  _resolver.async_resolve(_host,
                          _port,
                          boost::asio::bind_executor(
                            _strand,
                            std::bind(
                              &kiwi_ws_client::on_resolve,
                              shared_from_this(),
                              std::placeholders::_1,
                              std::placeholders::_2)));
}
void kiwi_ws_client::on_resolve(boost::system::error_code const &ec,
                              tcp::resolver::results_type const &results) {
  if (_disconnecting)
    return;
  if(ec) {
    // TODO: error handling
    return;
  }
  // Make the connection on the IP address we get from a lookup
  boost::asio::async_connect(_ws.lowest_layer(),
                             results.begin(),
                             results.end(),
                             boost::asio::bind_executor(
                               _strand,
                               std::bind(
                                 &kiwi_ws_client::on_connect,
                                 shared_from_this(),
                                 std::placeholders::_1)));
}
void kiwi_ws_client::on_connect(boost::system::error_code const &ec) {
  if (_disconnecting)
    return;
  if (ec) {
    // TODO: error handling
    return;
  }
  _ws.async_handshake(_host,
                      make_uri("SND"),
                      boost::asio::bind_executor(
                        _strand,
                        std::bind(
                          &kiwi_ws_client::on_handshake,
                          shared_from_this(),
                          std::placeholders::_1)));
}

void kiwi_ws_client::on_handshake(boost::system::error_code const &ec) {
  if (ec) {
    // TODO: error handling
    return;
  }

  ws_async_write(str(_fmt.at("auth") % _client_type % _password));
  ws_async_write(str(_fmt.at("ident_user") % "gr-kiwisdr"));
  ws_async_write(str(_fmt.at("geo") % "geo"));

  _connected = true;
  ws_async_read();
}

void kiwi_ws_client::ws_async_read() {
  _deadline_timer_read.expires_from_now(boost::posix_time::seconds(5));
  auto self = shared_from_this();
  _deadline_timer_read.async_wait([this, self](boost::system::error_code const &ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;

      if (_deadline_timer_read.expires_at() > boost::asio::deadline_timer::traits_type::now()) {
        return;
      }
      if (_ws.lowest_layer().is_open()) {
        boost::system::error_code ec_ignore{};
        _ws.lowest_layer().cancel(ec_ignore);
      }
    });

  _ws.async_read(_ws_buffer,
                 boost::asio::bind_executor(
                   _strand,
                   std::bind(
                     &kiwi_ws_client::on_read,
                     shared_from_this(),
                     std::placeholders::_1,
                     std::placeholders::_2)));
}

inline std::string to_string(boost::beast::flat_buffer const &buffer, int len=-1)
{
  return std::string(boost::asio::buffer_cast<char const*>(
                       boost::beast::buffers_front(buffer.data())),
                     (len < 0
                      ? boost::asio::buffer_size(buffer.data())
                      : len));
}

// consumes len byts of a flat_buffer and returns them as a string
// len==-1 -> all bytes are retured as a string
std::string kiwi_ws_client::consume_buffer_as_string(int len) {
  std::string const s = to_string(_ws_buffer, len);
  _ws_buffer.consume(len);
  return s;
}

void kiwi_ws_client::on_read(boost::system::error_code const &ec,
                           std::size_t bytes_transferred) {

  _deadline_timer_read.expires_at(boost::posix_time::pos_infin);

  if (ec == boost::beast::websocket::error::closed) {
    _deadline_timer_read.cancel();
    if (_ws.lowest_layer().is_open()) {
      boost::system::error_code ec_ignore{};
      _ws.lowest_layer().cancel(ec_ignore);
    }
    _connected = false;
    return;
  }
  if (ec) {
    return;
  }
  std::string const type = consume_buffer_as_string(3);
  if (type == "MSG") {
    on_message(consume_buffer_as_string());
  } else if (type == "SND") {
    gr::thread::scoped_lock lock(_mutex);
    size_t const n_bytes = boost::asio::buffer_size(_ws_buffer.data());

    auto const p0 = boost::asio::buffer_cast<uint8_t const*>(boost::beast::buffers_front(_ws_buffer.data()));
    std::vector<uint8_t> snd_buffer(n_bytes);
    std::copy(p0, p0+n_bytes, snd_buffer.begin());
    _ws_data_queue.emplace(snd_buffer);

    _ws_cond_wait_data.notify_one();
    _ws_buffer.consume(n_bytes);
  } else {
  }

  // make sure we consumed all bytes in the buffer
  assert(boost::asio::buffer_size(_ws_buffer.data()) == 0);

  if (!_connected) {
    // ws_async_close();
    return;
  }

  // send a keepalive message every 10 frames
  _keepalive_counter += 1;
  if (_keepalive_counter == 10 && _connected) {
    ws_async_write(str(_fmt["keepalive"]));
    _keepalive_counter = 0;
  }

  // get the next packet
  ws_async_read();
}

std::map<std::string, std::string>
parse_key_value(std::string const &s) {
  std::regex const re(
    "([^\\s]+)" // non-whitespace
    "="         // literal '='
    "([^\\s]+)" // non-whitespace
    "[\\s]*"    // zero or more whitespaces
    );
  std::map<std::string, std::string> key_value_map;
  for (auto i = std::sregex_iterator(s.begin(), s.end(), re),
          end = std::sregex_iterator(); i != end; ++i) {
    assert(i->size() == 3);
    key_value_map[(*i)[1]] = (*i)[2];
  }
  return key_value_map;
}

void kiwi_ws_client::on_message(std::string const &payload) {
  // payload consists of key=value pairs, e.g.,
  // "MSG audio_init=0 audio_rate=12000 sample_rate=12001.150"
  //
  // client_public_ip=178.195.198.48
  // rx_chans=4
  // chan_no_pwd=0
  // badp=0
  // version_maj=1 version_min=216
  // load_cfg=...
  // center_freq=15000000 bandwidth=30000000 adc_clk_nom=66666600
  // audio_init=0 audio_rate=12000 sample_rate=12001.150
  for (auto const kv : parse_key_value(payload)) {
    // save the key-value pair
    _msg[kv.first] = kv.second;
    // check if password is OK
    if (kv.first == "badp" && kv.second == "1")
      throw std::runtime_error("bad password");
    if (kv.first == "audio_rate") {
      ws_async_write(str(_fmt["AR"] % std::stoi(kv.second) % 44100));

    } else if (kv.first == "sample_rate") {
      ws_async_write(str(_fmt["squelch"]       % 0 % 0));
      ws_async_write(str(_fmt["lms_autonotch"] % 0));
      ws_async_write(str(_fmt["genattn"]       % 0));
      ws_async_write(str(_fmt["gen"]           % 0 % 0));
      ws_async_write(str(_fmt["compression"]   % 0));
      ws_async_write(str(_fmt["mod"]
                   % "iq"
                   % _rx_parameters.low_cut_Hz
                   % _rx_parameters.high_cut_Hz
                   % _rx_parameters.freq_kHz));
      ws_async_write(str(_fmt["agc"]
                   % true % false % -100 % 6 % 1000 % 50));
      ws_async_write(str(_fmt["keepalive"]));
    } else {
    }
  }
}

void kiwi_ws_client::ws_write(std::string const &msg) {
  boost::system::error_code ec{};
  size_t const n = _ws.write(boost::asio::buffer(msg), ec);
}

void kiwi_ws_client::ws_async_write(std::string const &msg) {
  bool const not_write_in_progress = _ws_write_queue.empty();
  _ws_write_queue.push_back(msg);

  if (not_write_in_progress) {
    _ws.async_write(boost::asio::buffer(
                      _ws_write_queue.front().data(),
                      _ws_write_queue.front().length()),
                    boost::asio::bind_executor(
                      _strand,
                      std::bind(
                        &kiwi_ws_client::on_write,
                        shared_from_this(),
                        std::placeholders::_1,
                        std::placeholders::_2)));
  }
}


void kiwi_ws_client::on_write(boost::system::error_code const &ec,
                            std::size_t bytes_transferred) {
  gr::thread::scoped_lock lock(_mutex);
  if (!_connected) {
    _ws_write_queue.clear();
    return;
  }
  _ws_write_queue.pop_front();
  if (!_ws_write_queue.empty()) {
    _ws.async_write(boost::asio::buffer(
                      _ws_write_queue.front().data(),
                      _ws_write_queue.front().length()),
                    boost::asio::bind_executor(
                      _strand,
                      std::bind(&kiwi_ws_client::on_write,
                                shared_from_this(),
                                std::placeholders::_1,
                                std::placeholders::_2)));
  }
}

void kiwi_ws_client::ws_async_close() {
  auto self = shared_from_this();
  _deadline_timer_close.expires_from_now(boost::posix_time::seconds(10));
  _deadline_timer_close.async_wait([this, self](boost::system::error_code const &ec) {
      if (ec == boost::asio::error::operation_aborted)
        return;

      if (_ws.lowest_layer().is_open()) {
        boost::system::error_code ec_ignore{};
        _ws.lowest_layer().cancel(ec_ignore);
        _ws.lowest_layer().shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec_ignore);
        _ws.lowest_layer().close(ec_ignore);
      }
    });

  auto self2 = shared_from_this();
  _ws.async_close(
    websocket::close_code::normal,
    [this, self2](boost::system::error_code const &ec) {
      _deadline_timer_read.expires_at(boost::date_time::neg_infin);
      _deadline_timer_close.expires_at(boost::date_time::neg_infin);
    });
}

} // namespace kiwisdr
} // namespace gr
