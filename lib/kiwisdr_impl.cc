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
#include <regex>

#include <gnuradio/io_signature.h>
#include <volk/volk.h>

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
              gr::io_signature::make (0, 0, sizeof(gr_complex)),
              gr::io_signature::make2(1, 2, sizeof(gr_complex), sizeof(float)))
  , _ioc{}
  , _work(boost::asio::make_work_guard(_ioc))
  , _strand(_ioc)
  , _resolver(_ioc)
  , _ws(_ioc)
  , _ws_cond_wait_data()
  , _ws_cond_wait_close()
  , _ws_thread()
  , _ws_buffer()
  , _snd_buffer()
  , _ws_write_queue()
  , _connected(false)
  , _keepalive_counter(0)
  , _host(host)
  , _port(port)
  , _client_type("kiwi")
  , _password()
  , _msg()
  , _rx_parameters{"iq", -6000, 6000, 15040}
  , _iq_mode(_rx_parameters.modulation == "iq")
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
  // empty for now
}

/*
 * Our virtual destructor.
 */
kiwisdr_impl::~kiwisdr_impl()
{
  if (_connected)
    disconnect();
}

int kiwisdr_impl::general_work(int noutput_items,
                               gr_vector_int& ninput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star& output_items)
{
  if (!_connected) {
    usleep(10);
    return 0;
  }

  gr::thread::scoped_lock lock(d_setlock);
  if (!_ws_cond_wait_data.timed_wait(lock, boost::posix_time::milliseconds(10))) // timeout
    return 0;


  // (1) decode snd_info_header
  snd_info_header snd_info;
  std::memcpy(&snd_info, &_snd_buffer[0], sizeof(snd_info));
  int header_length = sizeof(snd_info);
  std::cout << "SND: seq= " << snd_info.seq() << " RSSI= " << snd_info.rssi() << "\n";

  // (2) decode gnss timestamp header (IQ mode only)
  if (_iq_mode) {
    gnss_timestamp_header gnss_timestamp;
    std::memcpy(&gnss_timestamp, &_snd_buffer[sizeof(snd_info_header)], sizeof(gnss_timestamp));
    header_length += sizeof(gnss_timestamp_header);
    std::cout << "SND: gpssec= " << gnss_timestamp.gpssec()
              << " gpsnsec= " << gnss_timestamp.gpsnsec()
              << " last_gnss_solution= " << gnss_timestamp.last_gps_solution() << " | ";

    std::copy(_snd_buffer.begin() + sizeof(snd_info_header),
              _snd_buffer.begin() + sizeof(snd_info_header) + sizeof(gnss_timestamp_header),
              std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;
  }
  assert(_snd_buffer.size() > header_length);

  const auto n_samples = (_snd_buffer.size() - header_length) >> (_iq_mode ? 2 : 1);
  assert(n_samples <= noutput_items);
  assert((_snd_buffer.size() - header_length) % 2 == 0);
  noutput_items = (_snd_buffer.size() - header_length) >> 1;

  // big-endian -> little endian conversion
  volk_16u_byteswap((uint16_t*)(&_snd_buffer[header_length]), noutput_items);

  // uint16_t -> float conversion
  float* out = (float*)(output_items[_iq_mode ? 0 : 1]);
  volk_16i_s32f_convert_32f(out, (const int16_t*)(&_snd_buffer[header_length]), float((1<<15)-1), noutput_items);
  if (_iq_mode) {
    assert(noutput_items %2 == 0);
    noutput_items >>= 1;
  }

  // Tell runtime system how many output items we produced.
  return noutput_items;
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
  if (!_connected) {
    return true;
  }
  disconnect();
  return true;
}

void kiwisdr_impl::set_rx_parameters(const std::string& mode,
                                     int low_cut_Hz,
                                     int high_cut_Hz,
                                     double freq_kHz) {
  gr::thread::scoped_lock lock(d_setlock);
  _rx_parameters = rx_parameters{mode, low_cut_Hz, high_cut_Hz, freq_kHz};
  _iq_mode       = (mode == "iq");
  boost::asio::post(_strand,
                    std::bind(
                      &kiwisdr_impl::ws_async_write,
                      impl_shared_from_this(),
                      str(_fmt["mod"] % mode % low_cut_Hz % high_cut_Hz % freq_kHz)));
}

bool kiwisdr_impl::connect(const std::string& host,
                           const std::string& port) {
  gr::thread::scoped_lock lock(d_setlock);
  _host = host;
  _port = port;

  boost::asio::post(_strand, std::bind(&kiwisdr_impl::async_resolve, impl_shared_from_this()));

  _ws_thread = gr::thread::thread(boost::bind(&boost::asio::io_context::run, &_ioc));
  return true;
}

void kiwisdr_impl::disconnect() {
  if (!_connected)
    return;

  gr::thread::scoped_lock lock(d_setlock);
  _connected = false;
  ws_async_close();

  if (!_ws_cond_wait_close.timed_wait(lock, boost::posix_time::seconds(5))) // timeout
    std::cout << "timeout while waiting for async_close\n";
}

std::string kiwisdr_impl::make_uri(const std::string& what) const {
  auto const tnow = std::chrono::system_clock::now();
  const time_t t0 = std::chrono::system_clock::to_time_t(tnow);
  return "/" + std::to_string(t0) + "/" + what;
}

void kiwisdr_impl::async_resolve() {
  _resolver.async_resolve(_host,
                          _port,
                          boost::asio::bind_executor(
                            _strand,
                            std::bind(
                              &kiwisdr_impl::on_resolve,
                              impl_shared_from_this(),
                              std::placeholders::_1,
                              std::placeholders::_2)));
}
void kiwisdr_impl::on_resolve(const boost::system::error_code& ec,
                              const tcp::resolver::results_type& results) {
  if(ec) {
    // TODO: error handling
    return;
  }
  // Make the connection on the IP address we get from a lookup
  boost::asio::async_connect(_ws.next_layer(),
                             results.begin(),
                             results.end(),
                             boost::asio::bind_executor(
                               _strand,
                               std::bind(
                                 &kiwisdr_impl::on_connect,
                                 impl_shared_from_this(),
                                 std::placeholders::_1)));
}
void kiwisdr_impl::on_connect(const boost::system::error_code& ec) {
  std::cout << "on_connect: " << ec << " " << ec.message()  << std::endl;
  if (ec) {
    // TODO: error handling
    return;
  }
  _ws.async_handshake(_host,
                      make_uri("SND"),
                      boost::asio::bind_executor(
                        _strand,
                        std::bind(
                          &kiwisdr_impl::on_handshake,
                          impl_shared_from_this(),
                          std::placeholders::_1)));
}

void kiwisdr_impl::on_handshake(const boost::system::error_code& ec) {
  std::cout << "on_handshake: " << ec << " " << ec.message()  << std::endl;
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

void kiwisdr_impl::ws_async_read() {
  std::cout << "ws_async_read\n";
  _ws.async_read(_ws_buffer,
                 boost::asio::bind_executor(
                   _strand,
                   std::bind(
                     &kiwisdr_impl::on_read,
                     impl_shared_from_this(),
                     std::placeholders::_1,
                     std::placeholders::_2)));
}

inline std::string to_string(boost::beast::flat_buffer const& buffer, int len=-1)
{
  return std::string(boost::asio::buffer_cast<char const*>(
                       boost::beast::buffers_front(buffer.data())),
                     (len < 0
                      ? boost::asio::buffer_size(buffer.data())
                      : len));
}

// consumes len byts of a flat_buffer and returns them as a string
// len==-1 -> all bytes are retured as a string
std::string kiwisdr_impl::consume_buffer_as_string(int len) {
  const std::string s = to_string(_ws_buffer, len);
  _ws_buffer.consume(len);
  return s;
}

void kiwisdr_impl::on_read(const boost::system::error_code& ec,
                           std::size_t bytes_transferred) {
  if (ec == boost::beast::websocket::error::closed) {
    std::cout << "on_read: boost::beast::websocket::error::closed received" << std::endl;
    _connected = false;
    return;
  }
  if (ec) {
    // TODO: error handling
    std::cout << "on_read: error " << ec << " " << ec.message() << std::endl;
    return;
  }
  const std::string type = consume_buffer_as_string(3);
  std::cout << "on_read: type='" << type << "'\n";
  if (type == "MSG") {
    on_message(consume_buffer_as_string());
  } else if (type == "SND") {
    gr::thread::scoped_lock lock(d_setlock);

    const size_t n_bytes = boost::asio::buffer_size(_ws_buffer.data());
    _snd_buffer.resize(n_bytes);

    auto p0 = boost::asio::buffer_cast<uint8_t const*>(boost::beast::buffers_front(_ws_buffer.data()));
    std::copy(p0, p0+n_bytes, _snd_buffer.begin());

    _ws_buffer.consume(n_bytes);
    _ws_cond_wait_data.notify_one();
  } else {
    std::cout << "on_read: unknwon type '" << type << "'\n";
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
parse_key_value(const std::string& s) {
  const std::regex re(
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

void kiwisdr_impl::on_message(const std::string& payload) {
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
      std::cout << "audio_rate '" << kv.second << "' " << std::stoi(kv.second) << std::endl;
      ws_async_write(str(_fmt["AR"] % std::stoi(kv.second) % 44100));
      //   _audio_rate = std::stod(kv.second);
    } else if (kv.first == "sample_rate") {
      ws_async_write(str(_fmt["squelch"]       % 0 % 0));
      ws_async_write(str(_fmt["lms_autonotch"] % 0));
      ws_async_write(str(_fmt["genattn"]       % 0));
      ws_async_write(str(_fmt["gen"]           % 0 % 0));
      ws_async_write(str(_fmt["compression"]   % 0));
      ws_async_write(str(_fmt["mod"]
                   % _rx_parameters.modulation
                   % _rx_parameters.low_cut_Hz
                   % _rx_parameters.high_cut_Hz
                   % _rx_parameters.freq_kHz));
      ws_async_write(str(_fmt["agc"]
                   % true % false % -100 % 6 % 1000 % 50));
      ws_async_write(str(_fmt["keepalive"]));
      //   _sample_rate = std::stod(kv.second);
    } else {
      std::cout << "message: " << kv.first << " " << kv.second << std::endl;
    }
  }
}

void kiwisdr_impl::ws_write(const std::string& msg) {
  boost::system::error_code ec;
  const size_t n = _ws.write(boost::asio::buffer(msg), ec);
  std::cout << "ws_write '" << msg << "' n= " << n << " ec= " << ec << " " << ec.message()  << std::endl;
}

void kiwisdr_impl::ws_async_write(const std::string& msg) {
  const bool not_write_in_progress = _ws_write_queue.empty();
  _ws_write_queue.push_back(msg);

  if (not_write_in_progress) {
    _ws.async_write(boost::asio::buffer(
					_ws_write_queue.front().data(),
                      _ws_write_queue.front().length()),
                    boost::asio::bind_executor(
                      _strand,
                      std::bind(
                        &kiwisdr_impl::on_write,
                        impl_shared_from_this(),
                        std::placeholders::_1,
                        std::placeholders::_2)));
  }
}


void kiwisdr_impl::on_write(const boost::system::error_code& ec,
                            std::size_t bytes_transferred) {
  std::cout << "on_write: bytes_transferred= " << bytes_transferred << " ec= " << ec << " " << ec.message()  << std::endl;
  gr::thread::scoped_lock lock(d_setlock);
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
                      std::bind(&kiwisdr_impl::on_write,
                                impl_shared_from_this(),
                                std::placeholders::_1,
                                std::placeholders::_2)));
  }
}

void kiwisdr_impl::ws_async_close() {
  std::cout << "ws_async_close" << std::endl;
  _ws.async_close(
    websocket::close_code::normal,
    boost::asio::bind_executor(
      _strand,
      std::bind(
        &kiwisdr_impl::on_close,
        impl_shared_from_this(),
        std::placeholders::_1)));
}

void kiwisdr_impl::on_close(const boost::system::error_code& ec) {
  std::cout << "on_close ec=" << ec << " " << ec.message()  << " " << ec.message() << std::endl;
  _ws_cond_wait_close.notify_one();
  // _ioc.reset();
  // _ioc.stop();
  // _ws_thread.join();
  std::cout << "on_close: finished\n";
}


} /* namespace kiwisdr */
} /* namespace gr */
