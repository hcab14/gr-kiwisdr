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
  , _ioc()
  , _strand{_ioc}
  , _ws{_ioc}
  , _ws_cond_wait()
  , _ws_mutex()
  , _ws_thread()
  , _ws_buffer()
  , _snd_buffer()
  , _ws_write_queue()
  , _connected(false)
  , _host(host)
  , _port(port)
  , _client_type("kiwi")
  , _password()
  , _msg()
  , _rx_parameters{"iq",-6000,6000,15040}
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
  boost::unique_lock<boost::mutex> lock(_ws_mutex);

  if (!_ws_cond_wait.timed_wait(lock, boost::posix_time::milliseconds(10))) // timeout
    return 0;

  // (1) decode snd_info_header
  snd_info_header snd_info;
  std::memcpy(&snd_info, &_snd_buffer[0], sizeof(snd_info));
  int header_length = sizeof(snd_info);
  // std::cout << "SND: seq= " << snd_info.seq() << " RSSI= " << snd_info.rssi() << "\n";

  // (2) decode gnss timestamp header (IQ mode only)
  if (_iq_mode) {
    gnss_timestamp_header gnss_timestamp;
    std::memcpy(&gnss_timestamp, &_snd_buffer[sizeof(snd_info_header)], sizeof(gnss_timestamp));
    header_length += sizeof(gnss_timestamp_header);
    // std::cout << "SND: gpssec= " << gnss_timestamp.gpssec()
    //           << " gpsnsec= " << gnss_timestamp.gpsnsec()
    //           << " last_gnss_solution= " << gnss_timestamp.last_gps_solution() << " | ";

    // std::copy(_snd_buffer.begin() + sizeof(snd_info_header),
    //           _snd_buffer.begin() + sizeof(snd_info_header) + sizeof(gnss_timestamp_header),
    //           std::ostream_iterator<int>(std::cout, " "));
    // std::cout << std::endl;
  }
  assert(_snd_buffer.size() > header_length);

  const auto n_samples = (_snd_buffer.size() - header_length) >> (_iq_mode ? 2 : 1);
  assert(n_samples <= noutput_items);
  assert((_snd_buffer.size() - header_length) % 2 == 0);
  noutput_items = (_snd_buffer.size() - header_length) >> 1;

  // std::cout << "SND: n_samples = " << noutput_items << " " << _snd_buffer.size() - header_length << std::endl;


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

void kiwisdr_impl::ws_write(const std::string& msg) {
  boost::system::error_code ec;
  const size_t n = _ws.write(boost::asio::buffer(msg), ec);
  std::cout << "ws_write '" << msg << "' n= " << n << " ec= " << ec << std::endl;
}

inline std::string to_string(boost::beast::flat_buffer const& buffer, int len=-1)
{
  return std::string(boost::asio::buffer_cast<char const*>(
                       boost::beast::buffers_front(buffer.data())),
                     (len < 0
                      ? boost::asio::buffer_size(buffer.data())
                      : len));
}

std::string kiwisdr_impl::consume_buffer_as_string(int len) {
  const std::string s = to_string(_ws_buffer, len);
  _ws_buffer.consume(len);
  return s;
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
  for (auto i=std::sregex_iterator(s.begin(), s.end(), re),
         end=std::sregex_iterator(); i != end; ++i) {
    assert(i->size() == 3);
    key_value_map[(*i)[1]] = (*i)[2];
  }
  return key_value_map;
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
  _ws.handshake(_host, make_uri("SND"));

  ws_write(str(_fmt.at("auth") % _client_type % _password));
  ws_write(str(_fmt.at("ident_user") % "gr-kiwisdr"));
  ws_write(str(_fmt.at("geo") % "geo"));

  _ws_thread = gr::thread::thread(boost::bind(&kiwisdr_impl::run_io_context, this));
  _connected = true;
  ws_async_read();

  std::cout << "connected" << std::endl;


  return true;
}

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

void kiwisdr_impl::set_rx_parameters(const std::string& mode,
                                     int low_cut_Hz,
                                     int high_cut_Hz,
                                     double freq_kHz) {
  gr::thread::scoped_lock lock(d_setlock);
  _rx_parameters = rx_parameters{mode, low_cut_Hz, high_cut_Hz, freq_kHz};
  _iq_mode       = (mode == "iq");
  // TODO: _ioc.post(...);
  std::cout << "#################### set_rx_parameters "
            << mode << " "
            << low_cut_Hz << " "
            << high_cut_Hz << " "
            << freq_kHz << "####################" << std::endl;
  _strand.post(boost::bind(&kiwisdr_impl::ws_async_write,
                           this,
                           str(_fmt["mod"] % mode % low_cut_Hz % high_cut_Hz % freq_kHz)));
//  ws_async_write(str(_fmt["mod"] % mode % low_cut_Hz % high_cut_Hz % freq_kHz));
}

std::string kiwisdr_impl::make_uri(const std::string& what) const {
  auto const tnow = std::chrono::system_clock::now();
  const time_t t0 = std::chrono::system_clock::to_time_t(tnow);
  return "/" + boost::lexical_cast<std::string>(t0) + "/" + what;
}

void kiwisdr_impl::ws_async_read() {
//  std::cout << "ws_async_read\n";
  _ws.async_read(_ws_buffer,
                 boost::asio::bind_executor(_strand,
                                            boost::bind(&kiwisdr_impl::on_read,
                                                        this, // shared_from_this() returns sptr of kiwisdr instead of kiwisdr_impl!
                                                        boost::asio::placeholders::error,
                                                        boost::asio::placeholders::bytes_transferred)));
}

void kiwisdr_impl::ws_async_write(const std::string& msg) {
  std::cout << "ws_async_write '" << msg << "'\n";
  const bool not_write_in_progress = _ws_write_queue.empty();

  _ws_write_queue.push_back(msg);

  if (not_write_in_progress) {
    _ws.async_write(boost::asio::buffer(_ws_write_queue.front().data(),
                                        _ws_write_queue.front().length()),
                    boost::asio::bind_executor(_strand,
                                               boost::bind(&kiwisdr_impl::on_write,
                                                           this, // shared_from_this() returns sptr of kiwisdr instead of kiwisdr_impl!
                                                           boost::asio::placeholders::error,
                                                           boost::asio::placeholders::bytes_transferred)));
  }
}


void kiwisdr_impl::on_write(boost::system::error_code ec,
                           std::size_t bytes_transferred) {
  std::cout << "on_write: bytes_transferred= " << bytes_transferred << " ec= " << ec << std::endl;
  gr::thread::scoped_lock lock(d_setlock);
  _ws_write_queue.pop_front();
  if (!_ws_write_queue.empty()) {
    _ws.async_write(boost::asio::buffer(_ws_write_queue.front().data(),
                                        _ws_write_queue.front().length()),
                    boost::asio::bind_executor(_strand,
                                               boost::bind(&kiwisdr_impl::on_write,
                                                           this, // shared_from_this() returns sptr of kiwisdr instead of kiwisdr_impl!
                                                           boost::asio::placeholders::error,
                                                           boost::asio::placeholders::bytes_transferred)));
  }
}


void kiwisdr_impl::on_read(boost::system::error_code ec,
                           std::size_t bytes_transferred) {
//  std::cout << "on_read: bytes_transferred= " << bytes_transferred << " ec= " << ec << "'\n";

  const std::string type = consume_buffer_as_string(3);
  if (type == "MSG") {
    on_message(consume_buffer_as_string());
  } else if (type == "SND") {
    boost::lock_guard<gr::thread::mutex> lock(_ws_mutex);

    const size_t n_bytes = boost::asio::buffer_size(_ws_buffer.data());
    _snd_buffer.resize(n_bytes);

    auto p0 = boost::asio::buffer_cast<uint8_t const*>(boost::beast::buffers_front(_ws_buffer.data()));
    std::copy(p0, p0+n_bytes, _snd_buffer.begin());

    _ws_buffer.consume(n_bytes);
    _ws_cond_wait.notify_one();
  } else {
    std::cout << "on_read: unknwon type '" << type << "'\n";
  }

  assert(boost::asio::buffer_size(_ws_buffer.data()) == 0);

  // to be removed later on
  //_ws_buffer.consume(bytes_transferred);

  ws_async_read();
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
