/* -*- c++ -*- */
/*
 * Copyright 2018 hcab14@gmail.com.
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

#ifndef INCLUDED_KIWISDR_KIWI_WAV_SOURCE_IMPL_H
#define INCLUDED_KIWISDR_KIWI_WAV_SOURCE_IMPL_H

#include <fstream>
#include <kiwisdr/kiwi_wav_source.h>

#if defined(__GNUC__) && not defined(__MINGW32__)
#  define _PACKED __attribute__((__packed__))
#else
#  define _PACKED
#  define _USE_PRAGMA_PACK
#endif

namespace gr {
namespace kiwisdr {
namespace wav {
#ifdef _USE_PRAGMA_PACK
#  pragma pack(push, 1)
#endif

class chunk_base {
public:
  chunk_base()
    : _id()
    , _size() {
    static_assert(sizeof(chunk_base) == 8, "chunk_base has wrong packed size");
  }
  std::string id() const { return std::string((char*)(_id), 4); }
  std::streampos size() const { return _size; }
private:
  int8_t   _id[4];
  uint32_t _size;
} _PACKED;

class chunk_riff : public chunk_base {
public:
  chunk_riff()
    : _format() {
    static_assert(sizeof(chunk_riff) == 8+4, "chunk_riff has wrong packed size");
  }
  std::string format() const { return std::string((char*)(_format), 4); }

private:
  int8_t _format[4];
} _PACKED;

class chunk_fmt : public chunk_base {
public:
  chunk_fmt()
    : _format()
    , _num_channels()
    , _sample_rate()
    , _byte_rate()
    , _block_align()
    , _dummy() {
    static_assert(sizeof(chunk_fmt) == 8+16, "chunk_fmt has wrong packed size");
  }
  uint16_t format()       const { return _format; }
  uint16_t num_channels() const { return _num_channels; }
  uint32_t sample_rate()  const { return _sample_rate; }
  uint32_t byte_rate()    const { return _byte_rate; }
  uint16_t block_align()  const { return _block_align; }

protected:
  uint16_t _format;
  uint16_t _num_channels;
  uint32_t _sample_rate;
  uint32_t _byte_rate;
  uint16_t _block_align;
  uint16_t _dummy;
} _PACKED;

class chunk_kiwi : public chunk_base {
public:
  chunk_kiwi()
    : _last()
    , _dummy()
    , _gpssec()
    , _gpsnsec() {
    static_assert(sizeof(chunk_kiwi) == 8+10, "chunk_kiwi has wrong packed size");
  }
  int      last_gnss_solution() const { return _last; }
  uint32_t gpssec() const { return _gpssec; }
  uint32_t gpsnsec() const { return _gpsnsec; }
  double   as_double() const { return _gpssec+1e-9*_gpsnsec; }
private:
  uint8_t  _last, _dummy;
  uint32_t _gpssec, _gpsnsec;
} _PACKED;

#ifdef _USE_PRAGMA_PACK
#  pragma pack(pop)
#  undef _USE_PRAGMA_PACK
#endif

} // namespace wav

static const pmt::pmt_t TIME_KEY      = pmt::string_to_symbol("rx_time"); // taken from gr-uhd/lib/usrp_source_impl.cc
static const pmt::pmt_t SAMP_RATE_KEY = pmt::string_to_symbol("samp_rate");

class kiwi_wav_source_impl : public kiwi_wav_source
{
private:
  std::string _filename;
  std::shared_ptr<std::ifstream> _stream;
  std::streampos  _pos;
  wav::chunk_kiwi _kiwi_chunk;
  wav::chunk_kiwi _last_kiwi_chunk;
  std::uint64_t   _sample_counter;
  std::uint64_t   _chunk_counter;
  std::uint64_t   _num_samples_in_chunk;
  double          _last_gnss_time;
  bool            _use_new_gnss_solution;
  pmt::pmt_t _id;

  gr_complex read_sample(bool& eof);
  gr_complex get_next_sample(bool& kiwi_chunk, bool& eof);

public:
  kiwi_wav_source_impl(std::string filename);
  virtual ~kiwi_wav_source_impl();

  virtual bool start();
  virtual bool stop();

  // Where all the action really happens
  int work(int noutput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);
};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_KIWI_WAV_SOURCE_IMPL_H */
