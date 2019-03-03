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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <boost/format.hpp>

#include <gnuradio/io_signature.h>
#include <gnuradio/logger.h>
#include "kiwi_wav_source_impl.h"

namespace gr {
namespace kiwisdr {

kiwi_wav_source::sptr
kiwi_wav_source::make(std::string filename)
{
  return gnuradio::get_initial_sptr
    (new kiwi_wav_source_impl(filename));
}

kiwi_wav_source_impl::kiwi_wav_source_impl(std::string filename)
  : gr::sync_block("kiwi_wav_source",
                   gr::io_signature::make(0, 0, 0),
                   gr::io_signature::make(1, 1, sizeof(gr_complex)))
  , _filename(filename)
  , _stream()
  , _pos()
  , _last_kiwi_chunk()
  , _counter(0)
  , _num_samples(0)
  , _id(pmt::mp(_filename))
{
  GR_LOG_DECLARE_LOGPTR(d_logger);
  GR_LOG_ASSIGN_LOGPTR(d_logger, "kiwi_wav_source@"+_filename);
}

kiwi_wav_source_impl::~kiwi_wav_source_impl()
{
}

bool kiwi_wav_source_impl::start()
{
  _counter = 0;
  _num_samples = 0;
  _stream = std::make_shared<std::ifstream>(_filename.c_str(), std::ios::binary);
  if (!_stream || !(*_stream)) {
    GR_LOG_ERROR(d_logger, str(boost::format("failed to open the file '%s'") % _filename));
    return false;
  }

  for (wav::chunk_base c; *_stream; ) {
    _pos = _stream->tellg();
    _stream->read((char*)(&c), sizeof(c));
    if (!(*_stream)) {
      GR_LOG_ERROR(d_logger, "unexpected end of file");
      return false;
    }

    if (c.id() == "RIFF") {
      wav::chunk_riff cr;
      _stream->seekg(_pos);
      _stream->read((char*)(&cr), sizeof(cr));
      if (!(*_stream)) {
        GR_LOG_ERROR(d_logger, "unexpected end of file");
        return false;
      } else if (cr.format() != "WAVE") {
        GR_LOG_ERROR(d_logger, str(boost::format("unsupported format: '%s'") % cr.format()));
        return false;
      }
      // int const n = (int(cr.size())-sizeof(chunk_riff)-4)/2074; // number of samples
    } else if (c.id() == "fmt ") {
      _stream->seekg(_pos);
      wav::chunk_fmt  fmt;
      _stream->read((char*)(&fmt), sizeof(fmt));
      if (!(*_stream)) {
        GR_LOG_ERROR(d_logger, "unexpected end of file");
        return false;
      } else if (fmt.format() != 1 || fmt.num_channels() != 2) {
        GR_LOG_ERROR(d_logger, str(boost::format("unsupported WAVE format: format=%d num_channels=%d")
                                   % c.id() % fmt.format() %  fmt.num_channels()));
        return false;
      }
      return true;
    } else {
      GR_LOG_WARN(d_logger, str(boost::format("skipping unknown chunk '%s' len=%d") % c.id() % c.size()));
      _stream->seekg(_stream->tellg() + c.size());
    }
  }
  return false;
}

bool kiwi_wav_source_impl::stop()
{
  _stream.reset();
  _pos = 0;
  _counter = 0;
  _num_samples = 0;
  return true;
}

gr_complex kiwi_wav_source_impl::read_sample(bool& eof) {
  int16_t i=0, q=0;
  _stream->read((char*)(&i), sizeof(i));
  _stream->read((char*)(&q), sizeof(q));
  if (!(*_stream)) {
    eof = true;
    return gr_complex(0,0);
  }  else {
    if (++_counter == _num_samples)
      _counter = 0;
    return gr_complex(i/32768.0f, q/32768.0f);
  }
}
gr_complex kiwi_wav_source_impl::get_next_sample(bool& kiwi_chunk, bool& eof) {
  kiwi_chunk = eof = false;
  if (_counter == 0) {
    wav::chunk_base c;
    _pos = _stream->tellg();
    _stream->read((char*)(&c), sizeof(c));
    if (!(*_stream)) {
      eof = true;
      return gr_complex(0,0);
    }
    if (c.id() == "data") {
      _num_samples = c.size()/4;
      return read_sample(eof);
    } else if (c.id() == "kiwi") {
      _stream->seekg(_pos);
      _stream->read((char*)(&_last_kiwi_chunk), sizeof(_last_kiwi_chunk));
      if (!(*_stream)) {
        GR_LOG_ERROR(d_logger, "incomplete kiwi chunk");
        eof = true;
      } else {
        kiwi_chunk = true;
      }
      return gr_complex(0,0);
    } else {
      GR_LOG_WARN(d_logger, str(boost::format("skipping unknown chunk '%s' len=%d") % c.id() % c.size()));
      _stream->seekg(_stream->tellg() + c.size());
    }
  } else if (_counter < _num_samples) {
    return read_sample(eof);
  }
  return WORK_DONE; // should never be reached
}

int kiwi_wav_source_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
{
  gr_complex *out = (gr_complex *)output_items[0];
  bool eof=false, kiwi_chunk=false;
  int nout = 0;
  for (nout=0; nout<noutput_items;) {
    out[nout] = get_next_sample(kiwi_chunk, eof);
    if (eof)
      return WORK_DONE;
    if (kiwi_chunk) {
      pmt::pmt_t const val = pmt::make_tuple(pmt::from_uint64(_last_kiwi_chunk.gpssec()),
                                             pmt::from_double(1e-9*_last_kiwi_chunk.gpsnsec()));
      add_item_tag(0, nitems_written(0)+nout, TIME_KEY, val, _id);
    } else {
      ++nout;
    }
  }
  return nout;
}

} /* namespace kiwisdr */
} /* namespace gr */
