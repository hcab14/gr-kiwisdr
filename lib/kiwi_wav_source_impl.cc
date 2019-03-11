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
kiwi_wav_source::make(std::string filename, double ref_signal_freq)
{
  return gnuradio::get_initial_sptr
    (new kiwi_wav_source_impl(filename, ref_signal_freq));
}

kiwi_wav_source_impl::kiwi_wav_source_impl(std::string filename, double ref_signal_freq)
  : gr::sync_block("kiwi_wav_source",
                   gr::io_signature::make(0, 0, 0),
                   gr::io_signature::make(1, 2, sizeof(gr_complex)))
  , _filename(filename)
  , _ref_signal_freq(ref_signal_freq)
  , _stream()
  , _pos()
  , _kiwi_chunk()
  , _last_kiwi_chunk()
  , _sample_counter(0)
  , _num_samples_in_chunk(0)
  , _last_gnss_time(0)
  , _use_new_gnss_solution(false)
  , _samp_rate(1)
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
  _sample_counter = 0;
  _chunk_counter = 0;
  _num_samples_in_chunk = 0;
  _use_new_gnss_solution = false;
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
      // skip the first two chunks
      for (int i=0; i<2*512; ++i) {
        bool has_kiwi_chunk=false, eof=false;
        gr_complex ref_signal;
        get_next_sample(ref_signal, has_kiwi_chunk, eof);
        if (eof)
          return false;
        if ((i%512) == 0) {
          if (!has_kiwi_chunk)
            return false;
          _last_kiwi_chunk = _kiwi_chunk;
          _last_gnss_time  = _kiwi_chunk.as_double();
          ++_chunk_counter;
        }
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
  _sample_counter = 0;
  _chunk_counter = 0;
  _num_samples_in_chunk = 0;
  _use_new_gnss_solution = false;
  return true;
}

gr_complex kiwi_wav_source_impl::read_sample(gr_complex& ref_signal, bool& eof) {
  ref_signal = gr_complex(0,0);
  int16_t i=0, q=0;
  _stream->read((char*)(&i), sizeof(i));
  _stream->read((char*)(&q), sizeof(q));
  if (!(*_stream)) {
    eof = true;
    return gr_complex(0,0);
  }  else {
    // TODO: handle GNSS week rollover
    double const time  = _last_kiwi_chunk.as_double() + _sample_counter/_samp_rate;
    double const phase = 2*M_PI * _ref_signal_freq * time;
    ref_signal = gr_complex(std::cos(phase), std::sin(phase));
    if (++_sample_counter == _num_samples_in_chunk)
      _sample_counter = 0;
    return gr_complex(i/32768.0f, q/32768.0f);
  }
}
gr_complex kiwi_wav_source_impl::get_next_sample(gr_complex& ref_signal, bool& has_kiwi_chunk, bool& eof) {
  has_kiwi_chunk = eof = false;
  if (_sample_counter == 0) {
    wav::chunk_base c;
    _pos = _stream->tellg();
    _stream->read((char*)(&c), sizeof(c));
    if (!(*_stream)) {
      eof = true;
      return gr_complex(0,0);
    }
    if (c.id() == "data") {
      _num_samples_in_chunk = c.size()/4;
      return read_sample(ref_signal, eof);
    } else if (c.id() == "kiwi") {
      _stream->seekg(_pos);
      _stream->read((char*)(&_kiwi_chunk), sizeof(_kiwi_chunk));
      if (!(*_stream)) {
        GR_LOG_ERROR(d_logger, "incomplete kiwi chunk");
        eof = true;
      } else {
        has_kiwi_chunk = true;
      }
      return gr_complex(0,0);
    } else {
      GR_LOG_WARN(d_logger, str(boost::format("skipping unknown chunk '%s' len=%d") % c.id() % c.size()));
      _stream->seekg(_stream->tellg() + c.size());
    }
  } else if (_sample_counter < _num_samples_in_chunk) {
    return read_sample(ref_signal, eof);
  }
  GR_LOG_FATAL(d_logger, "this point shoud be never reached");
  return gr_complex(0,0);
}

int kiwi_wav_source_impl::work(int noutput_items,
                               gr_vector_const_void_star &input_items,
                               gr_vector_void_star &output_items)
{
  gr_complex *out_iq  = (gr_complex *)output_items[0];
  gr_complex *out_ref = output_items.size() == 2 ? (gr_complex *)output_items[1] : NULL;
  int nout = 0;
  for (nout=0; nout<noutput_items;) {
    bool eof=false, has_kiwi_chunk=false;
    gr_complex ref_signal(0,0);
    out_iq[nout] = get_next_sample(ref_signal, has_kiwi_chunk, eof);
    if (out_ref)
      out_ref[nout] = ref_signal;
    if (eof)
      return WORK_DONE;
    if (has_kiwi_chunk) {
      // add TIME_KEY tag
      pmt::pmt_t const val = pmt::make_tuple(pmt::from_uint64(_kiwi_chunk.gpssec()),
                                             pmt::from_double(1e-9*_kiwi_chunk.gpsnsec()));
      add_item_tag(0, nitems_written(0)+nout, TIME_KEY, val, _id);
      // add SAMP_RATE_KEY tag when there is a new GNSS solution w.r.t. the old GNSS solution
      double const gnss_time = _kiwi_chunk.as_double();
      if (not _use_new_gnss_solution || (_kiwi_chunk.last_gnss_solution() > _last_kiwi_chunk.last_gnss_solution())) {
        double delta_t = gnss_time - _last_gnss_time;
        delta_t += (delta_t < 0)*7*24*3600; // GNSS week rollover
        _samp_rate = 512*_chunk_counter/delta_t;
        add_item_tag(0, nitems_written(0)+nout, SAMP_RATE_KEY, pmt::from_double(_samp_rate), _id);
        _last_gnss_time = gnss_time;
        _chunk_counter = 0;
        _use_new_gnss_solution = true;
      }
      _last_kiwi_chunk = _kiwi_chunk;
      ++_chunk_counter;
    } else {
      ++nout;
    }
  }
  return nout;
}

} /* namespace kiwisdr */
} /* namespace gr */
