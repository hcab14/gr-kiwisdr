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
                   gr::io_signature::make(0, 0, sizeof(gr_complex)),
                   gr::io_signature::make(1, 1, sizeof(gr_complex)))
  , _filename(filename)
  , _stream()
  , _pos()
  , _fmt()
  , _last_kiwi_chunk()
  , _buffer()
  , _gnss_tag_done(false)
  , _id(pmt::mp(_filename))
{
  GR_LOG_DECLARE_LOGPTR(d_logger);
  GR_LOG_ASSIGN_LOGPTR(d_logger, "kiwi_wav_source@"+_filename);
}

kiwi_wav_source_impl::~kiwi_wav_source_impl()
{
}

bool
kiwi_wav_source_impl::start()
{
  _buffer.clear();
  _stream = std::make_shared<std::ifstream>(_filename.c_str(), std::ios::binary);
  if (!_stream || !(*_stream)) {
    GR_LOG_ERROR(d_logger, str(boost::format("failed to open the file '%s'") % _filename));
    return false;
  }

  for (chunk_base c; *_stream; ) {
    _pos = _stream->tellg();
    _stream->read((char*)(&c), sizeof(c));
    if (!(*_stream)) {
      GR_LOG_ERROR(d_logger, "unexpected end of file");
      return false;
    }

    if (c.id() == "RIFF") {
      chunk_riff cr;
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
      _stream->read((char*)(&_fmt), sizeof(_fmt));
      if (!(*_stream)) {
        GR_LOG_ERROR(d_logger, "unexpected end of file");
        return false;
      } else if (_fmt.format() != 1 || _fmt.num_channels() != 2) {
        GR_LOG_ERROR(d_logger, str(boost::format("unsupported WAVE format: format=%d num_channels=%d") % c.id() % _fmt.format() %  _fmt.num_channels()));
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

bool
kiwi_wav_source_impl::stop()
{
  _stream.reset();
  _buffer.clear();
  _pos = 0;
  return true;
}

int
kiwi_wav_source_impl::work(int noutput_items,
                           gr_vector_const_void_star &input_items,
                           gr_vector_void_star &output_items)
{
  gr_complex *out = (gr_complex *)output_items[0];
  int nout=0;
  // insert buffered samples from the last call to work
  if (!_buffer.empty())
    GR_LOG_INFO(d_logger, str(boost::format("buffer size: %d (%d)") % _buffer.size() % noutput_items));

  for (int i=0; i<_buffer.size() && nout < noutput_items; ++i)
    out[nout++] = _buffer[i];

  if (nout == _buffer.size()) {
    // clear buffer
    _buffer.clear();
  } else { // buffer is not empty
    std::vector<gr_complex> buffer_new(_buffer.size() - nout);
    std::copy(_buffer.begin()+nout, _buffer.end(), buffer_new.begin());
    std::swap(_buffer, buffer_new);
    return nout;
  }

  for (chunk_base c; *_stream && nout < noutput_items; ) {
    _pos = _stream->tellg();
    _stream->read((char*)(&c), sizeof(c));
    if (!(*_stream))
      return WORK_DONE;

    if (c.id() == "data") {
      int const n = c.size()/4;
      int16_t i=0, j=0, q=0;
      for (; j<n && *_stream && nout < noutput_items ; ++j, ++nout) {
        _stream->read((char*)(&i), sizeof(i));
        _stream->read((char*)(&q), sizeof(q));
        out[nout] = gr_complex(i/32768.0f, q/32768.0f);
      }
      // buffer remaining IQ samples in the data block and insert them in the next call to work
      for (;j<n && *_stream; ++j) {
        _stream->read((char*)(&i), sizeof(i));
        _stream->read((char*)(&q), sizeof(q));
        _buffer.push_back(gr_complex(i/32768.0f, q/32768.0f));
      }
      if (j != n)
        GR_LOG_ERROR(d_logger, str(boost::format("incomplete data chunk: %d < %n") % j % n));
    } else if (c.id() == "kiwi") {
      _stream->seekg(_pos);
      chunk_kiwi kiwi;
      _stream->read((char*)(&kiwi), sizeof(kiwi));
      if (!(*_stream)) {
        GR_LOG_ERROR(d_logger, "incomplete kiwi chunk");
        return WORK_DONE;
      }
      //     insert a stream tag for each new (=not interpolated) gps timestamp
      if (kiwi.last_gnss_solution() - _last_kiwi_chunk.last_gnss_solution() < 0 && !_gnss_tag_done) {
        // GR_LOG_DEBUG(d_logger,(boost::format("gpssec=%16.9f (%3d)")
        //                        % kiwi.as_double()
        //                        % kiwi.last_gnss_solution()));
        // taken from gr-uhd/lib/usrp_source_impl.cc
        pmt::pmt_t const val = pmt::make_tuple(pmt::from_uint64(kiwi.gpssec()),
                                               pmt::from_double(1e-9*kiwi.gpsnsec()));
        add_item_tag(0, nitems_written(0)+nout, TIME_KEY, val, _id);
        _gnss_tag_done = true;//(kiwi.last_gnss_solution() == 0);
      }
      _last_kiwi_chunk = kiwi;
    } else {
      GR_LOG_WARN(d_logger, str(boost::format("skipping unknown chunk '%s' len=%d") % c.id() % c.size()));
      _stream->seekg(_stream->tellg() + c.size());
    }
  }
  return nout;
}

} /* namespace kiwisdr */
} /* namespace gr */
