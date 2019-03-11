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


#ifndef INCLUDED_KIWISDR_KIWI_WAV_SOURCE_H
#define INCLUDED_KIWISDR_KIWI_WAV_SOURCE_H

#include <kiwisdr/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace kiwisdr {

/*!
 * \brief Source block reading KiwiSDR IQ wav files containing GNSS timestamps
 * wav files are produced by kiwirecorder.py and contain alternating 'kiwi' and 'data' chunks
 * where the 'kiwi' chunk contains GNSS timestamps
 * \ingroup kiwisdr
 *
 */
class KIWISDR_API kiwi_wav_source : virtual public gr::sync_block
{
  public:
  typedef boost::shared_ptr<kiwi_wav_source> sptr;

  /*!
   * \brief Return a shared_ptr to a new instance of kiwisdr::kiwi_wav_source.
   *
   * To avoid accidental use of raw pointers, kiwisdr::kiwi_wav_source's
   * constructor is in a private implementation
   * class. kiwisdr::kiwi_wav_source::make is the public interface for
   * creating new instances.
   */
  static sptr make(std::string filename, double ref_signal_freq);
};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_KIWI_WAV_SOURCE_H */

