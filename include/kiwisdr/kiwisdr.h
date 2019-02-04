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

#ifndef INCLUDED_KIWISDR_KIWISDR_H
#define INCLUDED_KIWISDR_KIWISDR_H

#include <kiwisdr/api.h>
#include <gnuradio/block.h>

namespace gr {
namespace kiwisdr {

/*!
 * \brief <+description of block+>
 * \ingroup kiwisdr
 *
 */
class KIWISDR_API kiwisdr : virtual public gr::block
{
  public:
  typedef boost::shared_ptr<kiwisdr> sptr;

  virtual ~kiwisdr() {}

  /*!
   * \brief Return a shared_ptr to a new instance of kiwisdr::kiwisdr.
   *
   * To avoid accidental use of raw pointers, kiwisdr::kiwisdr's
   * constructor is in a private implementation
   * class. kiwisdr::kiwisdr::make is the public interface for
   * creating new instances.
   */
  static sptr make(const std::string& host="",
                   const std::string& port="",
                   double freq_kHz = 10000,
                   int low_cut_Hz  = -6000,
                   int high_cut_Hz =  6000);

  virtual std::string get_client_public_ip() const = 0;
  virtual int         get_rx_chans()         const = 0;
  virtual int         get_chan_no_pwd()      const = 0;
  virtual bool        is_password_ok()       const = 0;
  virtual std::string get_version()          const = 0;
  virtual std::string get_cfg()              const = 0;
  virtual double      get_sample_rate()      const = 0;
  virtual bool        is_audio_initialized() const = 0;
  virtual double      get_center_freq()      const = 0;
  virtual double      get_bandwidth()        const = 0;
  virtual double      get_adc_clk_nom()      const = 0;

  virtual void set_rx_parameters(double freq_kHz,
                                 int low_cut_Hz,
                                 int high_cut_Hz) = 0;
};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_KIWISDR_H */
