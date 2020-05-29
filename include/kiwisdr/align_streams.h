/* -*- c++ -*- */
/*
 * Copyright 2019 Christoph Mayer hcab14@gmail.com.
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

#ifndef INCLUDED_KIWISDR_ALIGN_STREAMS_H
#define INCLUDED_KIWISDR_ALIGN_STREAMS_H

#include <kiwisdr/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace kiwisdr {

/*!
 * \brief <+description of block+>
 * \ingroup kiwisdr
 *
 */
class KIWISDR_API align_streams : virtual public gr::sync_block
{
public:
  typedef std::shared_ptr<align_streams> sptr;

  virtual ~align_streams() {}

  /*!
   * \brief Return a shared_ptr to a new instance of kiwisdr::align_streams.
   *
   * To avoid accidental use of raw pointers, kiwisdr::align_streams's
   * constructor is in a private implementation
   * class. kiwisdr::align_streams::make is the public interface for
   * creating new instances.
   */
  static sptr make(unsigned num_streams,
                   bool from_same_kiwi);
};

} // namespace kiwisdr
} // namespace gr

#endif /* INCLUDED_KIWISDR_ALIGN_STREAMS_H */
