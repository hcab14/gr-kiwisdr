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

#ifndef INCLUDED_KIWISDR_RX_PARAMETERS_H
#define INCLUDED_KIWISDR_RX_PARAMETERS_H

struct kiwi_rx_parameters {
  double freq_kHz;
  int    low_cut_Hz;
  int    high_cut_Hz;
} ;

#endif //  INCLUDED_KIWISDR_RX_PARAMETERS_H
