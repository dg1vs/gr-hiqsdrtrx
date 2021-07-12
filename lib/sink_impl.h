/* -*- c++ -*- */
/*
 * Copyright 2021 dg1vs.
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

#ifndef INCLUDED_HIQSDRTRX_SINK_IMPL_H
#define INCLUDED_HIQSDRTRX_SINK_IMPL_H

#include <hiqsdrtrx/sink.h>
#include "source_impl.h"

namespace gr
{
  namespace hiqsdrtrx
  {

    class sink_impl : public sink
    {
    private:
      // Nothing to declare in this block.
      void predistortion(double &re, double &im);
      size_t d_itemsize;

      int d_payload_size; // maximum transmission unit (packet length)
      bool d_eof;         // send zero-length packet on disconnect
      int d_socket;       // handle to socket
      bool d_connected;   // are we connected?
      double th1, th2;
      //gruel::mutex  d_mutex;           // protects d_socket and d_connected
      dcplx interp(double idx);

    public:
      sink_impl(size_t itemsize, const char *host, unsigned short port, int payload_size, bool eof);
      ~sink_impl();

      // Where all the action really happens
      int work(int noutput_items,
               gr_vector_const_void_star &input_items,
               gr_vector_void_star &output_items);
      int payload_size() { return d_payload_size; }
      void connect(const char *host, unsigned short port);
      void disconnect();
    };

  } // namespace hiqsdrtrx
} // namespace gr

#endif /* INCLUDED_HIQSDRTRX_SINK_IMPL_H */
