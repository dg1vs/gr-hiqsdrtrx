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

#ifndef INCLUDED_HIQSDRTRX_SOURCE_H
#define INCLUDED_HIQSDRTRX_SOURCE_H

#include <hiqsdrtrx/api.h>
#include <gnuradio/sync_block.h>

#define RX_CLOCK	122880000LL
#define RX_EXT			0x80

#define TX_OFF      0
#define TX_CW       1
#define TX_SSB      2
#define USE_EXT_IO  4
#define TX_PTT      8

#define HIQ_PREAMP      0x10
#define HIQ_ANT         0x01

#define FILTER_RX	0
#define FILTER_TX	1

namespace gr {
  namespace hiqsdrtrx {

    /*!
     * \brief <+description of block+>
     * \ingroup hiqsdrtrx
     *
     */
    class HIQSDRTRX_API source : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<source> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of hiqsdrtrx::source.
       *
       * To avoid accidental use of raw pointers, hiqsdrtrx::source's
       * constructor is in a private implementation
       * class. hiqsdrtrx::source::make is the public interface for
       * creating new instances.
       */
      static sptr make(size_t itemsize, const char *host,unsigned short port, unsigned short c_port, unsigned short rx_fir_port, unsigned short tx_fir_port, int payload_size=1472,bool eof=true, bool wait=true, int rxfreq=7000000, int txfreq=7000000, int rate=48000,bool ant=0, int presel=0, int att=0, int txLevel=255, bool ptt=0, int txRate=48000, int clockCorr=0,std::vector<gr_complex> rxFirTaps=std::vector<gr_complex>() , std::vector<gr_complex> txFirTaps=std::vector<gr_complex>(), bool dpd=0);
    };

  } // namespace hiqsdrtrx
} // namespace gr

#endif /* INCLUDED_HIQSDRTRX_SOURCE_H */

