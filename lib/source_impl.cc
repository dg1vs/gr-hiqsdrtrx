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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <complex>

#include "source_impl.h"
#include "fd_filter.h"

static double dcRef = 0, dcImf = 0;

dcplx lut[NLUT];
dcplx lut1[NLUT];
int lutn[NLUT];
double lutPartAv[NLUT];
int lutPartUsed[NLUT];
// for flowcontrol in tx, dirty hack :-(
int gr_hiqsdr_tx_samples;
int dpdState;
int testSig = TEST_SIG_NONE;
int txMirror = 1;

double checkRange(double n, double min, double max)
{
	if (n > max)
		return max;
	if (n < min)
		return min;
	return n;
}

void cprint(dcplx n, bool nl)
{
	printf("%+0.7f %+0.7f,", std::abs(n), std::arg(n));
	if (nl)
		printf("\n");
}

dcplx normal(0.4, 0);

namespace gr
{
	namespace hiqsdrtrx
	{

		source::sptr
		source::make(size_t itemsize, const char *host, unsigned short port, unsigned short c_port, unsigned short rx_fir_port, unsigned short tx_fir_port, int payload_size, bool eof, bool wait, int rxfreq, int txfreq, int rate, bool ant, int presel, int att, int txLevel, bool ptt, int txRate, int clockCorr, std::vector<gr_complex> rxFirTaps, std::vector<gr_complex> txFirTaps, bool dpd)
		{
			return gnuradio::get_initial_sptr(new source_impl(itemsize, host, port, c_port, rx_fir_port, tx_fir_port, payload_size, eof, wait, rxfreq, txfreq, rate, ant, presel, att, txLevel, ptt, txRate, clockCorr, rxFirTaps, txFirTaps, dpd));
		}

		/*
     * The private constructor
     */
		source_impl::source_impl(size_t itemsize, const char *host, unsigned short port, unsigned short c_port, unsigned short rx_fir_port, unsigned short tx_fir_port, int payload_size, bool eof, bool wait, int rxfreq, int txfreq, int rate, bool ant, int presel, int att, int txLevel, bool ptt, int txRate, int clockDiff, std::vector<gr_complex> rxFirTaps, std::vector<gr_complex> txFirTaps, bool dpd_on)
			: gr::sync_block("source",
							 gr::io_signature::make(0, 0, 0),
							 gr::io_signature::make(1, 1, itemsize))
		{
			d_itemsize = itemsize;
			d_payload_size = payload_size;
			d_eof = eof;
			d_socket = -1;
			gr_hiqsdr_tx_samples = 0;

			testSig = TEST_SIG_NONE;
			cwMode = false;
			clockCorr = clockDiff;
			firmVersion = 1;

			memset(ctlBuf, 0, sizeof(ctlBuf));
			strncpy((char *)ctlBuf, "St", 2);
			ctlBuf[13] = firmVersion;
			if (txMirror)
				ctlBuf[21] = 0x01;

			d_temp_buff = new unsigned char[d_payload_size]; // allow it to hold up to payload_size bytes

			d_socket = openSocket(host, port);
			c_socket = openSocket(host, c_port);
			rx_fir_socket = openSocket(host, rx_fir_port);
			tx_fir_socket = openSocket(host, tx_fir_port);

			for (int i = 0; i < 3; i++)
			{
				(void)send(d_socket, "rr", 2, 0); // send return addr
				usleep(2000);
			}

			outBufPos = 0;
			rxDel = 0;
			rxDel1 = 0;
			rxDel2 = 0;
			found = 0;
			cnt = 0;
			dpd = dpd_on;

			if (dpd)
			{
				dpdState = DPD_SYNC_SEARCH_CORR;
			}
			else
			{
				dpdState = DPD_OFF;
			}

			setRXFreq(rxfreq);
			setTXFreq(txfreq);
			setSampleRate(rate);
			setAnt(ant);
			setPresel(presel);
			setAttenuator(att);
			setTXLevel(txLevel);
			setTXRate(txRate);
			setPtt(ptt);
			setFilter(rxFirTaps, FILTER_RX);
			setFilter(txFirTaps, FILTER_TX);

			memset(lut, 0, sizeof(lut));
			memset(lut1, 0, sizeof(lut1));
			memset(lutn, 0, sizeof(lutn));
			memset(lutPartAv, 0, sizeof(lutPartAv));
			memset(lutPartUsed, 0, sizeof(lutPartUsed));
			for (int i = 0; i < NLUT; i++)
			{
				lut[i] = dcplx(1, 0);
				lut1[i] = dcplx(1, 0);
			}
		}

		/*
     * Our virtual destructor.
     */
		source_impl::~source_impl()
		{
			disconnect();
			delete[] d_temp_buff;

			if (d_socket != 0)
			{
				shutdown(d_socket, SHUT_RDWR);
				::close(d_socket);
			}

			if (c_socket != 0)
			{
				shutdown(d_socket, SHUT_RDWR);
				::close(d_socket);
			}

			if (rx_fir_socket != 0)
			{
				shutdown(rx_fir_socket, SHUT_RDWR);
				::close(rx_fir_socket);
			}

			if (tx_fir_socket != 0)
			{
				shutdown(tx_fir_socket, SHUT_RDWR);
				::close(tx_fir_socket);
			}
		}

		int source_impl::work(int noutput_items,
							  gr_vector_const_void_star &input_items,
							  gr_vector_void_star &output_items)
		{

			gr_complex *out = (gr_complex *)output_items[0];
			ssize_t r = 0;
			int i;
			int ret = 0;

			rxData.clear();
			txData.clear();

			while (1)
			{
				// RCV_TIMEO doesn't work on all systems (e.g., Cygwin)
				// use select() instead of, or in addition to RCV_TIMEO
				fd_set readfds;
				timeval timeout;
				timeout.tv_sec = 1; // Init timeout each iteration.  Select can modify it.
				timeout.tv_usec = 0;
				FD_ZERO(&readfds);
				FD_SET(d_socket, &readfds);
				r = select(FD_SETSIZE, &readfds, NULL, NULL, &timeout);
				if (r < 0)
				{
					perror("hiqsdr: select");
					return -1;
				}
				else if (r == 0)
				{ // timed out
					return -1;
				}

				// This is a non-blocking call with a timeout set in the constructor
				r = recv(d_socket, d_temp_buff, d_payload_size, 0); // get the entire payload or the what's available

				// Check if there was a problem; forget it if the operation just timed out
				if (r < 0)
				{
					if (errno == EAGAIN)
					{ // handle non-blocking call timeout
						return -1;
					}
					else
					{
						perror("hiqsdr recv");
						return -1;
					}
				}
				else
				{
					if (++seqNr != d_temp_buff[0])
						printf("%i: lost packets %u\n", seqNr, abs((unsigned char)d_temp_buff[0] - seqNr));
					seqNr = d_temp_buff[0];
					// Calculate the number of bytes we can take from the buffer in this call
					r -= 2; // skip header
					int samples = r / byteMode / 2;
					if (txMirror)
						samples = r / (byteMode + 2) / 2;
					int bi = 2;
					static const int NBITS = 32; // # of bits in a sample
					gr_hiqsdr_tx_samples += samples * txRate / rxRate;
					//printf ("rx items %i\n",gr_hiqsdr_tx_samples);

					if (byteMode == 1)
					{
						for (i = 0; i < samples; i++)
						{
							// 8 Bit -> Float
							int re = d_temp_buff[bi + 0] << 24;
							int im = d_temp_buff[bi + 1] << 24;
							re = re >> 8;
							im = im >> 8;
							float ref = (float)re * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
							float imf = (float)im * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
							bi += 2;
							outBuf[outBufPos++] = gr_complex(ref, imf);
							if (outBufPos >= noutput_items)
							{
								memcpy(out, outBuf, noutput_items * sizeof(gr_complex));
								if (outBufPos - noutput_items > 0)
									memmove(outBuf, &outBuf[noutput_items], (outBufPos - noutput_items) * sizeof(gr_complex));
								outBufPos -= noutput_items;
								ret = noutput_items;
							}
						}
						if (ret == 0)
							continue;
						return ret;
					}
					else if (byteMode == 2)
					{
						// 16 Bit -> Float
						for (i = 0; i < samples; i++)
						{
							int re = (d_temp_buff[bi + 0] << 16) + (d_temp_buff[bi + 1] << 24);
							int im = (d_temp_buff[bi + 2] << 16) + (d_temp_buff[bi + 3] << 24);
							float ref = (float)re * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
							float imf = (float)im * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
							bi += 4;
							outBuf[outBufPos++] = gr_complex(ref, imf);
							if (outBufPos >= noutput_items)
							{
								memcpy(out, outBuf, noutput_items * sizeof(gr_complex));
								if (outBufPos - noutput_items > 0)
									memmove(outBuf, &outBuf[noutput_items], (outBufPos - noutput_items) * sizeof(gr_complex));
								outBufPos -= noutput_items;
								ret = noutput_items;
							}
						}
						if (ret == 0)
							continue;
						return ret;
					}
					else
					{
						for (i = 0; i < samples; i++)
						{
							// 24 Bit -> Float
							int re = (d_temp_buff[bi + 0] << 8) + (d_temp_buff[bi + 1] << 16) + (d_temp_buff[bi + 2] << 24);
							int im = (d_temp_buff[bi + 3] << 8) + (d_temp_buff[bi + 4] << 16) + (d_temp_buff[bi + 5] << 24);

							float ref = (float)re * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
							float imf = (float)im * (1.0 / (float)((1LL << (NBITS - 1)) - 1));

							// DC-Compensation
							int dcCompensation = 1;
							if (dcCompensation)
							{
								dcRef = (dcRef * 100000 + ref) / 100001;
								dcImf = (dcImf * 100000 + imf) / 100001;
								ref -= dcRef;
								imf -= dcImf;
							}

							rxData.push_back(dcplx(ref, imf));
							outBuf[outBufPos++] = gr_complex(ref, imf);
							//printf("%02x%02x%02x %02x%02x%02x %f %f\n", d_temp_buff[bi+2], d_temp_buff[bi+1], d_temp_buff[bi+0], d_temp_buff[bi+5], d_temp_buff[bi+4], d_temp_buff[bi+3], ref,imf);
							bi += 6;
							if (txMirror && (i & 1))
							{
								for (int i = 0; i < 2; i++)
								{
									int re = (d_temp_buff[bi + 0] << 16) + (d_temp_buff[bi + 1] << 24);
									int im = (d_temp_buff[bi + 2] << 16) + (d_temp_buff[bi + 3] << 24);
									float ref = (float)re * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
									float imf = (float)im * (1.0 / (float)((1LL << (NBITS - 1)) - 1));
									txData.push_back(dcplx(ref, imf));
									bi += 4;
									// outBuf[outBufPos++] = gr_complex(ref, imf);
								}
							}
							if (outBufPos >= noutput_items)
							{
								memcpy(out, outBuf, noutput_items * sizeof(gr_complex));
								if (outBufPos - noutput_items > 0)
									memmove(outBuf, &outBuf[noutput_items], (outBufPos - noutput_items) * sizeof(gr_complex));
								outBufPos -= noutput_items;
								ret = noutput_items;
							}
						}

						if (ret == 0)
							continue;

						if (dpd)
							calcDPD(rxData, txData);
						return ret;
					}
				}
			}
			return -1;
		}

		std::vector<dcplx> source_impl::fracDelay(std::vector<dcplx> &in, int fDelay)
		{
			int i;
			std::vector<dcplx> out;

			if (fDelay >= MAX_FD_FILTER)
				fDelay = MAX_FD_FILTER - 1;

			//in.clear();
			//for (i=0;i<100;i++) in.push_back(i);
			out.clear();
			for (int i = 0; i < in.size() - FD_FILTER_LENGTH; i++)
			{
				dcplx y(0, 0);
				for (int j = 0; j < FD_FILTER_LENGTH; j++)
					y += in[i + j] * dcplx((double)fd_filter[fDelay][j], 0);
				//printf("%i %f %f \n",fDelay, std::real(in[i]), std::real(y));
				out.push_back(y);
			}
			return out;
		}

		void source_impl::printDPDState()
		{
			static int oldState = DPD_OFF;
			if (dpdState != oldState)
			{
				switch (dpdState)
				{
				case DPD_OFF:
					printf("dpd state: off\n");
					break;
				case DPD_SYNC_SEARCH_CORR:
					printf("dpd state: search corr\n");
					break;
				case DPD_SYNC_SEARCH_FINE:
					printf("dpd state: search fine\n");
					break;
				case DPD_LEARN:
					printf("dpd state: learning\n");
					break;
				case DPD_READY:
					printf("dpd state: ready\n");
					break;
				case DPD_READY_OFF:
					printf("dpd state: off, ready\n");
					break;
				default:
					printf("dpd state: unknown\n");
					break;
				}
				oldState = dpdState;
			}
		}

		void source_impl::calcDPD(std::vector<dcplx> rxData, std::vector<dcplx> txData)
		{
			double max = 0;
			int maxIdx = 0;
			int i;

			printDPDState();
			switch (dpdState)
			{
			case DPD_SYNC_SEARCH_CORR:
				testSig = TEST_SIG_FM_NOICE;
				// step 1: search correlation to determination delta t
				// best cross correlation
				for (i = 0; i < 500; i++)
				{
					dcplx s(0, 0);
					for (int j = 0; j < 500; j++)
						s += rxData[i + j] * std::conj(txData[j]);
					if (std::abs(s) > max)
					{
						max = std::abs(s);
						maxIdx = i;
					}
				}

				printf("delay %i, max %f\n", maxIdx, max);
				if (maxIdx > 50 && maxIdx == maxIdx1)
					found++;
				else
					found = 0;

				if (found > 3)
				{
					sStep = MAX_FD_FILTER / 2;
					dpdState = DPD_SYNC_SEARCH_FINE;
					rxDel = maxIdx * MAX_FD_FILTER;
				}
				maxIdx1 = maxIdx;
				break;

			case DPD_SYNC_SEARCH_FINE:
				max = 0;
				for (i = -1; i < 2; i++)
				{
					std::vector<dcplx> rxData1 = rxData;
					rxData1.erase(rxData1.begin(), rxData1.begin() + (rxDel + i) / MAX_FD_FILTER - FD_FILTER_LENGTH / 2 + 1);

					dcplx s(0, 0);
					rxDelay = fracDelay(rxData1, (rxDel + i) % MAX_FD_FILTER);
					for (int j = 0; j < 500; j++)
					{
						s += rxDelay[j] * std::conj(txData[j]);
						s += rxDelay[j] * std::conj(txData[j]);
					}
					if (std::abs(s) > max)
					{
						max = std::abs(s);
						maxIdx = i;
					}
				}

				rxDel2 = rxDel1;
				rxDel1 = rxDel;
				rxDel += maxIdx * sStep;
				if (sStep > 1)
					sStep /= 2;
				printf("delay: %f max: %f\n", (double)rxDel / MAX_FD_FILTER, max);

				if (max > 0 && abs(rxDel2 - rxDel) < 2)
				{
					cnt = 0;
					for (i = 0; i < NLUT; i++)
					{
						lut[i] = dcplx(1, 0);
						lut1[i] = dcplx(1, 0);
					}
					lutParts = NLUT / 2;
					dpdState = DPD_LEARN;
				}

				if (max == 0)
				{
					dpdState = DPD_SYNC_SEARCH_CORR;
					rxDel = 0;
					rxDel1 == 0;
					rxDel2 == 0;
					found = 0;
				}
				break;

			case DPD_LEARN:
			case DPD_READY:
			{
				if (dpdState == DPD_LEARN)
					testSig = TEST_SIG_2TONE;
				else
					testSig = TEST_SIG_NONE;

				// apply rx to tx delay
				rxData.erase(rxData.begin(), rxData.begin() + (rxDel) / MAX_FD_FILTER - FD_FILTER_LENGTH / 2 + 1);
				rxDelay = fracDelay(rxData, (rxDel) % MAX_FD_FILTER);

				// iterate rx data
				std::vector<dcplx>::iterator itRx, itTx = txData.begin();
				for (itRx = rxDelay.begin(); itRx != rxDelay.end(); itRx++)
				{

					// initial step: wait for signal, caluculate maximum rx signal and fill intial LUT
					if (lut[0] == dcplx(1, 0))
					{
						if (std::abs(*itTx) >= 0.99)
						{
							maxRx = std::abs(*itRx / *itTx);
							printf("maxRx = %f\n", maxRx);
							for (i = 0; i < NLUT; i++)
								lut[i] = *itTx / *itRx * maxRx;
						}
					}
					else
					{

						// calculate rx/tx and fill LUT
						double fidx = (std::abs(*itRx) * NLUT / maxRx);
						int idx = (int)fidx;
						int average = 2000;
						if (idx < NLUT)
						{
							double r = (std::abs(lut[idx]) * average + std::abs(*itTx / *itRx) * maxRx) / (average + 1);
							double p = (std::arg(lut[idx]) * average + std::arg(*itTx / *itRx)) / (average + 1);
							int idx1 = idx / lutParts;
							for (i = lutParts * idx1; i < lutParts * idx1 + lutParts; i++)
								lut[i] = dcplx(r * cos(p), r * sin(p));
							if (lutParts == 1)
							{
								lutPartAv[idx] = (lutPartAv[idx] * average + fidx - idx) / (average + 1);
								lutPartUsed[idx] = 1000;
							}
						}
						else
						{
							//printf ("rx overflow %i\n",idx);
						}
					}
					itTx++;
				}
				for (i = 0; i < NLUT; i++)
					if (lutPartUsed[i] > 0)
						lutPartUsed[i]--;

				// if valid LUT, calculate lut1 for tx
				if (lut[0] == dcplx(1, 0))
					break;
				if (cnt++ > 10)
				{
					cnt = 0;

					// low part
					for (i = 0; i < lutParts / 2; i++)
						lut1[i] = lut[lutParts / 2];
					// liear approximation for other parts
					for (i = 0; i < NLUT; i += lutParts)
					{
						for (int j = 0; j < lutParts; j++)
						{
							if (i + lutParts >= NLUT)
							{
								if (j < lutParts / 2)
									lut1[lutParts / 2 + i + j] = (lut[i] - lut[i - lutParts]) * (double)j / (double)lutParts + lut[i];
							}
							else
							{
								lut1[lutParts / 2 + i + j] = (lut[i + lutParts] - lut[i]) * (double)j / (double)lutParts + lut[i];
							}
						}
					}

					// overwrite noice in for lower LUT index
					for (i = 0; i < 10; i++)
						lut1[i] = lut[10];

					//memcpy(lut1,lut,sizeof(lut));
					for (i = 0; i < NLUT; i++)
					{
						//								printf ("%i %i %f ",i, lutPartUsed[i], lutPartAv[i]); cprint(lut1[i],1);
					}

					if (lutParts >= 2)
					{
						lutParts /= 2;
						printf("parts %i\n", lutParts);
					}
					else
						dpdState = DPD_READY;
				}
			}
			break;
			}
		}

		int source_impl::openSocket(const char *host, unsigned short port)
		{
			int sock;

			sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (sock == -1)
			{
				perror("hiqsdr: socket open");
				return 0;
			}

			// Don't wait when shutting down
			linger lngr;
			lngr.l_onoff = 1;
			lngr.l_linger = 0;
			if (setsockopt(sock, SOL_SOCKET, SO_LINGER, &lngr, sizeof(linger)) == -1)
			{
				if (errno != ENOPROTOOPT)
				{ // no SO_LINGER for SOCK_DGRAM on Windows
					perror("hiqsdr: setsockopt");
					return 0;
				}
			}

			if (host != NULL)
			{
				// Get the destination address
				struct addrinfo *ip_dst;
				struct addrinfo hints;
				memset((void *)&hints, 0, sizeof(hints));
				hints.ai_family = AF_INET;
				hints.ai_socktype = SOCK_DGRAM;
				hints.ai_protocol = IPPROTO_UDP;
				char port_str[12];
				sprintf(port_str, "%d", port);

				if (getaddrinfo(host, port_str, &hints, &ip_dst) != 0)
				{
					perror("hiqsdr: socket getaddrinfo");
					return 0;
				}

				if (::connect(sock, ip_dst->ai_addr, ip_dst->ai_addrlen) == -1)
				{
					perror("hiqsdr: socket connect");
					return 0;
				}

				freeaddrinfo(ip_dst);
			}
			return sock;
		}

		// Return port number of d_socket
		int source_impl::get_port(void)
		{
			sockaddr_in name;
			socklen_t len = sizeof(name);
			int ret = getsockname(d_socket, (sockaddr *)&name, &len);
			if (ret)
			{
				perror("source_impl getsockname");
				return -1;
			}
			return ntohs(name.sin_port);
		}

		void source_impl::closeSocket(int socket)
		{
			// gruel::scoped_lock guard(d_mutex);  // protect d_socket from work()

			// Sending EOF can produce ERRCONNREFUSED errors that won't show up
			//  until the next send or recv, which might confuse us if it happens
			//  on a new connection.  The following does a nonblocking recv to
			//  clear any such errors.
			timeval timeout;
			timeout.tv_sec = 0; // zero time for immediate return
			timeout.tv_usec = 0;
			fd_set readfds;
			FD_ZERO(&readfds);
			FD_SET(socket, &readfds);
			int r = select(FD_SETSIZE, &readfds, NULL, NULL, &timeout);
			if (r < 0)
			{
				perror("hiqsdr select");
			}
			else if (r > 0)
			{ // call recv() to get error return
				r = recv(socket, (char *)&readfds, sizeof(readfds), 0);
				if (r < 0)
				{
					perror("hiqsdr recv");
				}
			}
		}

		void source_impl::disconnect()
		{
			(void)send(d_socket, "ss", 2, 0); // stop

			// Send a few zero-length packets to signal receiver we are done
			if (d_eof)
			{
				int i;
				for (i = 0; i < 3; i++)
					(void)send(d_socket, NULL, 0, 0); // ignore errors
			}
			usleep(1000);

			closeSocket(d_socket);
			closeSocket(c_socket);
			closeSocket(rx_fir_socket);
			closeSocket(tx_fir_socket);

			return;
		}

		void source_impl::hiqSend()
		{
			if (ctlBuf[13] > 0)
				send(c_socket, ctlBuf, 22, 0);
			else
				send(c_socket, ctlBuf, 14, 0);
		}

		void source_impl::setRXFreq(int f)
		{
			f = f % RX_CLOCK;
			if (f > RX_CLOCK / 2)
				f = RX_CLOCK - f;

			unsigned int ph = ((int)((float)(f) / (RX_CLOCK + clockCorr) * 0x100000000LL + 0.5)) & 0xffffffff;

			ctlBuf[2] = (ph >> 0) & 0xff;
			ctlBuf[3] = (ph >> 8) & 0xff;
			ctlBuf[4] = (ph >> 16) & 0xff;
			ctlBuf[5] = (ph >> 24) & 0xff;

			hiqSend();
		}

		void source_impl::setTXFreq(int f)
		{
			unsigned int ph = ((int)((float)(f) / (RX_CLOCK + clockCorr) * 0x100000000LL + 0.5)) & 0xffffffff;

			ctlBuf[6] = (ph >> 0) & 0xff;
			ctlBuf[7] = (ph >> 8) & 0xff;
			ctlBuf[8] = (ph >> 16) & 0xff;
			ctlBuf[9] = (ph >> 24) & 0xff;

			hiqSend();
		}

		void source_impl::setSampleRate(int rate)
		{
			rxRate = rate;
			byteMode = 3;
			switch (rate)
			{
			case 8000:
			case 9600:
			case 12000:
			case 16000:
			case 19200:
			case 24000:
			case 38400:
				ctlBuf[12] = (2 << 6) | ((RX_CLOCK / 320 / rate) - 1);
				hiqSend();
				break;
			case 48000:
			case 60000:
			case 96000:
			case 120000:
			case 192000:
			case 240000:
			case 320000:
			case 384000:
			case 480000:
			case 640000:
			case 960000:
				ctlBuf[12] = (RX_CLOCK / 64 / rate) - 1;
				hiqSend();
				break;
			case 1280000:
			case 1536000:
			case 1920000:
				byteMode = 2;
				ctlBuf[12] = (1 << 6) | ((RX_CLOCK / 16 / rate) - 1);
				hiqSend();
				break;
			case 2560000:
			case 3840000:
				byteMode = 1;
				ctlBuf[12] = (3 << 6) | ((RX_CLOCK / 16 / rate) - 1);
				hiqSend();
				break;
			default:;
			}
		}

		void source_impl::setPtt(bool on)
		{
			ctlBuf[11] &= 0xf0;
			ctlBuf[11] |= (cwMode ? TX_CW : TX_SSB);
			if (on)
				ctlBuf[11] |= TX_PTT;
			if (firmVersion > 0)
				ctlBuf[11] |= USE_EXT_IO;
			hiqSend();

			if (on)
			{
				if (dpdState == DPD_READY_OFF)
					dpdState = DPD_READY;
			}
			else
			{
				if (dpdState == DPD_READY)
					dpdState = DPD_READY_OFF;
			}
		}

		void source_impl::setCWMode(bool cw)
		{
			ctlBuf[11] &= 0xf0;
			cwMode = cw;
			ctlBuf[11] |= (cwMode ? TX_CW : TX_SSB);
			if (firmVersion > 0)
				ctlBuf[11] |= USE_EXT_IO;
			hiqSend();
		}

		void source_impl::setTXRate(int rate)
		{
			txRate = rate;
			ctlBuf[11] &= 0x0f;
			switch (rate)
			{
			case 192000:
				ctlBuf[11] |= 0x10;
				break;
			case 480000:
				ctlBuf[11] |= 0x20;
				break;
			case 8000:
				ctlBuf[11] |= 0x30;
				break;
			default:;
			}
			hiqSend();
		}

		void source_impl::setTXLevel(int l)
		{
			ctlBuf[10] = l;
			hiqSend();
		}

		void source_impl::setAttenuator(int n)
		{
			ctlBuf[15] = 0;
			if (n >= 20)
			{
				n -= 20;
				ctlBuf[15] |= 0x10;
			}
			if (n >= 10)
			{
				n -= 10;
				ctlBuf[15] |= 0x08;
			}
			if (n >= 8)
			{
				n -= 8;
				ctlBuf[15] |= 0x04;
			}
			if (n >= 4)
			{
				n -= 4;
				ctlBuf[15] |= 0x02;
			}
			if (n >= 2)
			{
				n -= 2;
				ctlBuf[15] |= 0x01;
			}
			hiqSend();
		}

		void source_impl::setDPD(bool on)
		{
			dpd = on;

			if (!dpd)
			{
				testSig = TEST_SIG_NONE;
				if (dpdState == DPD_READY)
				{
					dpdState = DPD_READY_OFF;
				}
				else
				{
					dpdState = DPD_OFF;
				}
				return;
			}
			if (dpdState == DPD_READY_OFF)
			{
				for (int i = 0; i < NLUT; i++)
				{
					lut[i] = dcplx(1, 0);
					lut1[i] = dcplx(1, 0);
				}
				lutParts = NLUT / 2;
				dpdState = DPD_LEARN;
			}
			else
			{
				rxDel = 0;
				rxDel1 == 0;
				rxDel2 == 0;
				found = 0;
				dpdState = DPD_SYNC_SEARCH_CORR;
			}
		}

		void source_impl::setPresel(int n)
		{

			ctlBuf[14] = n;
			hiqSend();
		}

		void source_impl::setAnt(bool on)
		{
			if (on)
				ctlBuf[16] |= HIQ_ANT;
			else
				ctlBuf[16] &= ~HIQ_ANT;
			hiqSend();
		}

		void source_impl::setClockCorr(int diff)
		{
			clockCorr = diff;
		}

#define COEF_LEN 5
#define COEF_CNT 4

		void source_impl::setFilter(std::vector<gr_complex> coef, int dest)
		{
			float re, im;
			unsigned int ire, iim;
			int reset = 0x80;
			int cnt = 0;
			int taps = 0;
			int socket;
			unsigned long long tmp;
			float sum_re = 0, sum_im = 0;

			printf("set filter tabs %u dest: %i\n", (unsigned int)coef.size(), dest);

			if (coef.size() == 0)
				return;

			socket = c_socket;

			memset(firBuf, 0, sizeof(firBuf));
			if (dest == FILTER_RX)
				strncpy((char *)firBuf, "Fr", 2);
			else
				strncpy((char *)firBuf, "Ft", 2);

			for (std::vector<gr_complex>::iterator it = coef.begin(); it != coef.end(); ++it)
			{
				re = (it->real() * ((1 << 17) - 1));
				im = (it->imag() * ((1 << 17) - 1));
				//printf ("%f + %f i\n",it->real(),it->imag());
				sum_re += it->real();
				sum_im += it->imag();

				if (re > 0)
					re += 0.5;
				if (re < 0)
					re -= 0.5;
				if (im > 0)
					im += 0.5;
				if (im < 0)
					im -= 0.5;

				ire = re;
				iim = im;
				tmp = iim & 0x3ffff;
				tmp = (tmp << 18);
				tmp |= ire & 0x3ffff;
				firBuf[2 + cnt * COEF_LEN + 0] = tmp >> 0;
				firBuf[2 + cnt * COEF_LEN + 1] = tmp >> 8;
				firBuf[2 + cnt * COEF_LEN + 2] = tmp >> 16;
				firBuf[2 + cnt * COEF_LEN + 3] = tmp >> 24;
				firBuf[2 + cnt * COEF_LEN + 4] = tmp >> 32 | reset;
				//	printf ("%f %f %u %lx\n",it->real(),re, ire, tmp);
				reset = 0;
				cnt++;
				if (cnt == COEF_CNT)
				{
					usleep(1000);
					send(socket, firBuf, 2 + COEF_CNT * COEF_LEN, 0);
					memset(firBuf + 2, 0, sizeof(firBuf) - 2);
					cnt = 0;
				}
				taps++;
			}
			if (cnt < COEF_CNT)
			{
				usleep(1000);
				send(socket, firBuf, 2 + COEF_CNT * COEF_LEN, 0);
				taps += (COEF_CNT - cnt);
			}
			memset(firBuf + 2, 0, sizeof(firBuf) - 2);
			if (dest == FILTER_RX)
			{
				// allign
				if (taps % 8 > 0)
				{
					usleep(1000);
					send(socket, firBuf, 2 + COEF_CNT * COEF_LEN, 0);
					taps += COEF_CNT;
				}
				// last 4 coefficients have to zero
				for (int i = 0; i < 8 * 4; i++)
				{
					usleep(1000);
					send(socket, firBuf, 2 + COEF_CNT * COEF_LEN, 0);
				}
			}
			printf("tabs %i, sum re %f sum im %f\n", taps, sum_re, sum_im);
		}

	} /* namespace hiqsdrtrx */
} /* namespace gr */
