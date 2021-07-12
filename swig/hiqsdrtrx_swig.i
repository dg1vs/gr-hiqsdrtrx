/* -*- c++ -*- */

#define HIQSDRTRX_API

%include "gnuradio.i"           // the common stuff

//load generated python docstrings
%include "hiqsdrtrx_swig_doc.i"

%{
#include "hiqsdrtrx/source.h"
#include "hiqsdrtrx/sink.h"
%}

%include "hiqsdrtrx/source.h"
GR_SWIG_BLOCK_MAGIC2(hiqsdrtrx, source);
%include "hiqsdrtrx/sink.h"
GR_SWIG_BLOCK_MAGIC2(hiqsdrtrx, sink);
