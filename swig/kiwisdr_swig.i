/* -*- c++ -*- */

#define KIWISDR_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "kiwisdr_swig_doc.i"

%{
#include "kiwisdr/kiwisdr.h"
#include "kiwisdr/kiwi_wav_source.h"
%}


%include "kiwisdr/kiwisdr.h"
GR_SWIG_BLOCK_MAGIC2(kiwisdr, kiwisdr);
%include "kiwisdr/kiwi_wav_source.h"
GR_SWIG_BLOCK_MAGIC2(kiwisdr, kiwi_wav_source);
