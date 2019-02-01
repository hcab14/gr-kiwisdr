/* -*- c++ -*- */

#define KIWISDR_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "kiwisdr_swig_doc.i"

%{
#ifdef GR_KIWI_WS_CLIENT
#include "kiwisdr/kiwisdr.h"
#endif
#include "kiwisdr/kiwi_wav_source.h"
%}


#ifdef GR_KIWI_WS_CLIENT
%include "kiwisdr/kiwisdr.h"
GR_SWIG_BLOCK_MAGIC2(kiwisdr, kiwisdr);
#endif
%include "kiwisdr/kiwi_wav_source.h"
GR_SWIG_BLOCK_MAGIC2(kiwisdr, kiwi_wav_source);
