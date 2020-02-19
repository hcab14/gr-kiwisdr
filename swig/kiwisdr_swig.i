/* -*- c++ -*- */

#define KIWISDR_API

%include "gnuradio.i"

//load generated python docstrings
%include "kiwisdr_swig_doc.i"

%{
#include "kiwisdr/align_streams.h"
#include "kiwisdr/kiwi_wav_source.h"
%}

%include "kiwisdr/align_streams.h"
GR_SWIG_BLOCK_MAGIC2(kiwisdr, align_streams);

%include "kiwisdr/kiwi_wav_source.h"
GR_SWIG_BLOCK_MAGIC2(kiwisdr, kiwi_wav_source);


#ifdef GR_KIWI_WS_CLIENT
%{
#include "kiwisdr/kiwisdr.h"
%}

%include "kiwisdr/kiwisdr.h"

GR_SWIG_BLOCK_MAGIC2(kiwisdr, kiwisdr);
#endif /* GR_KIWI_WS_CLIENT */
