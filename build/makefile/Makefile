###############################################################################
#  Copyright (c) [2022] Ittiam Systems Pvt. Ltd.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted (subject to the limitations in the
#  disclaimer below) provided that the following conditions are met:
#  •	Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  •	Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  •	Neither the names of Dolby Laboratories, Inc. (or its affiliates),
#  Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
#  to endorse or promote products derived from this software without
#  specific prior written permission.
#
#  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
#  BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
#  LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
#  LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
#  BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
#  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
#  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
###############################################################################

#This is for including values like architecture, compiler command amd assembler commands etc.
include ./Make.defs



CFLAGS += -c -O3 -Wall -Wsequence-point -Wno-multichar


BIN			= ../bin/ia_mpegh_enc_app_$(ARCH).out
LIBS		= ../lib/ia_mpegh_enc_lib_$(ARCH).a
PATH_ENC    = ../../encoder
PATH_TEST   = ../../test

SRCS+=$(PATH_TEST)/impeghe_error.c\
      $(PATH_TEST)/impeghe_drc_user_config.c\
	  $(PATH_TEST)/impeghe_mae_user_config.c\
	  $(PATH_TEST)/impeghe_mp4_writer.c\
	  $(PATH_TEST)/impeghe_testbench.c

					

INCLUDES = -I$(PATH_ENC) -I$(PATH_TEST)
			   

OBJS			= $(SRCS:.c=.o)
OBJS_LIBS		= $(SRCS_LIBS:.c=.o)

CFLAGS += $(INCLUDES)


all: $(BIN)

$(OBJS): %.o: %.c
	$(CC) $(CFLAGS) $*.c -o $*.o 
	
ifeq ($(ARCH), x86_64) 
$(BIN): $(OBJS) $(LIBS)
	$(CC) -o  $@ $^  -lm
else ifeq ($(ARCH), x86)	
$(BIN): $(OBJS) $(LIBS)
	$(CC) -m32 -o  $@ $^  -lm
else	
$(BIN): $(OBJS) $(LIBS)
	$(CC) -o  $@ $^ --static -lm	
endif	

clean: src_clean 		
	rm -f $(BIN)

src_clean:
	rm -f $(OBJS)

    
#$(CC) -m32 -Wl,--gc-sections -Wl,--print-gc-sections -o  $@ $^ --static -lm
