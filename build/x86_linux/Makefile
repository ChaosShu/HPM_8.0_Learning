DIR_TOP = ../..
DIR_INC = $(DIR_TOP)/inc
DIR_SRC = $(DIR_TOP)/src
DIR_APP = $(DIR_TOP)/app
DIR_LIB = $(DIR_TOP)/lib
DIR_BIN = $(DIR_TOP)/bin

CC = gcc
CXX = g++
AR = ar
LD = ld

MKDIR = mkdir -p
RM = rm -rf

CFLAGS  = -g -O3 -fomit-frame-pointer -Wall -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Werror -Wno-strict-overflow  -std=c99
CFLAGS += -DLINUX -DX86_64 -msse4.2
CFLAGS += -I$(DIR_INC) -DIFVC_TRACE=0 -DIFVC_ASSERT=1

CXXFLAGS  = -g -O3 -fomit-frame-pointer -Wall -Werror -std=c++11
CXXFLAGS += -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-variable -Wno-attributes -Wno-strict-overflow -Wno-sign-compare -Wno-unused-value -Wno-unknown-pragmas 
CXXFLAGS += -DLINUX -DX86_64 -msse4.2
CXXFLAGS += -I$(DIR_INC)

CFLAGS_ESAO = $(CFLAGS) -mavx -mavx2

LDFLAGS = -L$(DIR_LIB) -lcom -lm -lstdc++
ARFLAGS = r

CXXSRCS = $(DIR_SRC)/enc_ibc_hashmap.cpp \
		$(DIR_SRC)/enc_sp.cpp \

CSRCS_ESAO=$(DIR_SRC)/enc_esao.c \
		$(DIR_SRC)/com_esao.c \

CSRCS = $(DIR_SRC)/com_img.c \
		$(DIR_SRC)/com_ipred.c \
		$(DIR_SRC)/com_mc.c \
		$(DIR_SRC)/com_recon.c \
		$(DIR_SRC)/com_tbl.c \
		$(DIR_SRC)/com_util.c \
		$(DIR_SRC)/com_itdq.c \
		$(DIR_SRC)/com_df.c \
		$(DIR_SRC)/com_sao.c \
		$(DIR_SRC)/com_ComAdaptiveLoopFilter.c \
		$(DIR_SRC)/com_picman.c \
		$(DIR_SRC)/dec.c \
		$(DIR_SRC)/dec_eco.c \
		$(DIR_SRC)/dec_util.c \
		$(DIR_SRC)/dec_bsr.c \
		$(DIR_SRC)/dec_DecAdaptiveLoopFilter.c \
		$(DIR_SRC)/enc.c \
		$(DIR_SRC)/enc_eco.c \
		$(DIR_SRC)/enc_mode.c \
		$(DIR_SRC)/enc_pinter.c \
		$(DIR_SRC)/enc_pintra.c \
		$(DIR_SRC)/enc_pibc.c \
		$(DIR_SRC)/enc_tq.c \
		$(DIR_SRC)/enc_util.c \
		$(DIR_SRC)/enc_tbl.c \
		$(DIR_SRC)/enc_bsw.c \
		$(DIR_SRC)/enc_sad.c \
		$(DIR_SRC)/enc_EncAdaptiveLoopFilter.c \


CSRCS_APP =	$(DIR_APP)/app_encoder.c \
			$(DIR_APP)/app_decoder.c \
			$(DIR_APP)/app_bitstream_merge.c

OBJS = $(CSRCS:.c=.o)
ESAO_OBJS = $(CSRCS_ESAO:.c=.o)
CXXOBJS = $(CXXSRCS:.cpp=.o)

all: RELEASE

# target for release mode
RELEASE: depend $(CXXOBJS) $(OBJS) $(ESAO_OBJS)
	$(MKDIR) $(DIR_LIB)
	$(AR) $(ARFLAGS) $(DIR_LIB)/libcom.a $(CXXOBJS) $(OBJS) $(ESAO_OBJS)
	$(MKDIR) $(DIR_BIN)
	$(CC) -o $(DIR_BIN)/app_encoder $(CFLAGS) $(DIR_APP)/app_encoder.c $(LDFLAGS)
	$(CC) -o $(DIR_BIN)/app_decoder $(CFLAGS) $(DIR_APP)/app_decoder.c $(LDFLAGS)
	$(CC) -o $(DIR_BIN)/app_bitstream_merge $(CFLAGS) $(DIR_APP)/app_bitstream_merge.c $(LDFLAGS)

depend: Makefile $(CXXSRCS) $(CSRCS) $(CSRCS_APP) $(CSRCS_ESAO)
	$(CXX) -M $(CXXFLAGS) $(CXXSRCS)
	$(CC) -M $(CFLAGS_ESAO) $(CSRCS_ESAO)
	$(CC) -M $(CFLAGS) $(CSRCS) $(CSRCS_APP) > .$@


clean:
	$(RM) $(OBJS) $(ESAO_OBJS) $(CXXOBJS) $(DIR_LIB) $(DIR_BIN) .depend

$(ESAO_OBJS):
	$(CC) -c -o $@ $(CFLAGS_ESAO) $(@:.o=.c)

$(OBJS):
	$(CC) -c -o $@ $(CFLAGS) $(@:.o=.c)

#%.o: %.c
#	$(CC) -c -o $@ $(CFLAGS) $<
    
%.o: %.cpp
	$(CXX) -c -o $@ $(CXXFLAGS) $<	

-include .depend

