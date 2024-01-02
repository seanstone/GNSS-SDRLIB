#
# makefile for gnss-sdr-cli
#

# Use or not use RTLSDR and BLADERF
# 1:Use 0:Not Use
USE_RTLSDR=0
USE_BLADERF=0

INCLUDE = \
	-Isrc \
	-Ilib/rtklib \
	-Ithird-party/libfec \
	-Isrc/rcv/rtlsdr \
	-Isrc/rcv/bladerf

CC=gcc
OPTIONS=-DSSE2_ENABLE -DFFTMTX

LIBS=third-party/libfec/build/libfec.a

SRCS = \
	src/sdrmain.cpp \
	src/sdrcmn.c \
	src/sdracq.c \
	src/sdrcode.c \
	src/sdrinit.c \
	src/sdrnav.c \
	src/sdrnav_gps.c \
	src/sdrnav_glo.c \
	src/sdrnav_gal.c \
	src/sdrnav_bds.c \
	src/sdrnav_sbs.c \
	src/sdrout.c \
	src/sdrplot.c \
	src/sdrrcv.c \
	src/sdrspec.c \
	src/sdrtrk.c \
	src/sdrlex.c \
	src/sdrsync.c \
	lib/rtklib/rtkcmn.c \
	lib/rtklib/rtcm.c \
	lib/rtklib/rtcm2.c \
	lib/rtklib/rtcm3.c \
	lib/rtklib/rtcm3e.c \
	lib/rtklib/rinex.c \

ifeq ($(USE_RTLSDR),1)
OPTIONS+=-DRTLSDR
LIBS+=-lrtlsdr
SRCS+= src/rcv/rtlsdr/rtlsdr.c src/rcv/rtlsdr/convenience.c
endif

ifeq ($(USE_BLADERF),1)
OPTIONS+=-DBLADERF
LIBS+=-lbladeRF
SRCS+=src/rcv/bladerf/bladerf.c
endif

OBJS = $(patsubst %, build/%.o, $(SRCS))

CFLAGS=-Wall -O3 -march=native $(INCLUDE) $(OPTIONS) -g
LDLIBS=-lm -lrt -lfftw3f -lfftw3f_threads -lpthread $(LIBS) -lusb-1.0

bin/gnss-sdrcli: $(OBJS)
	mkdir -p $(@D)
	 $(CXX) $(OBJS) $(CFLAGS) $(LDLIBS) -o $@

.PHONY: libfec
libfec: third-party/libfec/build/libfec.a

third-party/libfec/build/libfec.a:
	cd third-party/libfec && mkdir -p build && cd build && cmake .. && make -j

build/%.c.o : %.c
	mkdir -p $(@D)
	$(CC) -c $< $(CFLAGS) -o $@

build/%.cpp.o : %.cpp
	mkdir -p $(@D)
	$(CXX) -c $< $(CFLAGS) -o $@

sdrmain.o: $(SRC)/sdr.h
sdrcmn.o : $(SRC)/sdr.h
sdracq.o : $(SRC)/sdr.h
sdrcode.o: $(SRC)/sdr.h
sdrinit.o: $(SRC)/sdr.h
sdrout.o : $(SRC)/sdr.h
sdrnav.o : $(SRC)/sdr.h
sdrnav_gps.o : $(SRC)/sdr.h
sdrnav_glo.o : $(SRC)/sdr.h
sdrnav_gal.o : $(SRC)/sdr.h
sdrnav_bds.o : $(SRC)/sdr.h
sdrnav_sbs.o : $(SRC)/sdr.h
sdrplot.o: $(SRC)/sdr.h
sdrrcv.o : $(SRC)/sdr.h
sdrspec.o: $(SRC)/sdr.h
sdrtrk.o : $(SRC)/sdr.h
sdrlex.o : $(SRC)/sdr.h
sdrsync.o: $(SRC)/sdr.h
rtkcmn.o : $(SRC)/sdr.h
rtcm.o: $(SRC)/sdr.h
rtcm2.o : $(SRC)/sdr.h
rtcm3.o : $(SRC)/sdr.h
rtcm3e.o: $(SRC)/sdr.h
rinex.o : $(SRC)/sdr.h
rtlsdr.o : $(SRC)/sdr.h
convenience.o : $(SRC)/sdr.h
bladerf.o : $(SRC)/sdr.h

clean:
	rm -rf build bin


