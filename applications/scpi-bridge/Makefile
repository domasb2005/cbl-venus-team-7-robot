include ../shared.mk

SOURCES:=$(wildcard *.c)
CFLAGS+=-I${PROJECT_ROOT}/external/scpi-parser/libscpi/inc/

CFLAGS+=$(shell pkg-config --cflags glib-2.0) -lpthread
LDFLAGS+=$(shell pkg-config --libs glib-2.0)

EXTERNAL_LIBS+=${PROJECT_ROOT}/external/scpi-parser/libscpi/dist/libscpi.a

include ../end.mk

