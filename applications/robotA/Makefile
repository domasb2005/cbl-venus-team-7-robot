include ../shared.mk

SOURCES:=$(wildcard *.c)
CFLAGS+=-Werror
LDFLAGS+=-lmosquitto -lm -lcjson
# Add the include path for cJSON headers if they're in a non-standard location
CFLAGS+=-I/usr/include -I/usr/local/include

include ../end.mk

