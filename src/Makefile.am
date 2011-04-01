COMMON_CFLAGS = -Wall -O2

lib_LTLIBRARIES = libhatcontrol.la

libhatcontrol_la_SOURCES  = shmemlib.c \
                            u3.c

libhatcontrol_la_CPPFLAGS = $(COMMON_CFLAGS) -fPIC
libhatcontrol_la_LDFLAGS  = -shared
libhatcontrol_la_LIBADD   = -lm -llabjackusb -lpthread

pkginclude_HEADERS = u3.h \
                     shmemlib.h \
                     hat_drv.h

bin_PROGRAMS     = hat_drv hat_ctrl

hat_drv_SOURCES  = hat_drv.c
hat_drv_CFLAGS   = $(COMMON_CFLAGS)
hat_drv_LDADD    = libhatcontrol.la

hat_ctrl_SOURCES = hat_ctrl.c
hat_ctrl_CFLAGS  = $(COMMON_CFLAGS)
hat_ctrl_LDADD   = libhatcontrol.la
