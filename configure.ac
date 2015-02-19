AC_INIT([hatcontrol], [0.1.0])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AM_PROG_CC_C_O
m4_ifdef([AM_PROG_AR], [AM_PROG_AR])
LT_INIT
AC_CONFIG_FILES([
	Makefile
	src/Makefile
	libhatcontrol.pc
	])
PKG_CHECK_MODULES([DEPS], [
 glib-2.0 >= 2.22.1
])
AC_OUTPUT
