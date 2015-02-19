AC_INIT([hatcontrol], [0.1.0])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_PROG_CC
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
