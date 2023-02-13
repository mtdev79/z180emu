/*
 * 86Box	A hypervisor and IBM PC system emulator that specializes in
 *		running old operating systems and software designed for IBM
 *		PC systems and compatibles from 1981 through fairly recent
 *		system designs based on the PCI bus.
 *
 *		This file is part of the 86Box distribution.
 *
 *		Main include file for the application.
 *
 * Version:	@(#)86box.h	1.0.25	2018/10/02
 *
 * Authors:	Miran Grca, <mgrca8@gmail.com>
 *		Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *		Copyright 2016-2018 Miran Grca.
 *		Copyright 2017,2018 Fred N. van Kempen.
 */

/* Revision: 2019-01-26 Michal Tomek z180emu */

#ifndef EMU_86BOX_H
# define EMU_86BOX_H

#define plat_fopen fopen
#define wcscasecmp strcasecmp
#define wcslen strlen

extern int VERBOSE;
#define LOG(...) do { if (VERBOSE) printf (__VA_ARGS__); } while (0)

char *plat_get_extension(char *s);

#endif	/*EMU_86BOX_H*/
