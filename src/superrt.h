#ifndef SUPERRT_H
#define SUPERRT_H

/* SuperRT ray tracing chip support (sd2snes mk3, FPGA core "superrt").
   Runs while a SuperRT ROM is loaded; returns 1 when the SNES was reset to
   the menu, 0 on a short reset (game restarts). */
int superrt_loop(void);

#endif
