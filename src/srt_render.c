/* Built with -O2 regardless of the firmware-wide -Os: this is the hot path. */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif

/*
 * SuperRT software renderer - see srt_render.h
 *
 * Port of the SuperRT RTL (Renderer.sv, RayEngine*.sv, ExecEngine*.sv,
 * Maths.sv, Sky.sv, FixedRcpClocked.sv, FixedSqrtClocked.sv) by Ben Carter,
 * MIT licensed.  The pipelined hardware executes one command list
 * instruction per clock; this port executes the same program sequentially and
 * reproduces the RTL's fixed point arithmetic bit for bit (40/48 bit
 * multiplier truncation, 16 bit wrap-arounds, Newton-Raphson reciprocal and
 * square root with the hardware's seed table).
 *
 * Work that the hardware does unconditionally but whose result is never used
 * (e.g. sphere normals of objects that are not hit) is done lazily here.
 */
#include "srt_render.h"

#ifndef SRT_HOT
#define SRT_HOT
#endif

#define FIXED_SHIFT 14
#define DEPTH_INF   ((int32_t)0x7FFFFFFF)

/* maximum number of instructions per ray before we give up (the hardware
   would hang forever on an endless loop in the command list) */
#define SRT_MAX_INSTRUCTIONS_PER_RAY 65536

srt_stats_t srt_stats;

/* ------------------------------------------------------------------------- */
/* Fixed point maths (Maths.sv)                                              */
/* ------------------------------------------------------------------------- */

/* FixedMul: 40 bit two's complement product, >>> 14, truncated to 32 bits.
   Result = bits [45:14] of the product sign extended from bit 39, i.e. the
   low word shifted right by 14 combined with the sign extended (from bit 7)
   high word. Written with 32 bit operations so that GCC emits
   SMULL / SXTB / LSR / ORR on the Cortex-M4. */
static inline int32_t fmul(int32_t x, int32_t y) {
  int64_t p = (int64_t)x * (int64_t)y;
  uint32_t lo = (uint32_t)p;
  uint32_t hi = (uint32_t)(int32_t)(int8_t)(uint8_t)((uint64_t)p >> 32);
  return (int32_t)((lo >> FIXED_SHIFT) | (hi << (32 - FIXED_SHIFT)));
}

/* FixedMul48: 48 bit product (sign extended from bit 47) */
static inline int32_t fmul48(int32_t x, int32_t y) {
  int64_t p = (int64_t)x * (int64_t)y;
  uint32_t lo = (uint32_t)p;
  uint32_t hi = (uint32_t)(int32_t)(int16_t)(uint16_t)((uint64_t)p >> 32);
  return (int32_t)((lo >> FIXED_SHIFT) | (hi << (32 - FIXED_SHIFT)));
}

/* FixedMul16x16: inputs are truncated to 16 bits, 32 bit result */
static inline int32_t fmul16(int32_t x, int32_t y) {
  return ((int32_t)(int16_t)x * (int32_t)(int16_t)y) >> FIXED_SHIFT;
}

static inline int32_t add32(int32_t a, int32_t b) { return (int32_t)((uint32_t)a + (uint32_t)b); }
static inline int32_t sub32(int32_t a, int32_t b) { return (int32_t)((uint32_t)a - (uint32_t)b); }
static inline int32_t neg32(int32_t a) { return (int32_t)(0u - (uint32_t)a); }
static inline int16_t neg16(int16_t a) { return (int16_t)(0u - (uint32_t)(uint16_t)a); }

/* first guess for 1/sqrt(x), indexed by the highest set bit (30..1) */
static const int32_t rsqrt_seed[31] = {
  0x00200000, /* no bit in 30..1 set */
  0x001279a7, 0x000d105e, 0x00093cd3, 0x0006882f, 0x00049e69, 0x00034417,
  0x00024f34, 0x0001a20b, 0x0001279a, 0x0000d105, 0x000093cd, 0x00006882,
  0x000049e6, 0x00003441, 0x000024f3, 0x00001a20, 0x00001279, 0x00000d10,
  0x0000093c, 0x00000688, 0x0000049e, 0x00000344, 0x0000024f, 0x000001a2,
  0x00000127, 0x000000d1, 0x00000093, 0x00000068, 0x00000049, 0x00000034
};

static inline int32_t rsqrt_guess(int32_t v) {
  uint32_t m = (uint32_t)v & 0x7FFFFFFEu;
  if(!m) return rsqrt_seed[0];
  return rsqrt_seed[31 - __builtin_clz(m)];
}

#define ONE_POINT_FIVE 0x00006000

/* two Newton-Raphson iterations of 1/sqrt, 'half' is the pre-shifted input */
static inline int32_t rsqrt_iter(int32_t g, int32_t half) {
  g = fmul(g, sub32(ONE_POINT_FIVE, fmul(half, fmul(g, g))));
  g = fmul(g, sub32(ONE_POINT_FIVE, fmul(half, fmul(g, g))));
  return g;
}

/* FixedRcpClocked.sv */
static int32_t rcp_hw(int32_t in) {
  int sign = in < 0;
  int32_t a = sign ? neg32(in) : in;
  int32_t r;
  if(in == 0) return DEPTH_INF;
  r = rsqrt_iter(rsqrt_guess(a), (int32_t)((uint32_t)a >> 1));
  r = fmul(r, r);
  return sign ? neg32(r) : r;
}

/* FixedSqrtClocked.sv */
static int32_t sqrt_hw(int32_t in) {
  int32_t r;
  if(in == 0) return 0;
  r = rsqrt_iter(rsqrt_guess(in), (int32_t)((uint32_t)in >> 1));
  return fmul(r, in);
}

/* Maths.sv FixedRcpSqrt (arithmetic shift) */
static int32_t rsqrt_fn(int32_t in) {
  return rsqrt_iter(rsqrt_guess(in), in >> 1);
}

/* operand format conversions */
static inline int32_t cf8_7(uint32_t v)  { return (int32_t)(v << 17) >> 10; } /* 15 bit -> <<7 */
static inline int32_t cf4_7(uint32_t v)  { return (int32_t)(v << 21) >> 14; } /* 11 bit -> <<7 */
static inline int16_t cf2_10(uint32_t v) { return (int16_t)((int32_t)(v << 20) >> 16); } /* 12 bit -> <<4 */
static inline int32_t cf8_12(uint32_t v) { return (int32_t)(v << 12) >> 10; } /* 20 bit -> <<2 */
static inline int32_t cf8_1(uint32_t v)  { return (int32_t)(v << 23) >> 10; } /* 9 bit -> <<13 */

static inline uint8_t cmul(uint8_t x, uint8_t y) { return (uint8_t)(((uint32_t)x * y) >> 8); }
static inline uint8_t cadd(uint8_t x, uint8_t y) { uint32_t s = (uint32_t)x + y; return s > 255 ? 255 : (uint8_t)s; }

/* ------------------------------------------------------------------------- */
/* Execution engine (ExecEngine.sv)                                          */
/* ------------------------------------------------------------------------- */

enum {
  I_NOP = 0, I_Sphere, I_Plane, I_SphereSub, I_PlaneSub, I_SphereAnd, I_PlaneAnd,
  I_AABB, I_AABBSub, I_AABBAnd, I_RegisterHit, I_RegisterHitNoReset,
  I_Checkerboard, I_ResetHitState, I_Jump, I_ResetHitStateAndJump, I_Origin,
  I_Start, I_End
};

enum { C_AL = 0, C_OH, C_NH, C_ORH };

typedef struct {
  /* current ray (u_*) */
  int32_t sx, sy, sz;
  int16_t dx, dy, dz;
  int32_t rcpx, rcpy, rcpz;
  int shadow;

  /* stage 3 state */
  int32_t ox, oy, oz;

  /* stage 14 state */
  int32_t hit_entry, hit_exit;
  int16_t hit_nx, hit_ny, hit_nz;
  int obj_reg_hit;
  /* s14_hitX/Y/Z = start + dir * depth, evaluated lazily */
  int32_t hit_pos_depth;
  int hit_pos_pending;
  int32_t hit_x, hit_y, hit_z;

  int reg_hit;
  int32_t reg_depth;
  int32_t reg_x, reg_y, reg_z;
  int16_t reg_nx, reg_ny, reg_nz;
  uint16_t reg_albedo;
  uint8_t reg_refl;
} exec_state_t;

static exec_state_t ee;
static const uint64_t *cmd;

static inline void hit_pos_resolve(void) {
  if(ee.hit_pos_pending) {
    ee.hit_x = add32(ee.sx, fmul(ee.dx, ee.hit_pos_depth));
    ee.hit_y = add32(ee.sy, fmul(ee.dy, ee.hit_pos_depth));
    ee.hit_z = add32(ee.sz, fmul(ee.dz, ee.hit_pos_depth));
    ee.hit_pos_pending = 0;
  }
}

/* intersection result. Normals are produced on demand. */
typedef struct {
  int32_t entry, exit;
  int kind;            /* 0 = sphere, 1 = plane, 2 = aabb */
  /* sphere */
  int32_t objx, objy, objz, rad;
  int inside;
  /* plane / aabb normals (precomputed, cheap) */
  int16_t enx, eny, enz, xnx, xny, xnz;
} isect_t;

static void sphere_normals(const isect_t *is, int exit_side, int16_t *nx, int16_t *ny, int16_t *nz) {
  int32_t inv = rcp_hw(is->rad);
  int32_t d = exit_side ? is->exit : is->entry;
  if(!exit_side && is->inside) {
    *nx = neg16(ee.dx); *ny = neg16(ee.dy); *nz = neg16(ee.dz);
    return;
  }
  *nx = (int16_t)fmul(sub32(add32(ee.sx, fmul(ee.dx, d)), is->objx), inv);
  *ny = (int16_t)fmul(sub32(add32(ee.sy, fmul(ee.dy, d)), is->objy), inv);
  *nz = (int16_t)fmul(sub32(add32(ee.sz, fmul(ee.dz, d)), is->objz), inv);
}

static inline void entry_normal(const isect_t *is, int16_t *nx, int16_t *ny, int16_t *nz) {
  if(is->kind == 0) sphere_normals(is, 0, nx, ny, nz);
  else { *nx = is->enx; *ny = is->eny; *nz = is->enz; }
}

static inline void exit_normal(const isect_t *is, int16_t *nx, int16_t *ny, int16_t *nz) {
  if(is->kind == 0) sphere_normals(is, 1, nx, ny, nz);
  else { *nx = is->xnx; *ny = is->xny; *nz = is->xnz; }
}

static SRT_HOT void isect_sphere(uint64_t w, isect_t *is) {
  int32_t ocx, ocy, ocz, cpa, dsq, rsq, dfc, s, entry;
  is->kind = 0;
  is->entry = DEPTH_INF; is->exit = 0;
  is->objx = add32(cf8_7((uint32_t)(w >> 8) & 0x7FFF), ee.ox);
  is->objy = add32(cf8_7((uint32_t)(w >> 23) & 0x7FFF), ee.oy);
  is->objz = add32(cf8_7((uint32_t)(w >> 38) & 0x7FFF), ee.oz);
  is->rad = cf4_7((uint32_t)(w >> 53) & 0x7FF);
  ocx = sub32(ee.sx, is->objx);
  ocy = sub32(ee.sy, is->objy);
  ocz = sub32(ee.sz, is->objz);
  cpa = neg32(add32(add32(fmul(ocx, ee.dx), fmul(ocy, ee.dy)), fmul(ocz, ee.dz)));
  dsq = add32(add32(fmul(ocx, ocx), fmul(ocy, ocy)), fmul(ocz, ocz));
  rsq = fmul(is->rad, is->rad);
  is->inside = dsq < rsq;
  if(!((cpa >= 0) || is->inside)) return;
  dfc = sub32(dsq, fmul(cpa, cpa));
  if(!(dfc < rsq)) return;
  s = sqrt_hw(sub32(rsq, dfc));
  entry = is->inside ? 0 : sub32(cpa, s);
  if(entry < 0) return;
  is->entry = entry;
  is->exit = add32(cpa, s);
}

static SRT_HOT void isect_plane(uint64_t w, isect_t *is) {
  int16_t nx, ny, nz;
  int32_t dist, px, py, pz, dot, denom, r, t;
  int inside;
  is->kind = 1;
  is->entry = DEPTH_INF; is->exit = 0;
  is->enx = is->eny = is->enz = 0;
  is->xnx = is->xny = is->xnz = 0;
  nx = cf2_10((uint32_t)(w >> 8) & 0xFFF);
  ny = cf2_10((uint32_t)(w >> 20) & 0xFFF);
  nz = cf2_10((uint32_t)(w >> 32) & 0xFFF);
  dist = cf8_12((uint32_t)(w >> 44) & 0xFFFFF);
  px = add32(fmul(nx, dist), ee.ox);
  py = add32(fmul(ny, dist), ee.oy);
  pz = add32(fmul(nz, dist), ee.oz);
  dot = add32(add32(fmul(sub32(ee.sx, px), nx), fmul(sub32(ee.sy, py), ny)), fmul(sub32(ee.sz, pz), nz));
  inside = dot < 0;
  if(inside) {
    nx = neg16(nx); ny = neg16(ny); nz = neg16(nz);
    dot = neg32(dot);
  }
  denom = neg32(add32(add32(fmul16(ee.dx, nx), fmul16(ee.dy, ny)), fmul16(ee.dz, nz)));
  /* For inputs <= -8 (and > -2^18) rcp_hw always returns a negative value,
     and only the sign matters in that case, so skip the Newton-Raphson.
     (Exhaustively checked; tiny inputs overflow the 40 bit multiplier and
     can come out with either sign, so they still take the full path.) */
  r = (denom <= -8 && denom > -0x40000) ? -1 : rcp_hw(denom);
  if(r < 0) {
    if(inside) {
      is->entry = 0;
      is->exit = DEPTH_INF;
      is->enx = neg16(ee.dx); is->eny = neg16(ee.dy); is->enz = neg16(ee.dz);
    }
  } else {
    t = fmul(dot, r);
    if(t >= 0) {
      is->entry = inside ? 0 : t;
      is->exit = inside ? t : DEPTH_INF;
      is->enx = nx; is->eny = ny; is->enz = nz;
      is->xnx = neg16(nx); is->xny = neg16(ny); is->xnz = neg16(nz);
    }
  }
}

static inline int dir_eff_zero(int16_t d) {
  uint8_t h = (uint8_t)((uint16_t)d >> 8);
  return h == 0x00 || h == 0xFF;
}

#define AXIS_N(d, neg) ((int16_t)((((uint16_t)(d) >> 15) ^ (neg)) ? 0xC000 : 0x4000))

static SRT_HOT void isect_aabb(uint64_t w, isect_t *is) {
  int32_t t0px, t0py, t0pz, t1px, t1py, t1pz;
  int32_t t0x, t0y, t0z, t1x, t1y, t1z;
  int32_t tminx, tminy, tminz, tmaxx, tmaxy, tmaxz, exitd;
  is->kind = 2;
  is->enx = is->eny = is->enz = 0;
  is->xnx = is->xny = is->xnz = 0;
  t0px = sub32(add32(cf8_1((uint32_t)(w >> 8) & 0x1FF), ee.ox), ee.sx);
  t0py = sub32(add32(cf8_1((uint32_t)(w >> 17) & 0x1FF), ee.oy), ee.sy);
  t0pz = sub32(add32(cf8_1((uint32_t)(w >> 26) & 0x1FF), ee.oz), ee.sz);
  t1px = sub32(add32(cf8_1((uint32_t)(w >> 35) & 0x1FF), ee.ox), ee.sx);
  t1py = sub32(add32(cf8_1((uint32_t)(w >> 44) & 0x1FF), ee.oy), ee.sy);
  t1pz = sub32(add32(cf8_1((uint32_t)(w >> 53) & 0x1FF), ee.oz), ee.sz);
  t0x = fmul48(t0px, ee.rcpx); t1x = fmul48(t1px, ee.rcpx);
  t0y = fmul48(t0py, ee.rcpy); t1y = fmul48(t1py, ee.rcpy);
  t0z = fmul48(t0pz, ee.rcpz); t1z = fmul48(t1pz, ee.rcpz);
  tminx = t0x < t1x ? t0x : t1x;  tmaxx = t0x > t1x ? t0x : t1x;
  tminy = t0y < t1y ? t0y : t1y;  tmaxy = t0y > t1y ? t0y : t1y;
  tminz = t0z < t1z ? t0z : t1z;  tmaxz = t0z > t1z ? t0z : t1z;
  if(tmaxx < tmaxy && tmaxx < tmaxz) {
    exitd = tmaxx; is->xnx = AXIS_N(ee.dx, 0);
  } else if(tmaxy < tmaxz) {
    exitd = tmaxy; is->xny = AXIS_N(ee.dy, 0);
  } else {
    exitd = tmaxz; is->xnz = AXIS_N(ee.dz, 0);
  }
  if((exitd < 0)
     || (dir_eff_zero(ee.dx) && ((t0px >= 0) || (t1px < 0)))
     || (dir_eff_zero(ee.dy) && ((t0py >= 0) || (t1py < 0)))
     || (dir_eff_zero(ee.dz) && ((t0pz >= 0) || (t1pz < 0)))) {
    is->entry = DEPTH_INF;
    is->exit = 0;
  } else if(tminx > tminy && tminx > tminz) {
    is->entry = tminx > 0 ? tminx : 0;
    is->exit = exitd;
    is->enx = AXIS_N(ee.dx, 1);
  } else if(tminy > tminz) {
    is->entry = tminy > 0 ? tminy : 0;
    is->exit = exitd;
    is->eny = AXIS_N(ee.dy, 1);
  } else {
    is->entry = tminz > 0 ? tminz : 0;
    is->exit = exitd;
    is->enz = AXIS_N(ee.dz, 1);
  }
}

static inline void reset_hit_state(void) {
  ee.hit_entry = DEPTH_INF;
  ee.hit_exit = 0;
}

/* run the command list for the current ray. Returns when End executes (or
   a shadow ray registers a hit). */
static SRT_HOT void exec_run(void) {
  uint32_t pc = 0;
  uint32_t budget = SRT_MAX_INSTRUCTIONS_PER_RAY;
  isect_t is;
  while(budget--) {
    uint64_t w = cmd[pc & (SRT_CMDBUF_WORDS - 1)];
    uint32_t op = (uint32_t)w & 0x3F;
    int exec;
    pc = (pc + 1) & 0xFFFF;

    switch((uint32_t)(w >> 6) & 3) {
      default:
      case C_AL:  exec = 1; break;
      case C_OH:  exec = ee.hit_entry < ee.hit_exit; break;
      case C_NH:  exec = !(ee.hit_entry < ee.hit_exit); break;
      case C_ORH: exec = ee.obj_reg_hit; break;
    }

    switch(op) {
      case I_Start:
        /* origin is reset in stage 3 regardless of the condition */
        ee.ox = ee.oy = ee.oz = 0;
        if(exec) {
          reset_hit_state();
          ee.obj_reg_hit = 0;
          ee.reg_hit = 0;
        }
        break;

      case I_Origin:
        ee.ox = cf8_7((uint32_t)(w >> 8) & 0x7FFF);
        ee.oy = cf8_7((uint32_t)(w >> 23) & 0x7FFF);
        ee.oz = cf8_7((uint32_t)(w >> 38) & 0x7FFF);
        break;

      case I_Sphere: case I_SphereSub: case I_SphereAnd:
      case I_Plane:  case I_PlaneSub:  case I_PlaneAnd:
      case I_AABB:   case I_AABBSub:   case I_AABBAnd: {
        int32_t entry, exitd, newentry;
        if(!exec) break;
        srt_stats.instructions++;
        if(op >= I_AABB) isect_aabb(w, &is);
        else if(op & 1) isect_sphere(w, &is);
        else isect_plane(w, &is);
        entry = is.entry; exitd = is.exit;
        newentry = ee.hit_entry;
        if(op == I_Sphere || op == I_Plane || op == I_AABB) {
          if(entry < exitd) {
            if(ee.hit_entry >= ee.hit_exit) {
              newentry = entry;
              ee.hit_exit = exitd;
              entry_normal(&is, &ee.hit_nx, &ee.hit_ny, &ee.hit_nz);
            } else {
              if(entry < ee.hit_entry) {
                newentry = entry;
                entry_normal(&is, &ee.hit_nx, &ee.hit_ny, &ee.hit_nz);
              }
              if(exitd > ee.hit_exit) ee.hit_exit = exitd;
            }
          }
        } else if(op == I_SphereSub || op == I_PlaneSub || op == I_AABBSub) {
          if(ee.hit_entry < ee.hit_exit) {
            if((entry <= ee.hit_entry) && (exitd >= ee.hit_exit)) {
              newentry = DEPTH_INF;
              ee.hit_exit = 0;
            } else if((entry < ee.hit_entry) && (exitd > ee.hit_entry) && (exitd <= ee.hit_exit)) {
              int16_t nx, ny, nz;
              newentry = exitd;
              exit_normal(&is, &nx, &ny, &nz);
              ee.hit_nx = neg16(nx); ee.hit_ny = neg16(ny); ee.hit_nz = neg16(nz);
            } else if((entry > ee.hit_entry) && (entry < ee.hit_exit) && (exitd >= ee.hit_exit)) {
              ee.hit_exit = entry;
            }
          }
        } else { /* And */
          if(ee.hit_entry < ee.hit_exit) {
            if(entry >= exitd) {
              newentry = DEPTH_INF;
              ee.hit_exit = 0;
            } else {
              if(entry > ee.hit_entry) {
                newentry = entry;
                entry_normal(&is, &ee.hit_nx, &ee.hit_ny, &ee.hit_nz);
              }
              if(exitd < ee.hit_exit) ee.hit_exit = exitd;
            }
          }
        }
        ee.hit_entry = newentry;
        ee.hit_pos_depth = newentry;
        ee.hit_pos_pending = 1;
        break;
      }

      case I_Checkerboard:
        if(exec) {
          hit_pos_resolve();
          if(((ee.hit_x ^ ee.hit_z) >> FIXED_SHIFT) & 1) {
            ee.reg_albedo = (uint16_t)(w >> 16);
            ee.reg_refl = (uint8_t)(w >> 8);
          }
        }
        break;

      case I_RegisterHit:
      case I_RegisterHitNoReset:
        if(exec) {
          ee.obj_reg_hit = 0;
          if(ee.hit_entry < ee.hit_exit) {
            int done = 0;
            if(!ee.reg_hit || (ee.hit_entry < ee.reg_depth)) {
              hit_pos_resolve();
              ee.reg_hit = 1;
              ee.reg_depth = ee.hit_entry;
              ee.reg_x = ee.hit_x; ee.reg_y = ee.hit_y; ee.reg_z = ee.hit_z;
              ee.reg_nx = ee.hit_nx; ee.reg_ny = ee.hit_ny; ee.reg_nz = ee.hit_nz;
              ee.reg_albedo = (uint16_t)(w >> 16);
              ee.reg_refl = (uint8_t)(w >> 8);
              ee.obj_reg_hit = 1;
              if(ee.shadow) done = 1;
            }
            if(op != I_RegisterHitNoReset) reset_hit_state();
            if(done) return;
          }
        }
        break;

      case I_ResetHitState:
        if(exec) reset_hit_state();
        break;

      case I_Jump:
      case I_ResetHitStateAndJump:
        if(op == I_ResetHitStateAndJump) reset_hit_state();
        if(exec) pc = (uint32_t)(w >> 8) & 0xFFFF;
        break;

      case I_End:
        if(exec) return;
        break;

      default:
        break;
    }
  }
}

/* ------------------------------------------------------------------------- */
/* Ray engine (RayEngine.sv)                                                 */
/* ------------------------------------------------------------------------- */

#define MIN_TRACE_START ((int32_t)0xFFF60000)
#define MAX_TRACE_START ((int32_t)0x000A0000)

enum { PH_PRIMARY = 0, PH_PRIMARY_SHADOW, PH_SECONDARY, PH_SECONDARY_SHADOW };

static srt_params_t P;

/* ray engine registers */
static struct {
  int16_t pdx, pdy, pdz;           /* primary ray dir */
  int32_t rsx, rsy, rsz;           /* current ray */
  int16_t rdx, rdy, rdz;

  int p_hit; int32_t p_depth, p_x, p_y, p_z; int16_t p_nx, p_ny, p_nz;
  uint16_t p_albedo; uint8_t p_refl; int p_shadow;
  int s_hit; int32_t s_depth, s_x, s_y, s_z; int16_t s_nx, s_ny, s_nz;
  uint16_t s_albedo; int s_shadow;

  uint8_t pr, pg, pb, sr, sg, sb;  /* primary / secondary ray colours */
} R;

static void sky_colour(int16_t dx, int16_t dy, int16_t dz, uint8_t *r, uint8_t *g, uint8_t *b) {
  int16_t sundot, sf, sf2, lerp;
  uint8_t suncol;
  uint32_t base, cr, cb;
  sundot = (int16_t)(fmul16(dx, P.light_x) + fmul16(dy, P.light_y) + fmul16(dz, P.light_z));
  if(sundot < 0) sundot = 0;
  sf = (int16_t)fmul(sundot, sundot);
  sf2 = (int16_t)fmul(sf, sf);
  suncol = (uint8_t)((uint16_t)sf2 >> (FIXED_SHIFT - 7));
  lerp = (int16_t)(dy + 0x4000);
  if(lerp < 0) lerp = 0;
  base = ((128u * (uint32_t)lerp) & 0xFFFFFF) >> FIXED_SHIFT;
  cr = (base + suncol) & 0x1FF;
  cb = (190u + suncol) & 0x1FF;
  *r = (cr & 0x100) ? 255 : (uint8_t)cr;
  *g = *r;
  *b = (cb & 0x100) ? 255 : (uint8_t)cb;
}

/* RayEngine-ColourCalculator.sv */
static void colour_calc(int primary, uint8_t *r, uint8_t *g, uint8_t *b) {
  int hit = primary ? R.p_hit : R.s_hit;
  if(hit) {
    int16_t nx = primary ? R.p_nx : R.s_nx;
    int16_t ny = primary ? R.p_ny : R.s_ny;
    int16_t nz = primary ? R.p_nz : R.s_nz;
    int shadow = primary ? R.p_shadow : R.s_shadow;
    uint16_t albedo = primary ? R.p_albedo : R.s_albedo;
    int16_t base, ndr, sdx, sdy, sdz, sdot, a, bb, c;
    uint8_t illum, spec, ar, ag, ab;

    base = (int16_t)((int16_t)fmul16(nx, P.light_x) + (int16_t)fmul16(ny, P.light_y) + (int16_t)fmul16(nz, P.light_z));
    if(base > 0x3FFF) base = 0x3FFF;
    if(base < 0) {
      illum = (uint8_t)((((uint16_t)neg16(base)) >> 2) >> (FIXED_SHIFT - 8));
    } else if(shadow) {
      illum = (uint8_t)((((uint16_t)base) >> 3) >> (FIXED_SHIFT - 8));
    } else {
      illum = (uint8_t)(((uint16_t)base) >> (FIXED_SHIFT - 8));
    }

    if(base < 0 || shadow) {
      spec = 0;
    } else {
      ndr = (int16_t)(fmul16(R.pdx, R.p_nx) + fmul16(R.pdy, R.p_ny) + fmul16(R.pdz, R.p_nz));
      sdx = (int16_t)(R.pdx - (int16_t)(fmul16(R.p_nx, ndr) << 1));
      sdy = (int16_t)(R.pdy - (int16_t)(fmul16(R.p_ny, ndr) << 1));
      sdz = (int16_t)(R.pdz - (int16_t)(fmul16(R.p_nz, ndr) << 1));
      sdot = (int16_t)((int16_t)fmul16(sdx, P.light_x) + (int16_t)fmul16(sdy, P.light_y) + (int16_t)fmul16(sdz, P.light_z));
      if(sdot < 0) sdot = 0;
      else if(sdot > 0x3FFF) sdot = 0x3FFF;
      a = (int16_t)fmul16(sdot, sdot);
      bb = (int16_t)fmul16(a, a);
      c = (int16_t)fmul16(bb, bb);
      spec = (uint8_t)(((uint16_t)c) >> (FIXED_SHIFT - 8));
    }

    ar = (uint8_t)((albedo & 0x1F) << 3);
    ag = (uint8_t)(((albedo >> 5) & 0x1F) << 3);
    ab = (uint8_t)(((albedo >> 10) & 0x3F) << 3);
    *r = cadd(cmul(ar, illum), spec);
    *g = cadd(cmul(ag, illum), spec);
    *b = cadd(cmul(ab, illum), spec);
  } else {
    sky_colour(R.rdx, R.rdy, R.rdz, r, g, b);
  }
}

static void run_phase(int phase) {
  ee.sx = R.rsx; ee.sy = R.rsy; ee.sz = R.rsz;
  ee.dx = R.rdx; ee.dy = R.rdy; ee.dz = R.rdz;
  ee.rcpx = rcp_hw(R.rdx);
  ee.rcpy = rcp_hw(R.rdy);
  ee.rcpz = rcp_hw(R.rdz);
  ee.shadow = (phase == PH_PRIMARY_SHADOW) || (phase == PH_SECONDARY_SHADOW);
  srt_stats.rays++;
  exec_run();
  /* the hit position registers are only stale copies once the ray is done;
     materialise any pending value using this ray's parameters */
  hit_pos_resolve();
  switch(phase) {
    case PH_PRIMARY:
      R.p_hit = ee.reg_hit; R.p_depth = ee.reg_depth;
      R.p_x = ee.reg_x; R.p_y = ee.reg_y; R.p_z = ee.reg_z;
      R.p_nx = ee.reg_nx; R.p_ny = ee.reg_ny; R.p_nz = ee.reg_nz;
      R.p_albedo = ee.reg_albedo; R.p_refl = ee.reg_refl;
      break;
    case PH_PRIMARY_SHADOW:
      R.p_shadow = ee.reg_hit;
      break;
    case PH_SECONDARY:
      R.s_hit = ee.reg_hit; R.s_depth = ee.reg_depth;
      R.s_x = ee.reg_x; R.s_y = ee.reg_y; R.s_z = ee.reg_z;
      R.s_nx = ee.reg_nx; R.s_ny = ee.reg_ny; R.s_nz = ee.reg_nz;
      R.s_albedo = ee.reg_albedo;
      break;
    default:
      R.s_shadow = ee.reg_hit;
      break;
  }
}

static const uint8_t dither_pattern[8] = { 0, 4, 2, 6, 3, 7, 1, 5 };

static uint16_t trace_pixel(int x, int y, int16_t cdx, int16_t cdy, int16_t cdz) {
  int phase = PH_PRIMARY;
  int16_t lensq, rl;
  uint8_t refl, inv, dith, fr, fg, fb;

  /* Renderer.sv FixedNormalise16Bit */
  lensq = (int16_t)((int16_t)fmul16(cdx, cdx) + (int16_t)fmul(cdy, cdy) + (int16_t)fmul(cdz, cdz));
  rl = (int16_t)rsqrt_fn(lensq);
  R.pdx = (int16_t)fmul16(cdx, rl);
  R.pdy = (int16_t)fmul16(cdy, rl);
  R.pdz = (int16_t)fmul16(cdz, rl);

  R.rsx = P.ray_start_x; R.rsy = P.ray_start_y; R.rsz = P.ray_start_z;
  R.rdx = R.pdx; R.rdy = R.pdy; R.rdz = R.pdz;
  R.p_hit = 0; R.p_shadow = 0; R.s_hit = 0; R.s_shadow = 0;

  for(;;) {
    int primary, next_phase;
    int16_t ndl;
    uint8_t cr, cg, cb;

    /* EES_StartingPhase */
    if(!((R.rsx < MIN_TRACE_START) || (R.rsx > MAX_TRACE_START) ||
         (R.rsy < MIN_TRACE_START) || (R.rsy > MAX_TRACE_START) ||
         (R.rsz < MIN_TRACE_START) || (R.rsz > MAX_TRACE_START))) {
      run_phase(phase);
    }

    /* EES_FinishingPhase: all right hand sides use the current registers */
    primary = (phase == PH_PRIMARY) || (phase == PH_PRIMARY_SHADOW);
    colour_calc(primary, &cr, &cg, &cb);
    if(primary) { R.pr = cr; R.pg = cg; R.pb = cb; }
    else        { R.sr = cr; R.sg = cg; R.sb = cb; }

    ndl = (int16_t)((int16_t)fmul16(P.light_x, primary ? R.p_nx : R.s_nx)
                  + (int16_t)fmul16(P.light_y, primary ? R.p_ny : R.s_ny)
                  + (int16_t)fmul16(P.light_z, primary ? R.p_nz : R.s_nz));
    next_phase = -1;

    switch(phase) {
      case PH_PRIMARY:
        if(R.p_hit) {
          if(ndl > 0) {
            next_phase = PH_PRIMARY_SHADOW;
          } else if(R.p_refl != 0) {
            R.p_shadow = 1;
            next_phase = PH_SECONDARY;
          } else {
            R.p_shadow = 1;
          }
        }
        break;
      case PH_PRIMARY_SHADOW:
        if(R.p_refl != 0) next_phase = PH_SECONDARY;
        break;
      case PH_SECONDARY:
        if(R.s_hit) {
          if(ndl > 0) {
            next_phase = PH_SECONDARY_SHADOW;
          } else {
            R.s_shadow = 1;
          }
        }
        break;
      default:
        break;
    }

    if(next_phase < 0) break;

    if(next_phase == PH_PRIMARY_SHADOW || next_phase == PH_SECONDARY_SHADOW) {
      /* shadow ray from the hit of the current phase towards the light */
      int32_t hx = primary ? R.p_x : R.s_x;
      int32_t hy = primary ? R.p_y : R.s_y;
      int32_t hz = primary ? R.p_z : R.s_z;
      R.rsx = add32(hx, P.light_x >> 4);
      R.rsy = add32(hy, P.light_y >> 4);
      R.rsz = add32(hz, P.light_z >> 4);
      R.rdx = P.light_x; R.rdy = P.light_y; R.rdz = P.light_z;
    } else {
      /* reflection ray */
      int16_t ndr = (int16_t)((int16_t)fmul16(R.pdx, R.p_nx) + (int16_t)fmul16(R.pdy, R.p_ny) + (int16_t)fmul16(R.pdz, R.p_nz));
      R.rsx = add32(R.p_x, R.p_nx >> 4);
      R.rsy = add32(R.p_y, R.p_ny >> 4);
      R.rsz = add32(R.p_z, R.p_nz >> 4);
      R.rdx = (int16_t)(R.pdx - (int16_t)(fmul16(R.p_nx, ndr) << 1));
      R.rdy = (int16_t)(R.pdy - (int16_t)(fmul16(R.p_ny, ndr) << 1));
      R.rdz = (int16_t)(R.pdz - (int16_t)(fmul16(R.p_nz, ndr) << 1));
    }
    phase = next_phase;
  }

  /* RayEngine-FinalColourCalculator.sv */
  if(R.p_hit) { refl = R.p_refl; inv = (uint8_t)(0xFF - R.p_refl); }
  else        { refl = 0; inv = 0xFF; }
  dith = dither_pattern[((y & 3) << 1) | (x & 1)];
  fr = cadd(cadd(cmul(R.pr, inv), cmul(R.sr, refl)), dith);
  fg = cadd(cadd(cmul(R.pg, inv), cmul(R.sg, refl)), dith);
  fb = cadd(cadd(cmul(R.pb, inv), cmul(R.sb, refl)), dith);
  return (uint16_t)(((fb >> 3) << 10) | ((fg >> 3) << 5) | (fr >> 3));
}

void srt_frame_begin(const srt_params_t *params, const uint64_t *cmdbuf) {
  P = *params;
  cmd = cmdbuf;
  srt_stats.instructions = 0;
  srt_stats.rays = 0;
}

static inline void pixel_dir(int x, int y, int16_t *dx, int16_t *dy, int16_t *dz) {
  /* the hardware accumulates the steps in 16 bit registers */
  *dx = (int16_t)(P.ray_dir_x + y * P.ystep_x + x * P.xstep_x);
  *dy = (int16_t)(P.ray_dir_y + y * P.ystep_y + x * P.xstep_y);
  *dz = (int16_t)(P.ray_dir_z + y * P.ystep_z + x * P.xstep_z);
}

uint16_t srt_render_pixel(int x, int y) {
  int16_t dx, dy, dz;
  pixel_dir(x, y, &dx, &dy, &dz);
  return trace_pixel(x, y, dx, dy, dz);
}

void srt_render_line(int y, uint16_t *out) {
  int x;
  int16_t dx, dy, dz;
  pixel_dir(0, y, &dx, &dy, &dz);
  for(x = 0; x < SRT_SCREEN_WIDTH; x++) {
    out[x] = trace_pixel(x, y, dx, dy, dz);
    dx = (int16_t)(dx + P.xstep_x);
    dy = (int16_t)(dy + P.xstep_y);
    dz = (int16_t)(dz + P.xstep_z);
  }
}

void srt_render_line_half(int y, uint16_t *out) {
  int x;
  int16_t dx, dy, dz;
  pixel_dir(0, y, &dx, &dy, &dz);
  for(x = 0; x < SRT_SCREEN_WIDTH; x += 2) {
    /* x / 2 for the ordered dither (x is only used for that), so the doubled
       pixels alternate like the pixels of a full resolution line */
    out[x >> 1] = trace_pixel(x >> 1, y, dx, dy, dz);
    /* the engine adds the doubled step in its 16 bit registers */
    dx = (int16_t)(dx + 2 * P.xstep_x);
    dy = (int16_t)(dy + 2 * P.xstep_y);
    dz = (int16_t)(dz + 2 * P.xstep_z);
  }
}
