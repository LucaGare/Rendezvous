/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) rk4_UGV_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s1[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* f:(i0[4],i1[2])->(o0[4]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  casadi_real *w0=w+0, w1, *w2=w+5, w3, w4;
  /* #0: @0 = input[0][0] */
  casadi_copy(arg[0], 4, w0);
  /* #1: @1 = @0[2] */
  for (rr=(&w1), ss=w0+2; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #2: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #3: @1 = @0[3] */
  for (rr=(&w1), ss=w0+3; ss!=w0+4; ss+=1) *rr++ = *ss;
  /* #4: output[0][1] = @1 */
  if (res[0]) res[0][1] = w1;
  /* #5: @1 = 11.75 */
  w1 = 1.1749999999999998e+01;
  /* #6: @2 = input[1][0] */
  casadi_copy(arg[1], 2, w2);
  /* #7: @3 = @2[0] */
  for (rr=(&w3), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #8: @1 = (@1*@3) */
  w1 *= w3;
  /* #9: @3 = @0[2] */
  for (rr=(&w3), ss=w0+2; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #10: @4 = 0.08 */
  w4 = 8.0000000000000002e-02;
  /* #11: @3 = (@3/@4) */
  w3 /= w4;
  /* #12: @1 = (@1-@3) */
  w1 -= w3;
  /* #13: output[0][2] = @1 */
  if (res[0]) res[0][2] = w1;
  /* #14: @1 = 11 */
  w1 = 11.;
  /* #15: @3 = @2[1] */
  for (rr=(&w3), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #16: @1 = (@1*@3) */
  w1 *= w3;
  /* #17: @3 = @0[3] */
  for (rr=(&w3), ss=w0+3; ss!=w0+4; ss+=1) *rr++ = *ss;
  /* #18: @4 = 0.08 */
  w4 = 8.0000000000000002e-02;
  /* #19: @3 = (@3/@4) */
  w3 /= w4;
  /* #20: @1 = (@1-@3) */
  w1 -= w3;
  /* #21: output[0][3] = @1 */
  if (res[0]) res[0][3] = w1;
  return 0;
}

/* rk4_UGV:(x0[4],u[2],dt)->(xf[4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real **res1=res+1, *rr;
  const casadi_real **arg1=arg+3, *cr, *cs;
  casadi_real *w0=w+9, w1, w2, *w3=w+15, *w4=w+17, w5, *w6=w+22, *w7=w+26;
  /* #0: @0 = input[0][0] */
  casadi_copy(arg[0], 4, w0);
  /* #1: @1 = input[2][0] */
  w1 = arg[2] ? arg[2][0] : 0;
  /* #2: @2 = 6 */
  w2 = 6.;
  /* #3: @2 = (@1/@2) */
  w2  = (w1/w2);
  /* #4: @3 = input[1][0] */
  casadi_copy(arg[1], 2, w3);
  /* #5: @4 = f(@0, @3) */
  arg1[0]=w0;
  arg1[1]=w3;
  res1[0]=w4;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #6: @5 = 2 */
  w5 = 2.;
  /* #7: @5 = (@1/@5) */
  w5  = (w1/w5);
  /* #8: @6 = (@5*@4) */
  for (i=0, rr=w6, cs=w4; i<4; ++i) (*rr++)  = (w5*(*cs++));
  /* #9: @6 = (@0+@6) */
  for (i=0, rr=w6, cr=w0, cs=w6; i<4; ++i) (*rr++)  = ((*cr++)+(*cs++));
  /* #10: @7 = f(@6, @3) */
  arg1[0]=w6;
  arg1[1]=w3;
  res1[0]=w7;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #11: @6 = (2.*@7) */
  for (i=0, rr=w6, cs=w7; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #12: @4 = (@4+@6) */
  for (i=0, rr=w4, cs=w6; i<4; ++i) (*rr++) += (*cs++);
  /* #13: @5 = 2 */
  w5 = 2.;
  /* #14: @5 = (@1/@5) */
  w5  = (w1/w5);
  /* #15: @7 = (@5*@7) */
  for (i=0, rr=w7, cs=w7; i<4; ++i) (*rr++)  = (w5*(*cs++));
  /* #16: @7 = (@0+@7) */
  for (i=0, rr=w7, cr=w0, cs=w7; i<4; ++i) (*rr++)  = ((*cr++)+(*cs++));
  /* #17: @6 = f(@7, @3) */
  arg1[0]=w7;
  arg1[1]=w3;
  res1[0]=w6;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #18: @7 = (2.*@6) */
  for (i=0, rr=w7, cs=w6; i<4; ++i) *rr++ = (2.* *cs++ );
  /* #19: @4 = (@4+@7) */
  for (i=0, rr=w4, cs=w7; i<4; ++i) (*rr++) += (*cs++);
  /* #20: @6 = (@1*@6) */
  for (i=0, rr=w6, cs=w6; i<4; ++i) (*rr++)  = (w1*(*cs++));
  /* #21: @6 = (@0+@6) */
  for (i=0, rr=w6, cr=w0, cs=w6; i<4; ++i) (*rr++)  = ((*cr++)+(*cs++));
  /* #22: @7 = f(@6, @3) */
  arg1[0]=w6;
  arg1[1]=w3;
  res1[0]=w7;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #23: @4 = (@4+@7) */
  for (i=0, rr=w4, cs=w7; i<4; ++i) (*rr++) += (*cs++);
  /* #24: @4 = (@2*@4) */
  for (i=0, rr=w4, cs=w4; i<4; ++i) (*rr++)  = (w2*(*cs++));
  /* #25: @0 = (@0+@4) */
  for (i=0, rr=w0, cs=w4; i<4; ++i) (*rr++) += (*cs++);
  /* #26: output[0][0] = @0 */
  casadi_copy(w0, 4, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int rk4_UGV(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int rk4_UGV_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int rk4_UGV_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rk4_UGV_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int rk4_UGV_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void rk4_UGV_release(int mem) {
}

CASADI_SYMBOL_EXPORT void rk4_UGV_incref(void) {
}

CASADI_SYMBOL_EXPORT void rk4_UGV_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rk4_UGV_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rk4_UGV_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real rk4_UGV_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rk4_UGV_name_in(casadi_int i){
  switch (i) {
    case 0: return "x0";
    case 1: return "u";
    case 2: return "dt";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rk4_UGV_name_out(casadi_int i){
  switch (i) {
    case 0: return "xf";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rk4_UGV_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rk4_UGV_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rk4_UGV_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 30;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
