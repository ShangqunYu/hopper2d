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
  #define CASADI_PREFIX(ID) A_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[21] = {17, 1, 0, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const casadi_int casadi_s2[43] = {6, 6, 0, 5, 10, 16, 22, 28, 34, 0, 2, 3, 4, 5, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};

/* A:(i0[12],i1[17])->(o0[6x6,34nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][4] : 0;
  a1=arg[1]? arg[1][3] : 0;
  a2=(a0+a1);
  a3=arg[1]? arg[1][2] : 0;
  a2=(a2+a3);
  a4=arg[1]? arg[1][0] : 0;
  a2=(a2+a4);
  if (res[0]!=0) res[0][0]=a2;
  a2=arg[1]? arg[1][11] : 0;
  a5=arg[0]? arg[0][2] : 0;
  a6=arg[0]? arg[0][3] : 0;
  a7=(a5+a6);
  a8=cos(a7);
  a9=(a2*a8);
  a10=arg[1]? arg[1][12] : 0;
  a11=(a5+a6);
  a12=arg[0]? arg[0][4] : 0;
  a11=(a11+a12);
  a13=cos(a11);
  a14=(a10*a13);
  a9=(a9+a14);
  a14=arg[1]? arg[1][10] : 0;
  a15=(a5+a6);
  a15=(a15+a12);
  a16=arg[0]? arg[0][5] : 0;
  a15=(a15+a16);
  a17=1.5707963267948966e+00;
  a15=(a15+a17);
  a15=cos(a15);
  a18=(a14*a15);
  a9=(a9+a18);
  a18=(a9*a0);
  a19=cos(a7);
  a20=(a2*a19);
  a21=arg[1]? arg[1][9] : 0;
  a11=cos(a11);
  a22=(a21*a11);
  a20=(a20+a22);
  a22=(a20*a1);
  a18=(a18+a22);
  a22=arg[1]? arg[1][8] : 0;
  a7=cos(a7);
  a23=(a22*a7);
  a24=(a23*a3);
  a18=(a18+a24);
  if (res[0]!=0) res[0][1]=a18;
  a8=(a2*a8);
  a18=(a10*a13);
  a8=(a8+a18);
  a18=(a14*a15);
  a8=(a8+a18);
  a18=(a8*a0);
  a19=(a2*a19);
  a24=(a21*a11);
  a19=(a19+a24);
  a24=(a19*a1);
  a18=(a18+a24);
  a7=(a22*a7);
  a24=(a7*a3);
  a18=(a18+a24);
  if (res[0]!=0) res[0][2]=a18;
  a13=(a10*a13);
  a18=(a14*a15);
  a13=(a13+a18);
  a18=(a13*a0);
  a11=(a21*a11);
  a24=(a11*a1);
  a18=(a18+a24);
  if (res[0]!=0) res[0][3]=a18;
  a15=(a14*a15);
  a18=(a15*a0);
  if (res[0]!=0) res[0][4]=a18;
  a18=(a0+a1);
  a18=(a18+a3);
  a18=(a18+a4);
  if (res[0]!=0) res[0][5]=a18;
  a18=(a5+a6);
  a4=sin(a18);
  a24=(a2*a4);
  a25=(a5+a6);
  a25=(a25+a12);
  a26=sin(a25);
  a27=(a10*a26);
  a24=(a24+a27);
  a5=(a5+a6);
  a5=(a5+a12);
  a5=(a5+a16);
  a5=(a5+a17);
  a5=sin(a5);
  a17=(a14*a5);
  a24=(a24+a17);
  a17=(a24*a0);
  a16=sin(a18);
  a12=(a2*a16);
  a25=sin(a25);
  a6=(a21*a25);
  a12=(a12+a6);
  a6=(a12*a1);
  a17=(a17+a6);
  a18=sin(a18);
  a6=(a22*a18);
  a27=(a6*a3);
  a17=(a17+a27);
  if (res[0]!=0) res[0][6]=a17;
  a4=(a2*a4);
  a17=(a10*a26);
  a4=(a4+a17);
  a17=(a14*a5);
  a4=(a4+a17);
  a17=(a4*a0);
  a2=(a2*a16);
  a16=(a21*a25);
  a2=(a2+a16);
  a16=(a2*a1);
  a17=(a17+a16);
  a22=(a22*a18);
  a18=(a22*a3);
  a17=(a17+a18);
  if (res[0]!=0) res[0][7]=a17;
  a10=(a10*a26);
  a26=(a14*a5);
  a10=(a10+a26);
  a26=(a10*a0);
  a21=(a21*a25);
  a25=(a21*a1);
  a26=(a26+a25);
  if (res[0]!=0) res[0][8]=a26;
  a14=(a14*a5);
  a5=(a14*a0);
  if (res[0]!=0) res[0][9]=a5;
  a5=5.0000000000000000e-01;
  a26=(a5*a0);
  a25=(a9+a9);
  a25=(a26*a25);
  a1=(a5*a1);
  a17=(a20+a20);
  a17=(a1*a17);
  a18=(a25+a17);
  a5=(a5*a3);
  a3=(a23+a23);
  a3=(a5*a3);
  a18=(a18+a3);
  if (res[0]!=0) res[0][10]=a18;
  a18=(a24+a24);
  a18=(a26*a18);
  a16=(a12+a12);
  a16=(a1*a16);
  a27=(a18+a16);
  a28=(a6+a6);
  a28=(a5*a28);
  a27=(a27+a28);
  if (res[0]!=0) res[0][11]=a27;
  a27=arg[1]? arg[1][7] : 0;
  a29=(a27+a0);
  a30=(a24*a18);
  a29=(a29+a30);
  a30=(a9*a25);
  a29=(a29+a30);
  a30=arg[1]? arg[1][6] : 0;
  a29=(a29+a30);
  a31=(a12*a16);
  a29=(a29+a31);
  a31=(a20*a17);
  a29=(a29+a31);
  a31=arg[1]? arg[1][5] : 0;
  a29=(a29+a31);
  a32=(a6*a28);
  a29=(a29+a32);
  a32=(a23*a3);
  a29=(a29+a32);
  a32=arg[1]? arg[1][1] : 0;
  a29=(a29+a32);
  if (res[0]!=0) res[0][12]=a29;
  a29=(a27+a0);
  a32=(a4*a18);
  a29=(a29+a32);
  a32=(a8*a25);
  a29=(a29+a32);
  a29=(a29+a30);
  a32=(a2*a16);
  a29=(a29+a32);
  a32=(a19*a17);
  a29=(a29+a32);
  a29=(a29+a31);
  a28=(a22*a28);
  a29=(a29+a28);
  a3=(a7*a3);
  a29=(a29+a3);
  if (res[0]!=0) res[0][13]=a29;
  a29=(a27+a0);
  a3=(a10*a18);
  a29=(a29+a3);
  a3=(a13*a25);
  a29=(a29+a3);
  a29=(a29+a30);
  a16=(a21*a16);
  a29=(a29+a16);
  a17=(a11*a17);
  a29=(a29+a17);
  if (res[0]!=0) res[0][14]=a29;
  a29=(a27+a0);
  a18=(a14*a18);
  a29=(a29+a18);
  a25=(a15*a25);
  a29=(a29+a25);
  if (res[0]!=0) res[0][15]=a29;
  a29=(a8+a8);
  a29=(a26*a29);
  a25=(a19+a19);
  a25=(a1*a25);
  a18=(a29+a25);
  a17=(a7+a7);
  a17=(a5*a17);
  a18=(a18+a17);
  if (res[0]!=0) res[0][16]=a18;
  a18=(a4+a4);
  a18=(a26*a18);
  a16=(a2+a2);
  a16=(a1*a16);
  a3=(a18+a16);
  a28=(a22+a22);
  a5=(a5*a28);
  a3=(a3+a5);
  if (res[0]!=0) res[0][17]=a3;
  a3=(a27+a0);
  a28=(a24*a18);
  a3=(a3+a28);
  a28=(a9*a29);
  a3=(a3+a28);
  a3=(a3+a30);
  a28=(a12*a16);
  a3=(a3+a28);
  a28=(a20*a25);
  a3=(a3+a28);
  a3=(a3+a31);
  a6=(a6*a5);
  a3=(a3+a6);
  a23=(a23*a17);
  a3=(a3+a23);
  if (res[0]!=0) res[0][18]=a3;
  a3=(a27+a0);
  a23=(a4*a18);
  a3=(a3+a23);
  a23=(a8*a29);
  a3=(a3+a23);
  a3=(a3+a30);
  a23=(a2*a16);
  a3=(a3+a23);
  a23=(a19*a25);
  a3=(a3+a23);
  a3=(a3+a31);
  a22=(a22*a5);
  a3=(a3+a22);
  a7=(a7*a17);
  a3=(a3+a7);
  if (res[0]!=0) res[0][19]=a3;
  a3=(a27+a0);
  a7=(a10*a18);
  a3=(a3+a7);
  a7=(a13*a29);
  a3=(a3+a7);
  a3=(a3+a30);
  a16=(a21*a16);
  a3=(a3+a16);
  a25=(a11*a25);
  a3=(a3+a25);
  if (res[0]!=0) res[0][20]=a3;
  a3=(a27+a0);
  a18=(a14*a18);
  a3=(a3+a18);
  a29=(a15*a29);
  a3=(a3+a29);
  if (res[0]!=0) res[0][21]=a3;
  a3=(a13+a13);
  a3=(a26*a3);
  a29=(a11+a11);
  a29=(a1*a29);
  a18=(a3+a29);
  if (res[0]!=0) res[0][22]=a18;
  a18=(a10+a10);
  a18=(a26*a18);
  a25=(a21+a21);
  a1=(a1*a25);
  a25=(a18+a1);
  if (res[0]!=0) res[0][23]=a25;
  a25=(a27+a0);
  a16=(a24*a18);
  a25=(a25+a16);
  a16=(a9*a3);
  a25=(a25+a16);
  a25=(a25+a30);
  a12=(a12*a1);
  a25=(a25+a12);
  a20=(a20*a29);
  a25=(a25+a20);
  if (res[0]!=0) res[0][24]=a25;
  a25=(a27+a0);
  a20=(a4*a18);
  a25=(a25+a20);
  a20=(a8*a3);
  a25=(a25+a20);
  a25=(a25+a30);
  a2=(a2*a1);
  a25=(a25+a2);
  a19=(a19*a29);
  a25=(a25+a19);
  if (res[0]!=0) res[0][25]=a25;
  a25=(a27+a0);
  a19=(a10*a18);
  a25=(a25+a19);
  a19=(a13*a3);
  a25=(a25+a19);
  a25=(a25+a30);
  a21=(a21*a1);
  a25=(a25+a21);
  a11=(a11*a29);
  a25=(a25+a11);
  if (res[0]!=0) res[0][26]=a25;
  a25=(a27+a0);
  a18=(a14*a18);
  a25=(a25+a18);
  a3=(a15*a3);
  a25=(a25+a3);
  if (res[0]!=0) res[0][27]=a25;
  a25=(a15+a15);
  a25=(a26*a25);
  if (res[0]!=0) res[0][28]=a25;
  a3=(a14+a14);
  a26=(a26*a3);
  if (res[0]!=0) res[0][29]=a26;
  a3=(a27+a0);
  a24=(a24*a26);
  a3=(a3+a24);
  a9=(a9*a25);
  a3=(a3+a9);
  if (res[0]!=0) res[0][30]=a3;
  a3=(a27+a0);
  a4=(a4*a26);
  a3=(a3+a4);
  a8=(a8*a25);
  a3=(a3+a8);
  if (res[0]!=0) res[0][31]=a3;
  a3=(a27+a0);
  a10=(a10*a26);
  a3=(a3+a10);
  a13=(a13*a25);
  a3=(a3+a13);
  if (res[0]!=0) res[0][32]=a3;
  a27=(a27+a0);
  a14=(a14*a26);
  a27=(a27+a14);
  a15=(a15*a25);
  a27=(a27+a15);
  if (res[0]!=0) res[0][33]=a27;
  return 0;
}

CASADI_SYMBOL_EXPORT int A(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int A_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int A_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void A_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int A_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void A_release(int mem) {
}

CASADI_SYMBOL_EXPORT void A_incref(void) {
}

CASADI_SYMBOL_EXPORT void A_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int A_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int A_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real A_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* A_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* A_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* A_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* A_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int A_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
