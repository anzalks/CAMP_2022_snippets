/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__Wghkampa
#define _nrn_initial _nrn_initial__Wghkampa
#define nrn_cur _nrn_cur__Wghkampa
#define _nrn_current _nrn_current__Wghkampa
#define nrn_jacob _nrn_jacob__Wghkampa
#define nrn_state _nrn_state__Wghkampa
#define _net_receive _net_receive__Wghkampa 
#define state state__Wghkampa 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define taur _p[0]
#define taud _p[1]
#define Pmax _p[2]
#define winit _p[3]
#define P _p[4]
#define iampa _p[5]
#define lr _p[6]
#define A _p[7]
#define B _p[8]
#define w _p[9]
#define cai _p[10]
#define ina _p[11]
#define ik _p[12]
#define factor _p[13]
#define Area _p[14]
#define DA _p[15]
#define DB _p[16]
#define Dw _p[17]
#define v _p[18]
#define _g _p[19]
#define _tsav _p[20]
#define _nd_area  *_ppvar[0]._pval
#define _ion_ina	*_ppvar[2]._pval
#define _ion_dinadv	*_ppvar[3]._pval
#define _ion_ik	*_ppvar[4]._pval
#define _ion_dikdv	*_ppvar[5]._pval
#define _ion_cai	*_ppvar[6]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static double _hoc_Omega();
 static double _hoc_efun();
 static double _hoc_eta();
 static double _hoc_ghk();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "Omega", _hoc_Omega,
 "efun", _hoc_efun,
 "eta", _hoc_eta,
 "ghk", _hoc_ghk,
 0, 0
};
#define Omega Omega_Wghkampa
#define efun efun_Wghkampa
#define eta eta_Wghkampa
#define ghk ghk_Wghkampa
 extern double Omega( _threadargsprotocomma_ double );
 extern double efun( _threadargsprotocomma_ double );
 extern double eta( _threadargsprotocomma_ double );
 extern double ghk( _threadargsprotocomma_ double , double , double , double );
 /* declare global and static user variables */
#define alpha2 alpha2_Wghkampa
 double alpha2 = 0.55;
#define alpha1 alpha1_Wghkampa
 double alpha1 = 0.35;
#define beta2 beta2_Wghkampa
 double beta2 = 80;
#define beta1 beta1_Wghkampa
 double beta1 = 80;
#define ko ko_Wghkampa
 double ko = 5;
#define ki ki_Wghkampa
 double ki = 140;
#define nao nao_Wghkampa
 double nao = 140;
#define nai nai_Wghkampa
 double nai = 18;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "taud", 1e-009, 1e+009,
 "taur", 1e-009, 1e+009,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "nai_Wghkampa", "mM",
 "nao_Wghkampa", "mM",
 "ki_Wghkampa", "mM",
 "ko_Wghkampa", "mM",
 "taur", "ms",
 "taud", "ms",
 "Pmax", "cm/s",
 "winit", "1",
 "A", "cm/s",
 "B", "cm/s",
 "w", "1",
 "P", "cm/s",
 "iampa", "nA",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 static double w0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "nai_Wghkampa", &nai_Wghkampa,
 "nao_Wghkampa", &nao_Wghkampa,
 "ki_Wghkampa", &ki_Wghkampa,
 "ko_Wghkampa", &ko_Wghkampa,
 "alpha1_Wghkampa", &alpha1_Wghkampa,
 "beta1_Wghkampa", &beta1_Wghkampa,
 "alpha2_Wghkampa", &alpha2_Wghkampa,
 "beta2_Wghkampa", &beta2_Wghkampa,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[7]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"Wghkampa",
 "taur",
 "taud",
 "Pmax",
 "winit",
 0,
 "P",
 "iampa",
 "lr",
 0,
 "A",
 "B",
 "w",
 0,
 0};
 static Symbol* _na_sym;
 static Symbol* _k_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 21, _prop);
 	/*initialize range parameters*/
 	taur = 2;
 	taud = 10;
 	Pmax = 1e-006;
 	winit = 1;
  }
 	_prop->param = _p;
 	_prop->param_size = 21;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 8, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_na_sym);
 	_ppvar[2]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[3]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_k_sym);
 	_ppvar[4]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[5]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[6]._pval = &prop_ion->param[1]; /* cai */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _Wghkampa_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("k", -10000.);
 	ion_reg("ca", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	_k_sym = hoc_lookup("k_ion");
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 21, 8);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Wghkampa D:/Documents/CAMP 2022/tutorials/Plasticity/Wghkampa.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
 static double R = 8.3145;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[3], _dlist1[3];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   lr = eta ( _threadargscomma_ cai ) ;
   Dw = lr * ( Omega ( _threadargscomma_ cai ) - w ) ;
   DA = - A / taur ;
   DB = - B / taud ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 lr = eta ( _threadargscomma_ cai ) ;
 Dw = Dw  / (1. - dt*( ( lr )*( ( ( - 1.0 ) ) ) )) ;
 DA = DA  / (1. - dt*( ( - 1.0 ) / taur )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / taud )) ;
  return 0;
}
 /*END CVODE*/
 static int state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   lr = eta ( _threadargscomma_ cai ) ;
    w = w + (1. - exp(dt*(( lr )*( ( ( - 1.0 ) ) ))))*(- ( ( lr )*( ( Omega ( _threadargscomma_ cai ) ) ) ) / ( ( lr )*( ( ( - 1.0 ) ) ) ) - w) ;
    A = A + (1. - exp(dt*(( - 1.0 ) / taur)))*(- ( 0.0 ) / ( ( - 1.0 ) / taur ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / taud)))*(- ( 0.0 ) / ( ( - 1.0 ) / taud ) - B) ;
   }
  return 0;
}
 
double ghk ( _threadargsprotocomma_ double _lv , double _lci , double _lco , double _lz ) {
   double _lghk;
 double _larg , _leci , _leco ;
 _larg = ( 0.001 ) * _lz * FARADAY * _lv / ( R * ( celsius + 273.15 ) ) ;
   _leco = _lco * efun ( _threadargscomma_ _larg ) ;
   _leci = _lci * efun ( _threadargscomma_ - _larg ) ;
   _lghk = ( 0.001 ) * _lz * FARADAY * ( _leci - _leco ) ;
   
return _lghk;
 }
 
static double _hoc_ghk(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  ghk ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) );
 return(_r);
}
 
double efun ( _threadargsprotocomma_ double _lz ) {
   double _lefun;
 if ( fabs ( _lz ) < 1e-4 ) {
     _lefun = 1.0 - _lz / 2.0 ;
     }
   else {
     _lefun = _lz / ( exp ( _lz ) - 1.0 ) ;
     }
   
return _lefun;
 }
 
static double _hoc_efun(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  efun ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double eta ( _threadargsprotocomma_ double _lci ) {
   double _leta;
 double _linv , _lP1 , _lP2 , _lP3 , _lP4 ;
 _lP1 = 100.0 ;
   _lP2 = _lP1 * 1e-4 ;
   _lP4 = 1e3 ;
   _lP3 = 3.0 ;
   _lci = ( _lci - 1e-4 ) * 1e3 ;
   _linv = _lP4 + _lP1 / ( _lP2 + _lci * _lci * _lci ) ;
   _leta = 1.0 / _linv ;
   
return _leta;
 }
 
static double _hoc_eta(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  eta ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double Omega ( _threadargsprotocomma_ double _lci ) {
   double _lOmega;
 _lci = ( _lci - 1e-4 ) * 1e3 ;
   _lOmega = 0.25 + 1.0 / ( 1.0 + exp ( - ( _lci - alpha2 ) * beta2 ) ) - 0.25 / ( 1.0 + exp ( - ( _lci - alpha1 ) * beta1 ) ) ;
   
return _lOmega;
 }
 
static double _hoc_Omega(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  Omega ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
       if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A;
    double __primary = (A + Pmax * factor ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / taur ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / taur ) - __primary );
    A += __primary;
  } else {
 A = A + Pmax * factor  ;
     }
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B;
    double __primary = (B + Pmax * factor ) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / taud ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / taud ) - __primary );
    B += __primary;
  } else {
 B = B + Pmax * factor  ;
     }
 } }
 
static int _ode_count(int _type){ return 3;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
   }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 3; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 3, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 4, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 5, 4);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 6, 1);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  A = A0;
  B = B0;
  w = w0;
 {
   double _ltp ;
 if ( taur / taud > .9999 ) {
     taur = .9999 * taud ;
     }
   A = 0.0 ;
   B = 0.0 ;
   _ltp = ( taur * taud ) / ( taud - taur ) * log ( taud / taur ) ;
   factor = - exp ( - _ltp / taur ) + exp ( - _ltp / taud ) ;
   factor = 1.0 / factor ;
   Area = 1.0 ;
   w = winit ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  cai = _ion_cai;
 initmodel(_p, _ppvar, _thread, _nt);
  }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   P = B - A ;
   ina = P * w * ghk ( _threadargscomma_ v , nai , nao , 1.0 ) * Area ;
   ik = P * w * ghk ( _threadargscomma_ v , ki , ko , 1.0 ) * Area ;
   iampa = ik + ina ;
   }
 _current += ina;
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  cai = _ion_cai;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
 double _dina;
  _dina = ina;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dinadv += (_dina - ina)/.001 * 1.e2/ (_nd_area);
  _ion_dikdv += (_dik - ik)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina * 1.e2/ (_nd_area);
  _ion_ik += ik * 1.e2/ (_nd_area);
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  cai = _ion_cai;
 {   state(_p, _ppvar, _thread, _nt);
  }  }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(w) - _p;  _dlist1[0] = &(Dw) - _p;
 _slist1[1] = &(A) - _p;  _dlist1[1] = &(DA) - _p;
 _slist1[2] = &(B) - _p;  _dlist1[2] = &(DB) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "Wghkampa.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "\n"
  "The kinetics part is obtained from Exp2Syn of NEURON.\n"
  "\n"
  "Two state kinetic scheme synapse described by rise time taur, and\n"
  "decay time constant taud. The normalized peak condunductance is 1.\n"
  "Decay time MUST be greater than rise time.\n"
  "\n"
  "The solution of A->G->bath with rate constants 1/taur and 1/taud is\n"
  "\n"
  " A = a*exp(-t/taur) and\n"
  " G = a*taud/(taud-taur)*(-exp(-t/taur) + exp(-t/taud))\n"
  "	where taur < taud\n"
  "\n"
  "If taud-taur -> 0 then we have a alphasynapse.\n"
  "and if taur -> 0 then we have just single exponential decay.\n"
  "\n"
  "The factor is evaluated in the initial block such that an event of\n"
  "weight 1 generates a peak conductance of 1.\n"
  "\n"
  "Because the solution is a sum of exponentials, the coupled equations\n"
  "can be solved as a pair of independent equations by the more efficient\n"
  "cnexp method.\n"
  "\n"
  "Added by Rishikesh Narayanan:\n"
  "\n"
  "1. GHK based ionic currents for AMPA current \n"
  "2. Weights, and their update, according Shouval et al., PNAS, 2002.\n"
  "\n"
  "Details may be found in:\n"
  "\n"
  "Narayanan R, Johnston D. The h current is a candidate mechanism for \n"
  "regulating the sliding modification threshold in a BCM-like synaptic \n"
  "learning rule.  J Neurophysiol. 2010 Aug;104(2):1020-33.\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS Wghkampa\n"
  "	USEION na WRITE ina\n"
  "	USEION k WRITE ik\n"
  "	USEION ca READ cai	: Weight update requires cai \n"
  "	\n"
  "	RANGE taur, taud\n"
  "	RANGE iampa,winit\n"
  "	RANGE P, Pmax, lr\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(nA) = (nanoamp)\n"
  "	(mV) = (millivolt)\n"
  "	(uS) = (microsiemens)\n"
  "	(molar) = (1/liter)\n"
  "	(mM) = (millimolar)\n"
  "	FARADAY = (faraday) (coulomb)\n"
  "	R = (k-mole) (joule/degC)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	taur=2 		(ms) <1e-9,1e9>\n"
  "	taud = 10 	(ms) <1e-9,1e9>\n"
  "	nai = 18	(mM)	: Set for a reversal pot of +55mV\n"
  "	nao = 140	(mM)\n"
  "	ki = 140	(mM)	: Set for a reversal pot of -90mV\n"
  "	ko = 5		(mM)\n"
  "	cai			(mM)\n"
  "	celsius		(degC)\n"
  "	Pmax=1e-6   (cm/s)	\n"
  "	alpha1=0.35	:Parameters for the Omega function.\n"
  "	beta1=80\n"
  "	alpha2=0.55\n"
  "	beta2=80\n"
  "	winit=1		(1)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	ina     (nA)\n"
  "	ik      (nA)\n"
  "	v (mV)\n"
  "	P (cm/s)\n"
  "	factor\n"
  "	iampa	(nA)\n"
  "	lr\n"
  "	Area (cm2)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	A (cm/s)\n"
  "	B (cm/s)\n"
  "	w (1)\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "	LOCAL tp\n"
  "	if (taur/taud > .9999) {\n"
  "		taur = .9999*taud\n"
  "	}\n"
  "	A = 0\n"
  "	B = 0\n"
  "	tp = (taur*taud)/(taud - taur) * log(taud/taur)\n"
  "	factor = -exp(-tp/taur) + exp(-tp/taud)\n"
  "	factor = 1/factor\n"
  "	Area=1\n"
  "	w=winit\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD cnexp\n"
  "	P=B-A\n"
  "\n"
  "	ina = P*w*ghk(v, nai, nao,1)*Area	\n"
  "	ik = P*w*ghk(v, ki, ko,1)*Area\n"
  "	iampa = ik + ina\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
  "	lr=eta(cai)\n"
  "	w' = lr*(Omega(cai)-w)\n"
  "	A' = -A/taur\n"
  "	B' = -B/taud\n"
  "}\n"
  "\n"
  "FUNCTION ghk(v(mV), ci(mM), co(mM),z) (0.001 coul/cm3) {\n"
  "	LOCAL arg, eci, eco\n"
  "	arg = (0.001)*z*FARADAY*v/(R*(celsius+273.15))\n"
  "	eco = co*efun(arg)\n"
  "	eci = ci*efun(-arg)\n"
  "	ghk = (0.001)*z*FARADAY*(eci - eco)\n"
  "}\n"
  "\n"
  "FUNCTION efun(z) {\n"
  "	if (fabs(z) < 1e-4) {\n"
  "		efun = 1 - z/2\n"
  "	}else{\n"
  "		efun = z/(exp(z) - 1)\n"
  "	}\n"
  "}\n"
  "\n"
  "FUNCTION eta(ci (mM)) { : when ci is 0, inv has to be 3 hours.\n"
  "	LOCAL inv, P1, P2, P3, P4\n"
  "	P1=100	\n"
  "	P2=P1*1e-4	: There was a slip in the paper, which says P2=P1/1e-4\n"
  "	P4=1e3\n"
  "	P3=3		: Cube, directly multiplying, see below.\n"
  "\n"
  "	ci=(ci-1e-4)*1e3 	: The function takes uM, and we get mM.\n"
  "\n"
  "	inv=P4 + P1/(P2+ci*ci*ci) :As P3 is 3, set ci^P3 as ci*ci*ci.\n"
  "	eta=1/inv\n"
  "}	\n"
  "\n"
  "FUNCTION Omega(ci (mM)) {\n"
  "	ci=(ci-1e-4)*1e3	: The function takes uM, and we get mM.\n"
  "	Omega=0.25+1/(1+exp(-(ci-alpha2)*beta2))-0.25/(1+exp(-(ci-alpha1)*beta1))\n"
  "}\n"
  "	\n"
  "NET_RECEIVE(weight (uS)) { 	: No use to weight, can be used instead of Pmax,\n"
  "							: if you want NetCon access to the synaptic\n"
  "							: conductance.\n"
  "	state_discontinuity(A, A + Pmax*factor)\n"
  "	state_discontinuity(B, B + Pmax*factor)\n"
  "}\n"
  ;
#endif
