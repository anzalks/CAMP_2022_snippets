/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
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
 
#define nrn_init _nrn_init__ghknmda
#define _nrn_initial _nrn_initial__ghknmda
#define nrn_cur _nrn_cur__ghknmda
#define _nrn_current _nrn_current__ghknmda
#define nrn_jacob _nrn_jacob__ghknmda
#define nrn_state _nrn_state__ghknmda
#define _net_receive _net_receive__ghknmda 
#define state state__ghknmda 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define taur _p[0]
#define taud _p[1]
#define mg _p[2]
#define Pmax _p[3]
#define P _p[4]
#define inmda _p[5]
#define A _p[6]
#define B _p[7]
#define cai _p[8]
#define cao _p[9]
#define ina _p[10]
#define ik _p[11]
#define ica _p[12]
#define factor _p[13]
#define Area _p[14]
#define DA _p[15]
#define DB _p[16]
#define _g _p[17]
#define _tsav _p[18]
#define _nd_area  *_ppvar[0]._pval
#define _ion_ina	*_ppvar[2]._pval
#define _ion_dinadv	*_ppvar[3]._pval
#define _ion_ik	*_ppvar[4]._pval
#define _ion_dikdv	*_ppvar[5]._pval
#define _ion_cai	*_ppvar[6]._pval
#define _ion_cao	*_ppvar[7]._pval
#define _ion_ica	*_ppvar[8]._pval
#define _ion_dicadv	*_ppvar[9]._pval
 
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
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static double _hoc_efun();
 static double _hoc_ghk();
 static double _hoc_mgblock();
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
 _p = _prop->param; _ppvar = _prop->dparam;
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
 "efun", _hoc_efun,
 "ghk", _hoc_ghk,
 "mgblock", _hoc_mgblock,
 0, 0
};
#define _f_mgblock _f_mgblock_ghknmda
#define efun efun_ghknmda
#define ghk ghk_ghknmda
#define mgblock mgblock_ghknmda
 extern double _f_mgblock( double );
 extern double efun( double );
 extern double ghk( double , double , double , double );
 extern double mgblock( double );
 /* declare global and static user variables */
#define ko ko_ghknmda
 double ko = 5;
#define ki ki_ghknmda
 double ki = 140;
#define mgb mgb_ghknmda
 double mgb = 0;
#define nao nao_ghknmda
 double nao = 140;
#define nai nai_ghknmda
 double nai = 18;
#define usetable usetable_ghknmda
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "taud", 1e-009, 1e+009,
 "taur", 1e-009, 1e+009,
 "usetable_ghknmda", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "nai_ghknmda", "mM",
 "nao_ghknmda", "mM",
 "ki_ghknmda", "mM",
 "ko_ghknmda", "mM",
 "mgb_ghknmda", "1",
 "taur", "ms",
 "taud", "ms",
 "mg", "mM",
 "Pmax", "cm/s",
 "A", "cm/s",
 "B", "cm/s",
 "P", "cm/s",
 "inmda", "nA",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double delta_t = 0.01;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "nai_ghknmda", &nai_ghknmda,
 "nao_ghknmda", &nao_ghknmda,
 "ki_ghknmda", &ki_ghknmda,
 "ko_ghknmda", &ko_ghknmda,
 "mgb_ghknmda", &mgb_ghknmda,
 "usetable_ghknmda", &usetable_ghknmda,
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
 
#define _cvode_ieq _ppvar[10]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"ghknmda",
 "taur",
 "taud",
 "mg",
 "Pmax",
 0,
 "P",
 "inmda",
 0,
 "A",
 "B",
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
 	_p = nrn_prop_data_alloc(_mechtype, 19, _prop);
 	/*initialize range parameters*/
 	taur = 5;
 	taud = 50;
 	mg = 2;
 	Pmax = 1e-006;
  }
 	_prop->param = _p;
 	_prop->param_size = 19;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 11, _prop);
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
 	_ppvar[7]._pval = &prop_ion->param[2]; /* cao */
 	_ppvar[8]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[9]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
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

 void _ghknmda_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("k", -10000.);
 	ion_reg("ca", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	_k_sym = hoc_lookup("k_ion");
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 19, 11);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 7, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 8, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 9, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 10, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 ghknmda D:/Documents/CAMP 2022/tutorials/Plasticity/ghknmda.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
 static double R = 8.3145;
 static double *_t_mgblock;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static double _n_mgblock(double);
 static int _slist1[2], _dlist1[2];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   DA = - A / taur ;
   DB = - B / taud ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 DA = DA  / (1. - dt*( ( - 1.0 ) / taur )) ;
 DB = DB  / (1. - dt*( ( - 1.0 ) / taud )) ;
  return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    A = A + (1. - exp(dt*(( - 1.0 ) / taur)))*(- ( 0.0 ) / ( ( - 1.0 ) / taur ) - A) ;
    B = B + (1. - exp(dt*(( - 1.0 ) / taud)))*(- ( 0.0 ) / ( ( - 1.0 ) / taud ) - B) ;
   }
  return 0;
}
 
double ghk (  double _lv , double _lci , double _lco , double _lz ) {
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
    _hoc_setdata(_vptr);
 _r =  ghk (  *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) );
 return(_r);
}
 
double efun (  double _lz ) {
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
    _hoc_setdata(_vptr);
 _r =  efun (  *getarg(1) );
 return(_r);
}
 static double _mfac_mgblock, _tmin_mgblock;
 static void _check_mgblock();
 static void _check_mgblock() {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  static double _sav_mg;
  if (!usetable) {return;}
  if (_sav_mg != mg) { _maktable = 1;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_mgblock =  - 140.0 ;
   _tmax =  80.0 ;
   _dx = (_tmax - _tmin_mgblock)/1000.; _mfac_mgblock = 1./_dx;
   for (_i=0, _x=_tmin_mgblock; _i < 1001; _x += _dx, _i++) {
    _t_mgblock[_i] = _f_mgblock(_x);
   }
   _sav_mg = mg;
  }
 }

 double mgblock(double _lv){ _check_mgblock();
 return _n_mgblock(_lv);
 }

 static double _n_mgblock(double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 return _f_mgblock(_lv); 
}
 _xi = _mfac_mgblock * (_lv - _tmin_mgblock);
 if (isnan(_xi)) {
  return _xi; }
 if (_xi <= 0.) {
 return _t_mgblock[0];
 }
 if (_xi >= 1000.) {
 return _t_mgblock[1000];
 }
 _i = (int) _xi;
 return _t_mgblock[_i] + (_xi - (double)_i)*(_t_mgblock[_i+1] - _t_mgblock[_i]);
 }

 
double _f_mgblock (  double _lv ) {
   double _lmgblock;
 _lmgblock = 1.0 / ( 1.0 + exp ( 0.062 * - _lv ) * ( mg / 3.57 ) ) ;
   
return _lmgblock;
 }
 
static double _hoc_mgblock(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
  _r =  mgblock (  *getarg(1) );
 return(_r);
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
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
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
  cao = _ion_cao;
     _ode_spec1 ();
    }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
  cao = _ion_cao;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 3, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 4, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 5, 4);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 6, 1);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 7, 2);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 8, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 9, 4);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  A = A0;
  B = B0;
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
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  cao = _ion_cao;
 initmodel();
   }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   P = B - A ;
   mgb = mgblock ( _threadargscomma_ v ) ;
   ina = P * mgb * ghk ( _threadargscomma_ v , nai , nao , 1.0 ) * Area ;
   ica = P * 10.6 * mgb * ghk ( _threadargscomma_ v , cai , cao , 2.0 ) * Area ;
   ik = P * mgb * ghk ( _threadargscomma_ v , ki , ko , 1.0 ) * Area ;
   inmda = ica + ik + ina ;
   }
 _current += ina;
 _current += ik;
 _current += ica;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  cao = _ion_cao;
 _g = _nrn_current(_v + .001);
 	{ double _dica;
 double _dik;
 double _dina;
  _dina = ina;
  _dik = ik;
  _dica = ica;
 _rhs = _nrn_current(_v);
  _ion_dinadv += (_dina - ina)/.001 * 1.e2/ (_nd_area);
  _ion_dikdv += (_dik - ik)/.001 * 1.e2/ (_nd_area);
  _ion_dicadv += (_dica - ica)/.001 * 1.e2/ (_nd_area);
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina * 1.e2/ (_nd_area);
  _ion_ik += ik * 1.e2/ (_nd_area);
  _ion_ica += ica * 1.e2/ (_nd_area);
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
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
  cao = _ion_cao;
 { error =  state();
 if(error){fprintf(stderr,"at line 94 in file ghknmda.mod:\n	SOLVE state METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 }   }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A) - _p;  _dlist1[0] = &(DA) - _p;
 _slist1[1] = &(B) - _p;  _dlist1[1] = &(DB) - _p;
   _t_mgblock = makevector(1001*sizeof(double));
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "ghknmda.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "Two state kinetic scheme synapse described by rise time taur,\n"
  "and decay time constant taud. The normalized peak condunductance is 1.\n"
  "Decay time MUST be greater than rise time.\n"
  "\n"
  "The solution of A->G->bath with rate constants 1/taur and 1/taud is\n"
  " A = a*exp(-t/taur) and\n"
  " G = a*taud/(taud-taur)*(-exp(-t/taur) + exp(-t/taud))\n"
  "	where taur < taud\n"
  "\n"
  "If taud-taur -> 0 then we have a alphasynapse.\n"
  "and if taur -> 0 then we have just single exponential decay.\n"
  "\n"
  "The factor is evaluated in the\n"
  "initial block such that an event of weight 1 generates a\n"
  "peak conductance of 1.\n"
  "\n"
  "Because the solution is a sum of exponentials, the\n"
  "coupled equations can be solved as a pair of independent equations\n"
  "by the more efficient cnexp method.\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "	POINT_PROCESS ghknmda\n"
  "	USEION na WRITE ina\n"
  "	USEION k WRITE ik\n"
  "	USEION ca READ cai, cao WRITE ica\n"
  "	\n"
  "	RANGE taur, taud\n"
  "	RANGE inmda\n"
  "\n"
  "	RANGE P, mg, Pmax\n"
  "	GLOBAL  mgb\n"
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
  "	taur=5 (ms) <1e-9,1e9>\n"
  "	taud = 50 (ms) <1e-9,1e9>\n"
  "	cai = 100e-6(mM)	: 100nM\n"
  "	cao = 2		(mM)\n"
  "	nai = 18	(mM)	: Set for a reversal pot of +55mV\n"
  "	nao = 140	(mM)\n"
  "	ki = 140	(mM)	: Set for a reversal pot of -90mV\n"
  "	ko = 5		(mM)\n"
  "	celsius		(degC)\n"
  "	mg = 2		(mM)\n"
  "	Pmax=1e-6   (cm/s)	: According to Canavier, PNMDA's default value is\n"
  "						: 1e-6 for 10uM, 1.4e-6 cm/s for 30uM of NMDA\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	ina     (nA)\n"
  "	ik      (nA)\n"
  "	ica     (nA)\n"
  "	v (mV)\n"
  "	P (cm/s)\n"
  "	factor\n"
  "	mgb	(1)\n"
  "	inmda	(nA)\n"
  "\n"
  "	Area (cm2)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	A (cm/s)\n"
  "	B (cm/s)\n"
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
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE state METHOD cnexp\n"
  "	P=B-A\n"
  "	mgb = mgblock(v)\n"
  "\n"
  ": Area is just for unit conversion of ghk output\n"
  "\n"
  "	ina = P*mgb*ghk(v, nai, nao,1)*Area	\n"
  "	ica = P*10.6*mgb*ghk(v, cai, cao,2)*Area\n"
  "	ik = P*mgb*ghk(v, ki, ko,1)*Area\n"
  "	inmda = ica + ik + ina\n"
  "}\n"
  "\n"
  "DERIVATIVE state {\n"
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
  "FUNCTION mgblock(v(mV)) (1){\n"
  "	TABLE \n"
  "	DEPEND mg\n"
  "	FROM -140 TO 80 WITH 1000 \n"
  "\n"
  "	: from Jahr & Stevens, JNS, 1990\n"
  "\n"
  "	mgblock = 1 / (1 + exp(0.062 (/mV) * -v) * (mg / 3.57 (mM)))\n"
  "}\n"
  "\n"
  "NET_RECEIVE(weight (uS)) { 	: No use to weight, can be used instead of Pmax,\n"
  "							: if you want NetCon access to the synaptic\n"
  "							: conductance.\n"
  "	state_discontinuity(A, A + Pmax*factor)\n"
  "	state_discontinuity(B, B + Pmax*factor)\n"
  "}\n"
  ;
#endif
