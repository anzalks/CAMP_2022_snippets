#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _Wghkampa_reg();
extern void _cad_reg();
extern void _ghknmda_reg();
extern void _h_reg();
extern void _kadist_reg();
extern void _kdrca1_reg();
extern void _na3s_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," Wghkampa.mod");
fprintf(stderr," cad.mod");
fprintf(stderr," ghknmda.mod");
fprintf(stderr," h.mod");
fprintf(stderr," kadist.mod");
fprintf(stderr," kdrca1.mod");
fprintf(stderr," na3s.mod");
fprintf(stderr, "\n");
    }
_Wghkampa_reg();
_cad_reg();
_ghknmda_reg();
_h_reg();
_kadist_reg();
_kdrca1_reg();
_na3s_reg();
}
