! Generate NCAR mapping program
FORTRAN/CODE:EIS MAP
FORTRAN/CODE:EIS SUPSUB
FORTRAN/CODE:EIS SUPSET
FORTRAN/CODE:EIS SUPLOT
FORTRAN/CODE:EIS SUPGRD
FORTRAN/CODE:EIS SUPLMB
FORTRAN/CODE:EIS SUPLAB
LINK/PROMPT/MAP:MAP/GLOBAL MAP,SUPSUB
SY:CSP2F3,SY:CHTABL,SY:CG2
DK:FORLIB,SY:SYSLIB
SUPSET/O:1
SUPLOT/O:1
SUPGRD,SUPLAB/O:1
SUPLMB/O:1
//
                                                                                                                                                                