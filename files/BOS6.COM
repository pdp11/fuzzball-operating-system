! Generate egp server
!MACRO EGP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
!MACRO EGP6+CFGLIB.SML/LIBRARY
!LINK EGP6,EGP,NETLIB
!DELETE/NOQUERY (EGP,EGP6).OBJ
! dcn6 (backroom)
! lsi-11/73 (2048k), rx02, mscp, dlv11, dmv11, etc.
MACRO DAT6+CFGLIB.SML/LIBRARY
MACRO/OBJECT:SUP6 SUP6+SUP+DCNLIB.SML/LIBRARY
MACRO/OBJECT:DSKDY6 SUP6+DSKDY+DCNLIB.SML/LIBRARY
MACRO/OBJECT:INP6 SUP6+INP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
MACRO/OBJECT:SBNET6 SUP6+SBNETH+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
MACRO UDEL6+CFGLIB.SML/LIBRARY
LINK/EXECUTE:BOS6/BOUNDARY:100/PROMPT/MAP:BOS6/GLOBAL DAT6,SUP6,DSKDY6,INP6
SBNET6,UDEL6,DSRLIB,TCPLIB//
$USEI
                                                                                                                                                                                                                                                                                                                                                                         