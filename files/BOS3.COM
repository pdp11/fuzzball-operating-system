! dcn2
! lsi-11/73 (256k), kwv11, rx02, rl02, dlv11, interlan ether
MACRO DAT3+CFGLIB.SML/LIBRARY
MACRO SUP3+SUP+DCNLIB.SML/LIBRARY
MACRO/OBJECT:DSKDY3 SUP3+DSKDY+DCNLIB.SML/LIBRARY
MACRO/OBJECT:DSKDL3 SUP3+DSKDL+DCNLIB.SML/LIBRARY
MACRO/OBJECT:INP3 SUP3+INP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
MACRO UDEL2+CFGLIB.SML/LIBRARY
MACRO/OBJECT:SBNET3 SUP3+SBNETH+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
LINK/EXECUTE:BOS3/BOUNDARY:100/PROMPT/MAP:BOS3/GLOBAL DAT3,SUP3,INP3
SBNET3,UDEL2,DSKDY3,DSKDL3,DSRLIB,TCPLIB//
$USEI
DELETE/NOQUERY (DAT3,SUP3,DSKDY3,DSKDL3,UDEL2,INP3).OBJ
DELETE/NOQUERY SBNET3.OBJ
                                                                                                                                                                                                                                                                                                                                                                                                                            