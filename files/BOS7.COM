! dcn1
! lsi-11/23 (256k), kwv11, rx02, rl02, dlv11, interlan ether
MACRO DAT7+CFGLIB.SML/LIBRARY
MACRO SUP7+SUP+DCNLIB.SML/LIBRARY
MACRO/OBJECT:DSKDY7 SUP7+DSKDY+DCNLIB.SML/LIBRARY
MACRO/OBJECT:DSKDL7 SUP7+DSKDL+DCNLIB.SML/LIBRARY
MACRO/OBJECT:INP7 SUP7+INP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
MACRO UDEL2+CFGLIB.SML/LIBRARY
MACRO/OBJECT:SBNET7 SUP7+SBNETH+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
LINK/EXECUTE:BOS7/BOUNDARY:100/PROMPT/MAP:BOS7/GLOBAL DAT7,SUP7,INP7
SBNET7,UDEL2,DSKDY7,DSKDL7,DSRLIB,TCPLIB//
$USEI
DELETE/NOQUERY (DAT7,SUP7,DSKDY7,DSKDL7,UDEL2,INP7).OBJ
DELETE/NOQUERY SBNET7.OBJ
                                                                                                                                                                                                                                                                                                                                                                                                                            