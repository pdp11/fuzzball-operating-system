! Generate macro libraries
LIBRARY/MACRO DCNLIB.SML INP:DCNLIB.MAC
LIBRARY/MACRO MOSLIB.SML INP:MOSLIB.MAC
LIBRARY/MACRO CFGLIB.SML INP:CFGLIB.MAC
LIBRARY/MACRO NETLIB.SML INP:NETLIB.MAC
LIBRARY/MACRO OUT:HELP.TXT INP:(HELP1,HELP2,HELP3).MLB
                                                                                                                                                                                                                                                                        