! Generate egp server
MACRO INP:EGP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
MACRO INP:EGP6+CFGLIB.SML/LIBRARY
LINK/MAP:EGP6/GLOBAL EGP6,EGP,NETLIB
DELETE/NOQUERY (EGP,EGP6).OBJ
                                                                                                                                                                                                                                                                                                                         