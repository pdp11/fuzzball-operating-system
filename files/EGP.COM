! Generate egp server
MACRO INP:EGP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
MACRO INP:EGP1+CFGLIB.SML/LIBRARY
LINK EGP1,EGP,NETLIB
DELETE/NOQUERY (EGP,EGP1).OBJ
                                                                                                                                                                                                                                                                                                                                         