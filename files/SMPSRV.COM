! Generate SMTP mail-transfer server program
MACRO INP:SMPSRV+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:SMPSRV SMPSRV,NETLIB
DELETE/NOQUERY SMPSRV.OBJ
                                                                                                                                                                                                                                                                                                                                     