! Generate FTPSRV file-transfer server program
MACRO INP:FTPSRV+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:FTPSRV FTPSRV,NETLIB
DELETE/NOQUERY FTPSRV.OBJ
                                                                                                                                                                                                                                                                                                                                   