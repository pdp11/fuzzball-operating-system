! Generate PING packet inter-net groper user/server program
MACRO INP:PING+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:PING PING,NETLIB
DELETE/NOQUERY PING.OBJ
                                                                                                                                                                                                                                                                                                                              