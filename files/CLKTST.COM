! Set system date and time from Internet host
MACRO INP:CLKTST+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:CLKTST CLKTST,NETLIB
DELETE/NOQUERY CLKTST.OBJ
                                                                                                                                                                                                                                                                                                                                    