! Set system date and time from Internet host
MACRO INP:NETCLK+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:NETCLK NETCLK,NETLIB
DELETE/NOQUERY NETCLK.OBJ
                                                                                                                                                                                                                                                                                                                                    