! Save server modules
COPY (UDP,CRYPT,CRYSUM,INISRV,NAMSRV).MAC OUT:
COPY (DOMSRV,TIMSRV,TFTSRV,STASRV,NTPSRV).MAC OUT:
COPY UDP.COM OUT:
COPY EGP.MAC OUT:
COPY LOG.MAC,LOG.COM OUT:
COPY (SRVGEN,SRVDUP).COM OUT:
COPY (DES,CSPLIB).OBJ OUT:
                                                                                                                                                                                                                                                                         