! Save IP/TCP application library modules
COPY (TYPE,RDASC,RDCLK,FORMAT,FNDQUE).MAC OUT:
COPY (NETSUB,NETCOM,NETMSG).MAC OUT:
COPY (RNAME,NAMRST,ASG,USER).MAC OUT:
COPY (GTUSER,TEKVCG,IMGTEK,BITFAX).MAC OUT:
COPY (LIBGEN,LIBDUP).COM OUT:
COPY (DES,CSPLIB).OBJ OUT:
                                                                                                                                                                                                                                                