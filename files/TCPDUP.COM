! Save IP/TCP modules
COPY (INP,INPTCP,INPRTP,INPNET,INPUSE).MAC OUT:
COPY (NBS,CHKSUM,PREMPT,RDRECT).MAC OUT:
COPY (SBNDCN,SBNARP,SBNETH,SBNPKT,SBNX25).MAC OUT:
COPY (GAT,GATNET).MAC OUT:
COPY (GATSYN,GATPKT,GATDMV,GATACC,GATX25).MAC OUT:
COPY (GATACP,GATILN,GATDEQ,GATKHV,GATSLP).MAC OUT:
COPY (TCPGEN,TCPDUP).COM OUT:
                                                                                                                                                                                       