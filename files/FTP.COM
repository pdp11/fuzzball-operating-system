! Generate FTP file-transfer user program
MACRO INP:FTP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
MACRO INP:FTPNET+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:FTP/PROMPT FTP,NETLIB
RNAME/O:1
FTPNET/O:1//
DELETE/NOQUERY (FTP,FTPNET).OBJ
                                                                                                                                                                                                                                