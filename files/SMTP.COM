! Generate SMTP mail-transfer user program
MACRO INP:SMTP+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
MACRO INP:SMPSRT+DCNLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
MACRO INP:SMPNET+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY+NETLIB.SML/LIBRARY
LINK/EXECUTE:OUT:SMTP/PROMPT SMTP,NETLIB
SMPSRT,RNAME/O:1
SMPNET/O:1//
DELETE/NOQUERY (SMTP,SMPSRT,SMPNET).OBJ
                                                                                                                                                     