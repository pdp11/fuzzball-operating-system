! Generate serial i/o modules
MACRO INP:TIO+DCNLIB.SML/LIBRARY
MACRO INP:TTY+DCNLIB.SML/LIBRARY
MACRO INP:TTZ+DCNLIB.SML/LIBRARY
! Generate direct-access i/o modules
MACRO INP:DSK+DCNLIB.SML/LIBRARY
MACRO INP:DSKDU+DCNLIB.SML/LIBRARY
MACRO INP:DSKFD+DCNLIB.SML/LIBRARY
MACRO INP:DSKDY+DCNLIB.SML/LIBRARY
MACRO INP:DSKDL+DCNLIB.SML/LIBRARY
MACRO INP:DSKVM+DCNLIB.SML/LIBRARY
! Generate emulator modules
MACRO INP:USE+DCNLIB.SML/LIBRARY
MACRO INP:RMN+DCNLIB.SML/LIBRARY+MOSLIB.SML/LIBRARY
MACRO INP:USR+DCNLIB.SML/LIBRARY
MACRO INP:CSI+DCNLIB.SML/LIBRARY
MACRO INP:SAV+DCNLIB.SML/LIBRARY
! Generate resident library
LIBRARY/CREATE/PROMPT DSRLIB
TIO,TTY,TTZ,DSK
DSKDU,DSKFD,DSKDY,DSKDL,DSKVM
USE,RMN,USR,CSI,SAV//
DELETE/NOQUERY (TIO,TTY,TTZ,DSK).OBJ
DELETE/NOQUERY (DSKDU,DSKFD,DSKDY,DSKDL,DSKVM).OBJ
DELETE/NOQUERY (USE,RMN,USR,CSI,SAV).OBJ
                                                                                                                                                               