! Fuzzball archives disk 1
CREATE ARC:FUZZ1.DSK/ALLOCATE:1976
MOUNT LD1: ARC:FUZZ1
INITIALIZE/NOQUERY LD1:
ASSIGN LD1: OUT:
@DSRDUP
@USEDUP
@RUNDUP
@TCPDUP
@LIBDUP
! Fuzzball archives disk 2
CREATE ARC:FUZZ2.DSK/ALLOCATE:1976
MOUNT LD2: ARC:FUZZ2
INITIALIZE/NOQUERY LD2:
ASSIGN LD2: OUT:
@LOGDUP
@NETDUP
@SRVDUP
@SUPDUP
                                                                                                                                                                             