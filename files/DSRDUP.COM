! Save resident ligrary modules
COPY (TIO,TTY,TTZ,DSK).MAC OUT:
COPY (DSKDU,DSKFD,DSKDY,DSKDL,DSKVM).MAC OUT:
COPY (USE,RMN,USR,CSI,SAV).MAC OUT:
COPY (DSRGEN,DSRDUP).COM OUT:
                                                                                                                                                                                                                                                                                                                                           