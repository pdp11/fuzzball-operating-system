! Create bos configuration volume
CREATE ARC:BOS.DSK/ALLOCATE:1976
MOUNT LD4: ARC:BOS
INITIALIZE/NOQUERY LD4:
ASSIGN LD4: OUT:
! UDel fuzzballs
COPY (DOMAIN,DCN1,UDEL).TXT OUT:
COPY UDEL2.MAC,UDEL6.MAC,EGP6.MAC OUT:
COPY DAT3.MAC,BOS3.COM,SUP3.MAC OUT:
COPY DAT5.MAC,BOS5.COM,SUP5.MAC,BOS5.INX OUT:
COPY DAT6.MAC,BOS6.COM,SUP6.MAC,BOS6.INX OUT:
! Other fuzzballs
COPY DAT11.MAC,BOS11.COM,SUP11.MAC,BOS11.INX OUT:
COPY DECWRL.(MAC,TXT,DAT) OUT:
COPY DAT9.MAC,BOS9.COM,SUP9.MAC,BOS9.INX OUT:
COPY ISI.(MAC,TXT,DAT) OUT:
COPY DAT12.MAC,BOS12.COM,SUP12.MAC,BOS12.INX OUT:
COPY NTARE.(MAC,TXT,DAT) OUT:
COPY DAT13.MAC,SUP13.MAC,BOS13.INX OUT:
COPY UNIK.MAC OUT:
COPY (BOS,BOSAUX,BOSDUP,BOSGEN,TABGEN).COM OUT:
                                                                                                                                                                                                                                                                                                          