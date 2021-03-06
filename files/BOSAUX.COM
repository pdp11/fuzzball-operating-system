! Command file to generate DCN fuzzball software
!
! "Laugh it up, fuzzball."
!	Han Solo to Chewbacca, "The Empire Strikes Back"
!
! Assign ARC: to the device where the archives live.
! Mount this volume on LD4: and assign DK: to it.
!
! logical devices used
! DK:	work device
!
! logical volumes and devices used
! LD1: ARC:FUZZ1	distribution sources 1
! LD2: ARC:FUZZ2	distribution sources 2
! LD3: ARC:FUZZ3	distribution binaries
! LD4: ARC:BOS		work volume
!
MOUNT LD1: ARC:FUZZ1
MOUNT LD2: ARC:FUZZ2
MOUNT LD3: ARC:FUZZ3
! COPY/PREDELETE LD1:(INP,SBNETH,DSKDL,DSKDY).MAC DK:
! COPY/PREDELETE LD2:(SUP,EGP).MAC DK:
! COPY/PREDELETE LD3:(DCN,MOS,CFG,NET)LIB.SML DK:
! COPY/PREDELETE LD3:(DSR,TCP,NET)LIB.OBJ DK:
!
! Specific configurations
!
! @BOS2
! @BOS5
! @BOS6
! @BOS7
@BOS8
@BOS9
@BOS10
@BOS12
                                                                                                                                                                                          