SET TT: QUIET
SET USR NOSWAP
!SET TT: FORM
!SET TT: WIDTH=80
!SET TT: TAB
SET TT: SCOPE
ASSIGN SY: DK:
ASSIGN SY: INP:
ASSIGN SY: OUT:
ASSIGN SY: ARC:
ASSIGN TT: LP:
SET DU0: UNIT=0 PART=0
SET DU1: UNIT=0 PART=1
SET DU2: UNIT=0 PART=2
SET DU3: UNIT=0 PART=3
SET DU4: UNIT=0 PART=4
DATE 1-JAN-86
R SETCLK
SET TT: NOQUIET
!R PGWAIT
!R BOS6
                                                                                                                                                         