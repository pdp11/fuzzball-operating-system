	SUBROUTINE MAPLOT
C
C THIS SUBROUTINE PLOTS THE CONTINENTAL AND U.S. STATE OUTLINES
C IOUT = 1 WORLD OUTLINE, = 2 U.S. OUTLINES, = 3 EXTENDED U.S. LINES
C
	COMMON /SUPMP1/ DTR,ILF,PHIOC,SINR,U2,CONE,EPS,ILTS,
	1 PI,TOVPI,V,CON1,ICROSS,IOUT,POLONG,U,VEPS,CON2,IPROJ,
	2 RLAT,UEPS,VMAX,COSO,IFST,JGR,RLON,UMAX,VMIN,COSR,IGO,OV90,
	3 RTD,UMIN,VOLD,DI,IGOLD,PHIA,SGN,UOLD,V1,DS,IGRID,PHIO,
	4 SINO,U1,V2,DSRDI,IIER
	DIMENSION X(8),Y(8),PTS(200)
	REAL MAXLAT,MINLAT,MAXLON,MINLON,MIDLAT,MIDLON
C
	GOTO (21,22,23),IOUT
	GOTO 110
C
21	CALL ASSIGN(3,'WORLD.BIN')
	GOTO 30
C
22	CALL ASSIGN(3,'CONUS.BIN')
	GOTO 30
C
23	CALL ASSIGN(3,'WRLDUS.BIN')
C
C READ NEXT LINE
C
30	READ (3) NPTS,MAXLAT,MINLAT,MAXLON,MINLON,(PTS(M),M = 1,NPTS)
	NPTS = NPTS/2
	IF (NPTS .EQ. 0) GOTO 100
	IF (NPTS .LE. 16 .OR. ICF .NE. 0) GOTO 70
C
C DOES THIS LINE INTERSECT THE SCREEN
C
C 	1---2---3
C 	I       I
C 	4       5
C 	I       I
C 	6---7---8
C
	MIDLAT = (MAXLAT+MINLAT)*.5
	MIDLON = (MAXLON+MINLON)*.5
	RLAT = MAXLAT
	RLON = MAXLON
	CALL QCON
	X(3) = U
	Y(3) = V
	RLON = MIDLON
	CALL QCON
	X(2) = U
	Y(2) = V
	RLON = MINLON
	CALL QCON
	X(1) = U
	Y(1) = V
	RLAT = MIDLAT
	CALL QCON
	X(4) = U
	Y(4) = V
	RLON = MAXLON
	CALL QCON
	X(5) = U
	Y(5) = V
	RLAT = MINLAT
	CALL QCON
	X(8) = U
	Y(8) = V
	RLON = MIDLON
	CALL QCON
	X(7) = U
	Y(7) = V
	RLON = MINLON
	CALL QCON
	X(6) = U
	Y(6) = V
	XMN = AMIN1(X(1),X(2),X(3),X(4),X(5),X(6),X(7),X(8))
	XMX = AMAX1(X(1),X(2),X(3),X(4),X(5),X(6),X(7),X(8))
	YMN = AMIN1(Y(1),Y(2),Y(3),Y(4),Y(5),Y(6),Y(7),Y(8))
	YMX = AMAX1(Y(1),Y(2),Y(3),Y(4),Y(5),Y(6),Y(7),Y(8))
C
	DX = AMIN1(XMX-XMN,180.)
	DY = AMIN1(YMX-YMN,180.)
	XMX = XMX+.01*DX
	XMN = XMN-.01*DX
	YMX = YMX+.01*DY
	YMN = YMN-.01*DY
C
	IF (XMN .GT. UMAX .OR. XMX .LT. UMIN .OR. YMN .GT. VMAX .OR.
	1 YMX .LT. VMIN) GOTO 30
C
70	IFST = 1
	IGO = 0
	RLAT = PTS(1)
	RLON = PTS(2)
	CALL QVEC
	DO 75 J = 2,NPTS
	RLAT = PTS(2*J-1)
	RLON = PTS(2*J)
	CALL QVEC
75	CONTINUE
	GOTO 30
C
100	CALL CLOSE(3)
110	RETURN
C
	END
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           