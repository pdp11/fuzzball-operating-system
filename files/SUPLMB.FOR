	SUBROUTINE MAPLMB
C
C THIS SUBROUTINE PLOTS THE LIMB LINES
C
	COMMON /SUPMP1/ DTR,ILF,PHIOC,SINR,U2,CONE,EPS,ILTS,
	1 PI,TOVPI,V,CON1,ICROSS,IOUT,POLONG,U,VEPS,CON2,IPROJ,
	2 RLAT,UEPS,VMAX,COSO,IFST,JGR,RLON,UMAX,VMIN,COSR,IGO,OV90,
	3 RTD,UMIN,VOLD,DI,IGOLD,PHIA,SGN,UOLD,V1,DS,IGRID,PHIO,
	4 SINO,U1,V2,DSRDI,IIER
	COMMON /SUPMP2/ ICF,SPLAT(2)
	DATA SINLMB,COSLMB/.017452406,.99984765/
	DATA FLOORC /10000./
C
	FLOOR(X) = AINT(X+FLOORC)-FLOORC
	CLING(X) = FLOOR(X)+1.
C
	GOTO (500,330,305,335,500,340,500,500,500,345,500,500,345)
	1 ,IPROJ
C
C LAMBERT CONFORMAL CONIC (3)
C
305	DLAT = 1.
	RLON = PHIO+CON2
	OLAT = AMAX1(-90.,SPLAT(2)-DLAT)
	K = CLING(SPLAT(1)-SPLAT(2))
	DO 320 I = 1,2
	IGO = 0
	IFST = 1
	RLAT = OLAT
	CALL QVEC
	DO 310 J = 1,K
	RLAT = RLAT+DLAT
	CALL QVEC
310	CONTINUE
	RLON = PHIO-CON2
320	CONTINUE
	GOTO 500
C
C ORTHOGRAPHIC (2)
C
330	RADIUS = 1.
	AXIS = 1.
	GOTO 350
C
C LAMBERT EQUAL AREA (4)
C
335	RADIUS = 2.
	AXIS = 1.
	GOTO 350
C
C AZIMUTHAL EQUDISTANT (6)
C
340	RADIUS = PI
	AXIS = 1.
	GOTO 350
C
C MOLLWEIDE (10,13)
C
345	RADIUS = 2.
	AXIS = .5
C
350	U = RADIUS
	V = 0.
	W = 0.
	ILMB = 0
	IGO = 0
	IFST = 1
	DO 370 I = 1,361
	V = AXIS*V
	IF (U .LE. UMAX .AND. U .GE. UMIN .AND. V .LE. VMAX .AND.
	1 V .GE. VMIN) GOTO 355
	ILMB = 1
	IGO = 0
	GOTO 365
C
355	IF (IGO .NE. 0) GOTO 360
	CALL FRSTPT(U,V)
	IGO = 1
	GOTO 365
C
360	CALL VECTOR(U,V)
365	V = U*SINLMB+W*COSLMB
	U = U*COSLMB-W*SINLMB
	W = V
370	CONTINUE
	IF (ILMB .EQ. 0) GOTO 400
C
C DRAW BORDER
C
500	CALL PERIM(1.,1.,1.,1.)
400	RETURN
C
	END
	SUBROUTINE PERIM(MAJRX,MINRX,MAJRY,MINRY)
	COMMON /PLOT/ XA,XB,YA,YB,XC,XD,YC,YD
	I1=XA
	I2=XB
	J1=YA	
	J2=YB
	CALL RECT(I1,J1,I2-I1,J2-J1)
	RETURN
	END
                                                                                                                                                                                                                                                                   