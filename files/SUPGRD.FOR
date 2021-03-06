	SUBROUTINE MAPGRD
C
C THIS SUBROUTINE PLOTS THE MERIDIANS AND PARALLELS
C
	COMMON /SUPMP1/ DTR,ILF,PHIOC,SINR,U2,CONE,EPS,ILTS,
	1 PI,TOVPI,V,CON1,ICROSS,IOUT,POLONG,U,VEPS,CON2,IPROJ,
	2 RLAT,UEPS,VMAX,COSO,IFST,JGR,RLON,UMAX,VMIN,COSR,IGO,OV90,
	3 RTD,UMIN,VOLD,DI,IGOLD,PHIA,SGN,UOLD,V1,DS,IGRID,PHIO,
	4 SINO,U1,V2,DSRDI,IIER
	COMMON /SUPMP2/ ICF,SPLAT(2)
	DATA FLOORC /10000./
C
	FLOOR(X) = AINT(X+FLOORC)-FLOORC
	CLING(X) = FLOOR(X)+1.
C
	IF (IGRID .EQ. 0) GOTO 300
	RGRID = IGRID
C
C CHECK POLES
C
	IPF = 0
	INPF = 0
	ISPF = 0
	RLAT = 90.
	RLON = 0.
	CALL QCON
	IF (U .GT. UMAX .OR. U .LT. UMIN .OR. V .GT. VMAX .OR.
	1 V .LT. VMIN) GOTO 120
	INPF = 1
	IPF = 1
120	RLAT = -90.
	CALL QCON
	IF (U .GT. UMAX .OR. U .LT. UMIN .OR. V .GT. VMAX .OR.
	1 V .LT. VMIN) GOTO 130
	ISPF = 1
	IPF = 1
C
C SHOULD WE BOTHER LIMITING GRID POINTS TRANSFORMED
C
130	IF (ICF .NE. 0) GOTO 270
	IF (IPROJ .GE. 8 .AND. IPROJ .LE. 10) GOTO 270
C
C SET UP TO FIND EXTREMA
C
	DLON = RGRID
	STLON = FLOOR(POLONG/RGRID)*RGRID
	IF (ISPF .NE. 0 .AND. INPF .EQ. 0) STLON = STLON+180.
	RLON = STLON-DLON
	SPLON = STLON+360.
	J = 0
	PSIGN = 1.
C
C CHECK FOR SOUTH POLE
C
	IF (ISPF .NE. 0) PSIGN = -1.
C
C DO WE GRID POLES SPECIALLY
C
	SPLAT(2) = 90.*PSIGN
	SPLAT(1) = SPLAT(2)
C
C IF BOTH POLES WITHIN FRAME JUMP
C
	IF (INPF .NE. 0 .AND. ISPF .NE. 0) GOTO 270
C
C IF EITHER IN FRAME USE AS BASE
C
	IF (INPF .NE. 0 .OR. ISPF .NE. 0) GOTO 230
C
C NO POLE IS CLOSE TO THE WINDOW
C
	J = -1
	SPLAT(2) = FLOOR(PHIA/RGRID)*RGRID
	IF (ABS(SPLAT(2)) .EQ. 90.) SPLAT(2) = 0.
C
C SEARCH FOR FIRST POINT WITHIN FRAME
C
210	RLON = RLON+DLON
	DLAT = RGRID
	RLAT = SPLAT(2)-DLAT
215	RLAT = RLAT+DLAT
	CALL QCON
	IF (U .LE. UMAX .AND. U .GE. UMIN .AND. V .LE. VMAX .AND.
	1 V .GE. VMIN) GOTO 225
	IF (ABS(RLAT) .LT. 90.) GOTO 215
	IF (DLAT .LT. 0.) GOTO 220
C
C REVERSE LATITUDE SEARCH DIRECTION
C
	RLAT = SPLAT(2)+DLAT
	DLAT = -DLAT
	GOTO 215
C
C UPDATE LONGITUDE OR QUIT
C
220	J = 0
	IF (RLON-SPLON) 210,300,300
C
C SET UP FOR LIMIT SEARCH
C
225	J = J+1
	STLON = RLON
	RLON = STLON-DLON
	IF (RLAT .EQ. 0.) RLAT = SIGN(RLAT,-PSIGN)
	SPLAT(2) = RLAT
	SPLAT(1) = SPLAT(2)
C
C LONGITUDE LOOP
C
C IGF FLAG TO SIGNAL NO POINTS WITHIN WINDOW.
C IPF FLAG SIGNALS WHETHER A POLE LIES WITHIN THE FRAME.
C ILF FLAG SIGNALS WHETHER TO PLOT COMPLETE LONGITUDES
C (I.E. TO POLE FOR ALL LATITUDES.)
C
230	RLON = RLON+DLON
	IF (RLON .GE. SPLON .OR. RLON .LT. STLON) GOTO 285
	I1 = IPF
	I2 = MOD(I1+1,2)
	TSA = PSIGN
	DLAT = -PSIGN
	DX = AMOD(90.,RGRID)
	IF (DX .EQ. 0.) DX = RGRID
	XLAT = 90.-DX
	IF (ILF .NE. 0 .OR. AMOD(RLON,90.) .EQ. 0.) XLAT = 90.
	OLAT = SIGN(AMIN1(ABS(SPLAT(I2+1)),XLAT),SPLAT(I2+1))
	IGF = 0
235	IFST = 1
	IGO = 0
	RLAT = OLAT
	CALL QVEC
C
C LATITUDE LOOP
C
240	RLAT = RLAT+DLAT
	IGF = MAX0(IGO,IGF)
	CALL QVEC
	IF (IGO .NE. 0) GOTO 245
C
C THIS POINT OUTSIDE THE FRAME
C
	IF (RLAT*TSA .LE. SPLAT(I1+1)*TSA) GOTO 250
245	IF (ABS(RLAT) .LT. XLAT) GOTO 240
	RLAT = SIGN(AMAX1(ABS(SPLAT(I1+1)),XLAT),SPLAT(I1+1))
C
C POSSIBLE NEW LATITUDE EXTREMA
C
250	SPLAT(I1+1) = RLAT
C
C REVERSE LATITUDE SEARCH DIRECTION
C
	I1 = I2
	I2 = MOD(I1+1,2)
	TSA = -PSIGN
	DLAT = PSIGN
	IF (I1 .NE. 0) GOTO 235
C
C LATITUDE LOOP FINISHED
C
	IF (ABS(SPLAT(I2+1)) .LT. 90.) GOTO 255
	IPF = 1
	PSIGN = SIGN(1.,SPLAT(I2+1))
	SPLAT(I2+1) = SPLAT(I1+1)
	SPLAT(I1+1) = 90.*PSIGN
255	IF (IGF .NE. 0) GOTO 230
C
C LONGITUDE EXTREMA REACHED
C
	IF (J .NE. 0) GOTO 260
C
C CHANGE LONGITUDE DIRECTION
C
	J = 1
	SPLON = RLON
	RLON = STLON
	DLON = -DLON
	STLON = SPLON-360.
	GOTO 230
C
C SET UP LAST LONGITUDE EXTREMA
C
260	IF (DLON .LT. 0.) GOTO 265
	SPLON = RLON
	GOTO 285
C
265	STLON = RLON
	GOTO 285
C
C DRAW ALL MERIDIANS
C
270	DLON = RGRID
	STLON = 0.
	SPLON = 360.
	RLON = 0.
	SPLAT(2) = 90.
	SPLAT(1) = -90.
	DX = AMOD(90.,RGRID)
	IF (DX .EQ. 0.) DX = RGRID
	OLAT = 90.-DX
C
275	RLON = RLON+DLON
	IFST = 1
	IGO = 0
	XLAT = OLAT
	IF (ILF .NE. 0 .OR. AMOD(RLON,90.) .EQ. 0.) XLAT = 90.
	RLAT = XLAT
	CALL QVEC
280	RLAT = RLAT-1.
	CALL QVEC
	IF (RLAT .GT. -XLAT) GOTO 280
	IF (RLON .LT. SPLON) GOTO 275
C
C DRAW PARALLELS
C
285	DLAT = RGRID
	RLAT = AMIN1(SPLAT(2),SPLAT(1))
	OLAT = AMAX1(SPLAT(2),SPLAT(1))
	SPLAT(2) = FLOOR(RLAT/RGRID)*RGRID
	SPLAT(1) = AMIN1(CLING(OLAT/RGRID)*RGRID,90.)
	RLAT = AMAX1(DLAT-90.,SPLAT(2))-DLAT
	OLAT = AMIN1(90.-DLAT,SPLAT(1))
	DLON = 1.
	IF (ILF .NE. 0) IPF = 0
C
290	RLAT = RLAT+DLAT
	IF (IPF .NE. 0) DLON = 1./COS(DTR*RLAT)
	IFST = 1
	IGO = 0
	RLON = STLON
	CALL QVEC
C
295	RLON = RLON+DLON
	CALL QVEC
	IF (RLON .LE. SPLON) GOTO 295
	IF (RLAT .LT. OLAT) GOTO 290
300	RETURN
C
	END
                                                                                                                                                                                                                                                                                                                                       