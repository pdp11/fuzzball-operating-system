C SUPMAP   FROM PORTLIB                                  08/20/84       SUP00000
      SUBROUTINE SUPMAP (JPROJ,POLAT,POLON,RROT,PL1,PL2,PL3,PL4,JJLTS,  SUP00001
     1                   JGRID,JUS,JDOT,IER)                            SUP00002
C                                                                       SUP00003
C                                                                       SUP00004
C     SUBROUTINE SUPMAP (JPROJ,POLAT,POLONG,ROT,PL1,PL2,PL3,PL4,JLTS,   SUP00005
C                        JGRID,IUSOUT,IDOT,IER)                         SUP00006
C                                                                       SUP00007
C                                                                       SUP00008
C     DIMENSION OF           PL1(2),PL2(2),PL3(2),PL4(2)                SUP00009
C     ARGUMENTS                                                         SUP00010
C                                                                       SUP00011
C     LATEST REVISION        OCTOBER, 1982                              SUP00012
C                                                                       SUP00013
C     PURPOSE                TO PLOT CONTINENTAL AND/OR U.S. STATE      SUP00014
C                            OUTLINES ACCORDING TO ONE OF NINE          SUP00015
C                            PROJECTIONS.  THE ORIGIN AND ORIENTATION OFSUP00016
C                            THE PROJECTION ARE SELECTED BY THE USER.   SUP00017
C                            POINTS ON THE EARTH DEFINED BY LATITUDE ANDSUP00018
C                            LONGITUDE ARE TRANSFORMED TO POINTS IN THE SUP00019
C                            U,V PLANE, THE PLANE OF PROJECTION.  THE U SUP00020
C                            AND V AXES ARE RESPECTIVELY PARALLEL TO THESUP00021
C                            X AND Y AXES OF THE PLOTTER.  A RECTANGULARSUP00022
C                            FRAME PARALLEL TO THE U AND V AXES IS      SUP00023
C                            CHOSEN AND ONLY MATERIAL WITHIN THE FRAME  SUP00024
C                            IS PLOTTED.                                SUP00025
C                                                                       SUP00026
C     USAGE                                                             SUP00027
C                                 CALL SUPMAP (JPROJ,POLAT,POLONG,ROT,  SUP00028
C                                              PL1,PL2,PL3,PL4,JLTS,    SUP00029
C                                              JGRID,IUSOUT,IDOT,IER)   SUP00030
C                                                                       SUP00031
C     ON INPUT               JPROJ                                      SUP00032
C     FOR SUPMAP               IABS(JPROJ) DEFINES THE PROJECTION TYPE  SUP00033
C                              ACCORDING TO THE FOLLOWING CODE:         SUP00034
C                                    1  STEREOGRAPHIC                   SUP00035
C                                    2  ORTHOGRAPHIC                    SUP00036
C                                    3  LAMBERT CONFORMAL CONIC WITH TWOSUP00037
C                                       STANDARD PARALLELS              SUP00038
C                                    4  LAMBERT EQUAL AREA              SUP00039
C                                    5  GNOMONIC                        SUP00040
C                                    6  AZIMUTHAL EQUIDISTANT           SUP00041
C                                    7  DUMMY--THIS CODE IS NOT USED    SUP00042
C                                    8  CYLINDRICAL EQUIDISTANT         SUP00043
C                                    9  MERCATOR                        SUP00044
C                                   10  MOLLWEIDE TYPE                  SUP00045
C                              IF JPROJ IS NEGATIVE, THE CONTINENTAL    SUP00046
C                              OUTLINES ARE OMITTED.                    SUP00047
C                                                                       SUP00048
C                            POLAT,POLONG,ROT                           SUP00049
C                              IF (IABS(JPROJ).NE.3)                    SUP00050
C                              . POLAT AND POLONG DEFINE IN DEGREES THE SUP00051
C                                LATITUDE AND LONGITUDE OF THE POINT ON SUP00052
C                                THE GLOBE WHICH IS TO TRANSFORM TO THE SUP00053
C                                ORIGIN OF THE U,V PLANE.               SUP00054
C                                    -90 @ POLAT @ 90                   SUP00055
C                                   -180 @ POLONG @ 180                 SUP00056
C                                DEGREES OF LATITUDE NORTH OF THE       SUP00057
C                                EQUATOR AND DEGREES OF LONGITUDE EAST  SUP00058
C                                OF THE GREENWICH MERIDIAN ARE POSITIVE.SUP00059
C                                IF THE ORIGIN IS AT THE NORTH POLE,    SUP00060
C                                'NORTH' IS CONSIDERED TO BE IN THE     SUP00061
C                                DIRECTION OF (POLONG+180.). IF THE     SUP00062
C                                ORIGIN IS AT THE SOUTH POLE, 'NORTH' ISSUP00063
C                                IN THE DIRECTION OF POLONG.            SUP00064
C                              . ROT IS THE ANGLE BETWEEN THE V AXIS ANDSUP00065
C                                NORTH AT THE ORIGIN.  IT IS MEASURED INSUP00066
C                                DEGREES AND IS TAKEN TO BE POSITIVE IF SUP00067
C                                THE ANGULAR MOVEMENT FROM NORTH TO THE SUP00068
C                                V AXIS IS COUNTER-CLOCKWISE.  FOR THE  SUP00069
C                                CYLINDRICAL PROJECTIONS (8,9,10), THE  SUP00070
C                                AXIS OF THE PROJECTION IS PARALLEL TO  SUP00071
C                                THE V AXIS.                            SUP00072
C                              IF (IABS(JPROJ.EQ.3) (LAMBERT CONFORMAL  SUP00073
C                              CONIC WITH TWO STANDARD PARALLELS)       SUP00074
C                              . POLONG = CENTRAL MERIDIAN OF PROJECTIONSUP00075
C                                IN DEGREES.                            SUP00076
C                              . POLAT,ROT ARE THE TWO STANDARD         SUP00077
C                                PARALLELS IN DEGREES.                  SUP00078
C                                                                       SUP00079
C                            JLTS,PL1,PL2,PL3,PL4                       SUP00080
C                              IABS(JLTS) CAN TAKE THE VALUES 1 THROUGH SUP00081
C                              5 AND SPECIFIES ONE OF FIVE OPTIONS ON   SUP00082
C                              THE WAY IN WHICH THE LIMITS OF THE       SUP00083
C                              RECTANGULAR MAP ARE DEFINED BY THE       SUP00084
C                              PARAMETERS                               SUP00085
C                                 PL1,PL2,PL3,PL4.                      SUP00086
C                                                                       SUP00087
C                              IABS(JLTS) = 1                           SUP00088
C                                THE MAXIMUM USEFUL AREA PRODUCED BY THESUP00089
C                                PROJECTION IS PLOTTED.  PL1, PL2, PL3, SUP00090
C                                PL4 ARE NOT USED AND MAY BE SET TO     SUP00091
C                                ZERO.                                  SUP00092
C                                                                       SUP00093
C                              IABS(JLTS) = 2                           SUP00094
C                                IN THIS CASE (PL1,PL2) AND (PL3,PL4)   SUP00095
C                                ARE THE LATITUDES AND LONGITUDES IN    SUP00096
C                                DEGREES OF TWO POINTS WHICH ARE TO BE  SUP00097
C                                AT OPPOSITE CORNERS OF THE MAP.  CARE  SUP00098
C                                MUST BE TAKEN WHEN USING CYLINDRICAL   SUP00099
C                                PROJECTIONS AND THIS OPTION.           SUP00100
C                                                                       SUP00101
C                              IABS(JLTS) = 3                           SUP00102
C                                THE MINIMUM AND MAXIMUM VALUES OF U ANDSUP00103
C                                V ARE SPECIFIED BY PL1 THROUGH PL4.    SUP00104
C                                PL1 = UMIN, PL2 = UMAX, PL3 = VMIN,    SUP00105
C                                PL4 = VMAX.  KNOWLEDGE OF THE          SUP00106
C                                TRANSFORMATION EQUATIONS IS NECESSARY  SUP00107
C                                FOR THIS OPTION TO BE USED (SEE BELOW).SUP00108
C                                                                       SUP00109
C                              IABS(JLTS) = 4                           SUP00110
C                                HERE PL1 = AUMIN, PL2 = AUMAX,         SUP00111
C                                PL3 = AVMIN, PL4 = AVMAX, WHERE        SUP00112
C                                   AUMIN = ANGULAR DISTANCE FROM ORIGINSUP00113
C                                           TO LEFT FRAME OF MAP.       SUP00114
C                                   AUMAX = ANGULAR DISTANCE FROM ORIGINSUP00115
C                                           TO RIGHT FRAME OF MAP.      SUP00116
C                                   AVMIN = ANGULAR DISTANCE FROM ORIGINSUP00117
C                                           TO LOWER FRAME.             SUP00118
C                                   AVMAX = ANGULAR DISTANCE FROM ORIGINSUP00119
C                                           TO UPPER FRAME.             SUP00120
C                                AUMIN, AUMAX, AVMIN, AVMAX MUST BE     SUP00121
C                                POSITIVE AND THE ORIGIN MUST BE WITHIN SUP00122
C                                THE RECTANGULAR LIMITS OF THE MAP.     SUP00123
C                                THIS OPTION IS USEFUL FOR POLAR        SUP00124
C                                PROJECTIONS.  IT IS NOT APPROPRIATE FORSUP00125
C                                THE LAMBERT CONFORMAL WITH TWO STANDARDSUP00126
C                                PARALLELS.  AN ERROR MESSAGE IS PRINTEDSUP00127
C                                IF AN ATTEMPT IS MADE TO USE JLTS = 4  SUP00128
C                                WHEN JPROJ = 3, (SEE BELOW).           SUP00129
C                                                                       SUP00130
C                              IABS(JLTS) = 5                           SUP00131
C                                PL1 THROUGH PL4 ARE TWO ELEMENT ARRAYS SUP00132
C                                GIVING THE LATITUDES AND LONGITUDES OF SUP00133
C                                FOUR POINTS WHICH ARE TO BE ON THE FOURSUP00134
C                                SIDES OF THE RECTANGULAR FRAME.        SUP00135
C                                PL1(1), PL1(2) ARE RESPECTIVELY THE    SUP00136
C                                LATITUDE AND LONGITUDE OF A POINT ON   SUP00137
C                                THE LEFT FRAME.  SIMILARLY PL2 LIES ON SUP00138
C                                THE RIGHT FRAME, PL3 LIES ON THE LOWER SUP00139
C                                FRAME AND PL4 LIES ON THE UPPER FRAME. SUP00140
C                                NOTE THAT IN THE CALLING PROGRAM PL1   SUP00141
C                                THROUGH PL4 WILL BE DIMENSIONED:       SUP00142
C                                                                       SUP00143
C                                  DIMENSION PL1(2),PL2(2),PL3(2),PL4(2)SUP00144
C                                                                       SUP00145
C                               .IF JLTS IS POSITIVE, THE SUPMAP CALL ISSUP00146
C                                WRITTEN BELOW THE MAP.  THIS IS OMITTEDSUP00147
C                                IF JLTS IS NEGATIVE.                   SUP00148
C                                                                       SUP00149
C                            JGRID                                      SUP00150
C                              IABS (JGRID) GIVES IN DEGREES THE        SUP00151
C                              INTERVAL AT WHICH LINES OF LATITUDE AND  SUP00152
C                              LONGITUDE ARE TO BE PLOTTED.  A VALUE IN SUP00153
C                              THE RANGE 1 THROUGH 10 WILL USUALLY BE   SUP00154
C                              APPROPRIATE BUT HIGHER VALUES ARE        SUP00155
C                              ACCEPTABLE.  IF JGRID                    SUP00156
C                              <  0  THE BORDER AROUND THE MAP IS       SUP00157
C                                    OMITTED.                           SUP00158
C                              =  0  NO GRID LINES ARE PLOTTED.         SUP00159
C                              = -0  BOTH GRID AND BORDER ARE OMITTED.  SUP00160
C                                                                       SUP00161
C                            IUSOUT                                     SUP00162
C                              IABS(IUSOUT)                             SUP00163
C                              =  1  U.S. STATE OUTLINES ARE PLOTTED.   SUP00164
C                              =  0  U.S. STATE OUTLINES ARE NOT        SUP00165
C                                    PLOTTED.                           SUP00166
C                                   .NOTE THAT IF U.S. STATE OUTLINES   SUP00167
C                                    ARE REQUIRED, IT WILL BE USUAL TO  SUP00168
C                                    SUPPRESS THE CONTINENTAL OUTLINES  SUP00169
C                                    BY MAKING JPROJ NEGATIVE.          SUP00170
C                                   .IF IUSOUT IS POSITIVE, THE SUPMAP  SUP00171
C                                    CALL AND VALUES OF UMIN, UMAX,     SUP00172
C                                    VMIN, VMAX ARE PRINTED AS AN AID   SUP00173
C                                    TO DEBUGGING.  THIS IS OMITTED IF  SUP00174
C                                    IUSOUT = -0 OR -1.                 SUP00175
C                                                                       SUP00176
C                            IDOT                                       SUP00177
C                              =  0  FOR CONTINUOUS OUTLINES.           SUP00178
C                              =  1  FOR DOTTED OUTLINES.               SUP00179
C                                                                       SUP00180
C     ON OUTPUT              ALL ARGUMENTS EXCEPT IER ARE UNCHANGED.    SUP00181
C     FOR SUPMAP                                                        SUP00182
C                                                                       SUP00183
C                            IER                                        SUP00184
C                              ERROR FLAG WITH THE FOLLOWING MEANINGS.  SUP00185
C                              IF IER                                   SUP00186
C                              =  0  MAP SUCCESSFULLY PLOTTED.          SUP00187
C                              = 33  ATTEMPT TO USE NON-EXISTANT        SUP00188
C                                    PROJECTION.                        SUP00189
C                              = 34  MAP LIMITS INAPPROPRIATE.          SUP00190
C                              = 35  ANGULAR LIMITS TOO GREAT.          SUP00191
C                              = 36  MAP HAS ZERO AREA.                 SUP00192
C                              = 37-40  FAILURES IN MAP DATA ACCESS.    SUP00193
C                                       IF IER                          SUP00194
C                                    = 37  EOF ON DATA FILE.            SUP00195
C                                    = 39  UNSUCCESSFUL DATA READ.      SUP00196
C                                    = 40  NO CHECK ON LAST DATA READ.  SUP00197
C                                                                       SUP00198
C                                                                       SUP00199
C     ENTRY POINTS           MAPLOT, SUPCON, SUPFST, SUPMAP, SUPTRP,    SUP00200
C                            SUPVEC, QCON, QVEC, ULIBER, VECPLT         SUP00201
C                                                                       SUP00202
C                            MAPLOT                                     SUP00203
C                              ACTUALLY DRAWS THE MAP.                  SUP00204
C                                                                       SUP00205
C                            SUPCON                                     SUP00206
C                              ONCE THE TRANSFORMATION HAS BEEN SET UP  SUP00207
C                              BY AN INITIAL CALL TO SUPMAP, THE        SUP00208
C                              SUBROUTINE SUPCON MAY BE CALLED TO       SUP00209
C                              TRANSFORM A POINT, (LATITUDE, LONGITUDE )SUP00210
C                              TO THE CORRESPONDING POINT, ( U, V ) ON  SUP00211
C                              THE PLANE. CONTOURS MAY THUS BE READILY  SUP00212
C                              DRAWN AGAINST THE MAP BACKGROUND.        SUP00213
C                              (SEE SUPFST AND SUPVEC BELOW).           SUP00214
C                                                                       SUP00215
C                                 CALL SUPCON(RLAT,RLON,U,V)            SUP00216
C                                                                       SUP00217
C                              ON INPUT:                                SUP00218
C                                RLAT,RLON ARE THE LATITUDE AND         SUP00219
C                                LONGITUDE OF A POINT TO BE TRANSFORMED SUP00220
C                                TO THE U,V PLANE.  -90. @ RLAT @ 90.   SUP00221
C                               -180. @ RLON @ 180.                     SUP00222
C                                                                       SUP00223
C                              ON OUTPUT:                               SUP00224
C                                RLAT,RLON ARE UNCHANGED.               SUP00225
C                                U,V ARE THE TRANSFORMED COORDINATES OF SUP00226
C                                THE POINT (RLAT,RLON).                 SUP00227
C                                                                       SUP00228
C                                                                       SUP00229
C                            QCON                                       SUP00230
C                              ACTUALLY PERFORMS THE ABOVE MENTIONED    SUP00231
C                              TRANSFORMATION.                          SUP00232
C                                                                       SUP00233
C                            SUPFST                                     SUP00234
C                            SUPVEC                                     SUP00235
C                              TO FACILITATE DRAWING LINES ON THE MAP   SUP00236
C                              THESE ROUTINES WHICH ACT LIKE THE        SUP00237
C                              PLOTTING ROUTINES FRSTPT AND VECTOR ARE  SUP00238
C                              INCLUDED.                                SUP00239
C                               THEY ARE SUBJECT TO THE SAME RESTRICTI  SUP00240
C                               ONS                                     SUP00241
C                              AS SUPCON ABOVE.                         SUP00242
C                                                                       SUP00243
C                                 CALL SUPFST (RLAT,RLON)               SUP00244
C                                 CALL SUPVEC (RLAT,RLON)               SUP00245
C                                                                       SUP00246
C                            QVEC                                       SUP00247
C                              THIS ROUTINE DECIDES WHAT LINES ARE TO   SUP00248
C                              DRAWN AND WHERE.                         SUP00249
C                                                                       SUP00250
C                            SUPTRP                                     SUP00251
C                              PERFORMS INTERPOLATION TO THE EDGES OF   SUP00252
C                              THE FRAME.                               SUP00253
C                                                                       SUP00254
C                            VECPLT                                     SUP00255
C                              CALLED BY QVEC TO DRAW (DOT) LINES ON    SUP00256
C                              THE PLOTTER.                             SUP00257
C                                                                       SUP00258
C                            ULIBER                                     SUP00259
C                              PRINTS OUT AN ERROR MESSAGE.             SUP00260
C                                                                       SUP00261
C     COMMON BLOCKS           NAME      LENGTH                          SUP00262
C                            SUPMP1       49                            SUP00263
C                            SUPMP2      205    ( OCCURES ONLY IN SUPMAPSUP00264
C                                                 AND MAPLOT. BLANK     SUP00265
C                                                 COMMON MAY BE USED. ) SUP00266
C                            SUPMP3      101                            SUP00267
C                                                                       SUP00268
C     I/O                    MAP PLOTTED.  OUTLINE DATA IS READ         SUP00269
C                            FROM TAPE. SUPMAP CALL PRINTED.            SUP00270
C                                                                       SUP00271
C     PRECISION              SINGLE                                     SUP00272
C                                                                       SUP00273
C     LANGUAGE               FORTRAN                                    SUP00274
C                                                                       SUP00275
C     HISTORY                REVISED JANUARY 1969, MAY 1971,            SUP00276
C                            STANDARDIZED OCTOBER, 1973.                SUP00277
C                            REVISED JULY, 1974                         SUP00278
C                            REVISED AUGUST, 1976                       SUP00279
C                            REVISED OCTOBER, 1982                      SUP00280
C                                                                       SUP00281
C     ALGORITHM              THE LATITUDES AND LONGITUDES OF SUCCESSIVE SUP00282
C                            OUTLINE POINTS ARE TRANSFORMED TO          SUP00283
C                            COORDINATES IN THE PLANE OF PROJECTION AND SUP00284
C                            JOINED BY A VECTOR.                        SUP00285
C                                                                       SUP00286
C     REFERENCES             HERSHEY, A. V., THE PLOTTING OF MAPS ON A  SUP00287
C                              CRT PRINTER.  NWL REPORT NO. 1844, 1963. SUP00288
C                            LEE, TSO-HWA, STUDENTS SUMMARY REPORTS,    SUP00289
C                              WORK-STUDY PROGRAM IN SCIENTIFIC         SUP00290
C                              COMPUTING.  NCAR 1968.                   SUP00291
C                            PARKER, R. L., 2UCSD SUPERMAP:  WORLD      SUP00292
C                              PLOTTING PACKAGE.                        SUP00293
C                            STEERS, J.A., AN INTRODUCTION TO THE STUDY SUP00294
C                              OF MAP PROJECTIONS.  UNIVERSITY OF LONDONSUP00295
C                              PRESS, 1962.                             SUP00296
C                                                                       SUP00297
C     ACCURACY               THE DEFINITION OF THE MAP PRODUCED IS      SUP00298
C                            LIMITED BY TWO FACTORS:                    SUP00299
C                            .  THE OUTLINE DATA HAS A RESOLUTION OF    SUP00300
C                               1 DEGREE.                               SUP00301
C                            .  THE RESOLUTION OF THE DD80 IS LIMITED TOSUP00302
C                               1024 UNITS IN THE X AND Y DIRECTIONS.   SUP00303
C                                                                       SUP00304
C     TIMING                 USUALLY LESS THAN ONE SECOND PER MAP       SUP00305
C                            DEPENDING UPON PROJECTION, ORIGIN, AND     SUP00306
C                            ORIENTATION.  THE CYLINDRICAL EQUIDISTANT  SUP00307
C                            PROJECTION WITH POLAT = ROT = 0.0 IS       SUP00308
C                            PARTICULARLY FAST.                         SUP00309
C                                                                       SUP00310
C     PORTABILITY            THIS IS THE MOST PORTABLE SUPMAP VERSION.  SUP00311
C                            NOTE, HOWEVER, THE NON-PORTABILITIES WITH  SUP00312
C                            INPUT PARAMETERS JGRID AND IUSOUT  (A 1'S  SUP00313
C                            COMPLEMENT COMPUTER WITH -0 REPRESENTABLE).SUP00314
C                                                                       SUP00315
C     PLOTTING ROUTINES      PWRT, FRSTPT, VECTOR, POINT, PERIM,        SUP00316
C     USED                   SET, OPTN                                  SUP00317
C                                                                       SUP00318
C     REQUIRED RESIDENT      ATAN, TAN, SIN, COS, ALOG, SQRT, ATAN2,    SUP00319
C     ROUTINES               ACOS                                       SUP00320
C                                                                       SUP00321
C                                                                       SUP00322
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP00323
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP00324
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP00325
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP00326
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP00327
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP00328
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP00329
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP00330
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP00331
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP00332
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP00333
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP00334
     2                IIER                                              SUP00335
      COMMON /SUPMP2/ NPTS       ,MAXLAT     ,MINLAT     ,MAXLON     ,  SUP00336
     1                MINLON     ,PTS(200)                              SUP00337
C                                                                       SUP00338
      DIMENSION       PL1(2)     ,PL2(2)     ,PL3(2)     ,PL4(2)        SUP00339
      DIMENSION       LABA(8)    ,LABB(7)                               SUP00340
C                                                                       SUP00341
      REAL            LAT1       ,LAT2                                  SUP00342
C                                                                       SUP00343
      EQUIVALENCE     (PLA1,AUMIN)           ,(PLA2,AUMAX)           ,  SUP00344
     1                (PLA3,AVMIN)           ,(PLA4,AVMAX)              SUP00345
      EQUIVALENCE     (PHIA,LAT1),(ROT,LAT2)                            SUP00346
C                                                                       SUP00347
      DATA   PART / 0.9/                                                SUP00348
      DATA PLTRES / 1024./                                              SUP00349
      DATA RESLIM / 10.0 /                                              SUP00350
C                                                                       SUP00351
      SQU(X) = X*X                                                      SUP00352
C                                                                       SUP00353
C                                                                       SUP00354
      ROT = RROT                                                        SUP00355
      ILTS = IABS(JJLTS)                                                SUP00356
      JLTS = JJLTS                                                      SUP00357
      LGRID = JGRID                                                     SUP00358
      IGRID = IABS(LGRID)                                               SUP00359
      JGR = ISIGN(1,LGRID)                                              SUP00360
      IUS = JUS                                                         SUP00361
      IUSGN = ISIGN(1,IUS)                                              SUP00362
      PLA1 = PL1(1)                                                     SUP00363
      PLA2 = PL2(1)                                                     SUP00364
      PLA3 = PL3(1)                                                     SUP00365
      PLA4 = PL4(1)                                                     SUP00366
      LABL = 77                                                         SUP00367
      IDOT = JDOT                                                       SUP00368
C                                                                       SUP00369
C INITIALIZATION                                                        SUP00370
C                                                                       SUP00371
      LPROJ = JPROJ                                                     SUP00372
      IPROJ = IABS(LPROJ)                                               SUP00373
      JPR = ISIGN(1,LPROJ)                                              SUP00374
      IOUT = 0                                                          SUP00375
      IF (JPR .GT. 0) IOUT = 1                                          SUP00376
      IF (IABS(IUS) .EQ. 1) IOUT = IOUT+2                               SUP00377
      PHIA = POLAT                                                      SUP00378
      POLONG = POLON                                                    SUP00379
      PHIO = POLON                                                      SUP00380
      PHIOC = 540.-PHIO                                                 SUP00381
      ICROSS = 0                                                        SUP00382
      IIER = 0                                                          SUP00383
      ILF = 0                                                           SUP00384
C                                                                       SUP00385
C COMPUTE CONSTANTS APPROPRIATE TO EACH PROJECTION                      SUP00386
C                                                                       SUP00387
      IF (IPROJ .NE. 3) GO TO 30                                        SUP00388
C                                                                       SUP00389
C LAMBERT CONFORMAL CONIC                                               SUP00390
C                                                                       SUP00391
      SGN = SIGN(1.,0.5*(LAT1+LAT2))                                    SUP00392
      CHI1 = (90.-SGN*LAT1)*DTR                                         SUP00393
      IF (LAT1 .EQ. LAT2) GO TO 20                                      SUP00394
      CHI2 = (90.-SGN*LAT2)*DTR                                         SUP00395
      CONE = ALOG(SIN(CHI1)/SIN(CHI2))/ALOG(TAN(0.5*CHI1)/TAN(0.5*CHI2))SUP00396
      GO TO 60                                                          SUP00397
C                                                                       SUP00398
   20 CONE = COS(CHI1)                                                  SUP00399
      GO TO 60                                                          SUP00400
C                                                                       SUP00401
C THE OTHERS                                                            SUP00402
C                                                                       SUP00403
   30 X1 = ROT*DTR                                                      SUP00404
      X2 = PHIA*DTR                                                     SUP00405
      SINR = SIN(X1)                                                    SUP00406
      COSR = COS(X1)                                                    SUP00407
      SINO = SIN(X2)                                                    SUP00408
      COSO = COS(X2)                                                    SUP00409
C                                                                       SUP00410
C     GO TO (60,60,60,60,60,60,67,40,40,40), IPROJ                      SUP00411
C                                                                       SUP00412
      IF (IPROJ-7) 60,67,40                                             SUP00413
C                                                                       SUP00414
C CYLINDRICAL PROJECTIONS               (8,9,10)                        SUP00415
C                                                                       SUP00416
   40 IF (PHIA .NE. 0.0) GO TO 42                                       SUP00417
      IF (ROT .EQ. 0.0) GO TO 45                                        SUP00418
      IF (ABS(ROT) .EQ. 180.) GO TO 50                                  SUP00419
   42 SINO1 = COSO*COSR                                                 SUP00420
      COSO1 = SQRT(CON1-SINO1*SINO1)                                    SUP00421
      OVC1 = 1./COSO1                                                   SUP00422
      PHIO = PHIO-ATAN2(SINR*OVC1,-COSR*SINO*OVC1)*RTD                  SUP00423
      PHIOC = 540.-PHIO                                                 SUP00424
      SINR = SINR*COSO*OVC1                                             SUP00425
      COSR = -SINO*OVC1                                                 SUP00426
      SINO = SINO1                                                      SUP00427
      COSO = COSO1                                                      SUP00428
      GO TO 60                                                          SUP00429
C                                                                       SUP00430
C USE SIMPLE TRANSFORMATIONS FOR CYLINDRICAL PROJECTIONS IF ROT = POLAT SUP00431
C = 0.0 .                                                               SUP00432
C   I.E. IPROJ = 11, 12, ! 13                                           SUP00433
C                                                                       SUP00434
   45 SINO = 1.0                                                        SUP00435
      IPROJ = IPROJ+3                                                   SUP00436
      GO TO 55                                                          SUP00437
C                                                                       SUP00438
   50 SINO = -1.0                                                       SUP00439
      PHIO = PHIO+180.                                                  SUP00440
      PHIOC = PHIOC+180.                                                SUP00441
   55 COSO = 0.0                                                        SUP00442
      SINR = 0.0                                                        SUP00443
      COSR = 1.0                                                        SUP00444
      ILF = 1                                                           SUP00445
C                                                                       SUP00446
C ILTS = 1         THE MAXIMUM USEFUL AREA IS PLOTTED.                  SUP00447
C ---------                                                             SUP00448
C                                                                       SUP00449
   60 GO TO (61 ,62 ,62 ,61 ,61 ,66 ,67 ,68 ,66 ,70 ,                   SUP00450
     1       68 ,66 ,70 ),IPROJ                                         SUP00451
C                                                                       SUP00452
C STEREOGRAPHIC                         ( 1 )                           SUP00453
C LAMBERT EQUAL AREA                    ( 4 )                           SUP00454
C GNOMONIC                              ( 5 )                           SUP00455
C                                                                       SUP00456
   61 UMIN = -2.0                                                       SUP00457
      VMIN = -2.0                                                       SUP00458
      UMAX = 2.0                                                        SUP00459
      VMAX = 2.0                                                        SUP00460
      GO TO 80                                                          SUP00461
C                                                                       SUP00462
C ORTHOGRAPHIC                          ( 2 )                           SUP00463
C LAMBERT CONFORMAL CONIC               ( 3 )                           SUP00464
C                                                                       SUP00465
   62 UMIN = -1.0                                                       SUP00466
      VMIN = -1.0                                                       SUP00467
      UMAX = 1.0                                                        SUP00468
      VMAX = 1.0                                                        SUP00469
      GO TO 80                                                          SUP00470
C                                                                       SUP00471
C AZIMUTHAL EQUIDISTANT                 ( 6 )                           SUP00472
C MERCATOR WITH ARBITRARY POLE.         ( 9 )                           SUP00473
C MERCATOR                              ( 12 )                          SUP00474
C                                                                       SUP00475
   66 UMAX = PI                                                         SUP00476
      VMAX = PI                                                         SUP00477
      UMIN = -PI                                                        SUP00478
      VMIN = -PI                                                        SUP00479
      GO TO 80                                                          SUP00480
C                                                                       SUP00481
C DUMMY  --  ERROR EXIT                 ( 7 )                           SUP00482
C                                                                       SUP00483
   67 IIER = 33                                                         SUP00484
      CALL ULIBER (IIER,                                                SUP00485
     1             46H SUPMAP-ATTEMPT TO USE NON-EXISTANT PROJECTION,46)SUP00486
      GO TO 700                                                         SUP00487
C                                                                       SUP00488
C CYLINDRICAL EQUIDISTANT.              ( 8,11 )                        SUP00489
C                                                                       SUP00490
   68 UMAX = 180.                                                       SUP00491
      UMIN = -180.                                                      SUP00492
      VMAX = 90.                                                        SUP00493
      VMIN = -90.                                                       SUP00494
      GO TO 80                                                          SUP00495
C                                                                       SUP00496
C MOLLWEIDE TYPE                        ( 10,13 )                       SUP00497
C                                                                       SUP00498
   70 UMAX = 2.0                                                        SUP00499
      UMIN = -2.0                                                       SUP00500
      VMAX = 1.0                                                        SUP00501
      VMIN = -1.0                                                       SUP00502
C                                                                       SUP00503
   80 UEPS = 0.5*(UMAX-UMIN)                                            SUP00504
      VEPS = 0.5*(VMAX-VMIN)                                            SUP00505
      IF (IPROJ .EQ. 3) UEPS = 180.                                     SUP00506
C                                                                       SUP00507
C                                                                       SUP00508
C                                                                       SUP00509
C COMPUTE THE APPROPRIATE MAP BOUNDARIES.                               SUP00510
C                                                                       SUP00511
      GO TO (600,200,300,400,500),ILTS                                  SUP00512
C                                                                       SUP00513
C ILTS = 2         POINT (PL1,PL2) IN UPPER RIGHT CORNER , (PL3,PL4) IN SUP00514
C ---------        LOWER LEFT CORNER OF PLOT.                           SUP00515
C                                                                       SUP00516
  200 RLAT = PLA1                                                       SUP00517
      RLON = PLA2                                                       SUP00518
      CALL QCON                                                         SUP00519
      U1 = U                                                            SUP00520
      V1 = V                                                            SUP00521
      RLAT = PLA3                                                       SUP00522
      RLON = PLA4                                                       SUP00523
      CALL QCON                                                         SUP00524
      UMAX = AMAX1(U1,U)                                                SUP00525
      UMIN = AMIN1(U1,U)                                                SUP00526
      VMAX = AMAX1(V1,V)                                                SUP00527
      VMIN = AMIN1(V1,V)                                                SUP00528
      GO TO 600                                                         SUP00529
C                                                                       SUP00530
C ILTS = 3         SET PLOT LIMITS DIRECTLY.                            SUP00531
C ----------                                                            SUP00532
C                                                                       SUP00533
  300 UMAX = PLA2                                                       SUP00534
      UMIN = PLA1                                                       SUP00535
      VMAX = PLA4                                                       SUP00536
      VMIN = PLA3                                                       SUP00537
      GO TO 600                                                         SUP00538
C                                                                       SUP00539
C ILTS = 4         USE ANGULAR DISTANCES TO SET PLOT LIMITS.            SUP00540
C ----------                                                            SUP00541
C                                                                       SUP00542
  400 COSUMI = COS(AUMIN*DTR)                                           SUP00543
      SINUMI = SQRT(CON1-COSUMI*COSUMI)                                 SUP00544
      COSUMA = COS(AUMAX*DTR)                                           SUP00545
      SINUMA = SQRT(CON1-COSUMA*COSUMA)                                 SUP00546
      COSVMI = COS(AVMIN*DTR)                                           SUP00547
      SINVMI = SQRT(CON1-COSVMI*COSVMI)                                 SUP00548
      COSVMA = COS(AVMAX*DTR)                                           SUP00549
      SINVMA = SQRT(CON1-COSVMA*COSVMA)                                 SUP00550
C                                                                       SUP00551
      GO TO (401,402,403,404,405,406,407,408,409,410,                   SUP00552
     1       408,409,410),IPROJ                                         SUP00553
C                                                                       SUP00554
C STEREOGRAPHIC                         ( 1 )                           SUP00555
C                                                                       SUP00556
  401 UMAX = (1.-COSUMA)/SINUMA                                         SUP00557
      UMIN = -(1.-COSUMI)/SINUMI                                        SUP00558
      VMAX = (1.-COSVMA)/SINVMA                                         SUP00559
      VMIN = -(1.-COSVMI)/SINVMI                                        SUP00560
      GO TO 600                                                         SUP00561
C                                                                       SUP00562
C ORTHOGRAPHIC                          ( 2 )                           SUP00563
C                                                                       SUP00564
  402 IF (AMAX1(AUMIN,AUMAX,AVMIN,AVMAX) .GT. 90.) GO TO 900            SUP00565
      UMAX = SINUMA                                                     SUP00566
      UMIN = -SINUMI                                                    SUP00567
      VMAX = SINVMA                                                     SUP00568
      VMIN = -SINVMI                                                    SUP00569
      GO TO 600                                                         SUP00570
C                                                                       SUP00571
C LAMBERT CONFORMAL CONIC               ( 3 )                           SUP00572
C                                                                       SUP00573
  403 IIER = 34                                                         SUP00574
      CALL ULIBER (IIER,32H SUPMAP-MAP LIMITS INAPPROPRIATE,32)         SUP00575
      GO TO 700                                                         SUP00576
C                                                                       SUP00577
C LAMBERT EQUAL AREA                    ( 4 )                           SUP00578
C                                                                       SUP00579
  404 UMAX = (1.+COSUMA)/SINUMA                                         SUP00580
      UMIN = (1.+COSUMI)/SINUMI                                         SUP00581
      VMAX = (1.+COSVMA)/SINVMA                                         SUP00582
      VMIN = (1.+COSVMI)/SINVMI                                         SUP00583
      UMAX = 2./SQRT(1.+UMAX*UMAX)                                      SUP00584
      UMIN = -2./SQRT(1.+UMIN*UMIN)                                     SUP00585
      VMAX = 2./SQRT(1.+VMAX*VMAX)                                      SUP00586
      VMIN = -2./SQRT(1.+VMIN*VMIN)                                     SUP00587
      GO TO 600                                                         SUP00588
C                                                                       SUP00589
C GNOMONIC                              ( 5 )                           SUP00590
C                                                                       SUP00591
  405 IF (AMAX1(AUMIN,AUMAX,AVMIN,AVMAX) .GE. 90.) GO TO 900            SUP00592
      UMAX = SINUMA/COSUMA                                              SUP00593
      UMIN = -SINUMI/COSUMI                                             SUP00594
      VMAX = SINVMA/COSVMA                                              SUP00595
      VMIN = -SINVMI/COSVMI                                             SUP00596
      GO TO 600                                                         SUP00597
C                                                                       SUP00598
C AZIMUTHAL EQUIDISTANT                 ( 6 )                           SUP00599
C                                                                       SUP00600
  406 UMAX = AUMAX*DTR                                                  SUP00601
      UMIN = -AUMIN*DTR                                                 SUP00602
      VMAX = AVMAX*DTR                                                  SUP00603
      VMIN = -AVMIN*DTR                                                 SUP00604
      GO TO 600                                                         SUP00605
C                                                                       SUP00606
C DUMMY  --  ERROR EXIT                 ( 7 )                           SUP00607
C                                                                       SUP00608
  407 GO TO 67                                                          SUP00609
C                                                                       SUP00610
C CYLINDRICAL EQUIDISTANT.              ( 8,11 )                        SUP00611
C                                                                       SUP00612
  408 UMAX = AUMAX                                                      SUP00613
      UMIN = -AUMIN                                                     SUP00614
      VMAX = AVMAX                                                      SUP00615
      VMIN = -AVMIN                                                     SUP00616
      GO TO 600                                                         SUP00617
C                                                                       SUP00618
C MERCATOR                              ( 9,12 )                        SUP00619
C                                                                       SUP00620
  409 IF (AMAX1(AVMIN,AVMAX) .GE. 90.) GO TO 900                        SUP00621
      UMAX = AUMAX*DTR                                                  SUP00622
      UMIN = -AUMIN*DTR                                                 SUP00623
      VMAX = ALOG((1.+SINVMA)/COSVMA)                                   SUP00624
      VMIN = -ALOG((1.+SINVMI)/COSVMI)                                  SUP00625
      GO TO 600                                                         SUP00626
C                                                                       SUP00627
C MOLLWEIDE TYPE                        ( 10,13 )                       SUP00628
C                                                                       SUP00629
  410 UMAX = AUMAX*OV90                                                 SUP00630
      UMIN = -AUMIN*OV90                                                SUP00631
      VMAX = SINVMA                                                     SUP00632
      VMIN = -SINVMI                                                    SUP00633
      GO TO 600                                                         SUP00634
C                                                                       SUP00635
C ILTS = 5         USE FOUR EDGE POINTS TO SET LIMITS.                  SUP00636
C ----------                                                            SUP00637
C                                                                       SUP00638
  500 PLB1 = PL1(2)                                                     SUP00639
      RLAT = PLA1                                                       SUP00640
      RLON = PLB1+EPS                                                   SUP00641
      CALL QCON                                                         SUP00642
      UMIN = U                                                          SUP00643
      PLB2 = PL2(2)                                                     SUP00644
      RLAT = PLA2                                                       SUP00645
      RLON = PLB2-EPS                                                   SUP00646
      CALL QCON                                                         SUP00647
      UMAX = U                                                          SUP00648
      PLB3 = PL3(2)                                                     SUP00649
      RLAT = PLA3                                                       SUP00650
      RLON = PLB3                                                       SUP00651
      CALL QCON                                                         SUP00652
      VMIN = V                                                          SUP00653
      PLB4 = PL4(2)                                                     SUP00654
      RLAT = PLA4                                                       SUP00655
      RLON = PLB4                                                       SUP00656
      CALL QCON                                                         SUP00657
      VMAX = V                                                          SUP00658
C                                                                       SUP00659
C COMPUTE MAP LIMITS FOR PLOT                                           SUP00660
C                                                                       SUP00661
  600 DU = UMAX-UMIN                                                    SUP00662
      DV = VMAX-VMIN                                                    SUP00663
C                                                                       SUP00664
C ERROR IF MAP HAS ZERO AREA                                            SUP00665
C                                                                       SUP00666
      IF (DU.EQ.0.0 .OR. DV.EQ.0.0) GO TO 905                           SUP00667
      IF (DU .GT. DV) GO TO 610                                         SUP00668
      Y1 = 0.5*(1.-PART)                                                SUP00669
      Y2 = 1.-Y1                                                        SUP00670
      X1 = 0.5*(1.-PART*DU/DV)                                          SUP00671
      X2 = 1.-X1                                                        SUP00672
      GO TO 620                                                         SUP00673
C                                                                       SUP00674
  610 X1 = 0.5*(1.-PART)                                                SUP00675
      X2 = 1.-X1                                                        SUP00676
      Y1 = 0.5*(1.-PART*DV/DU)                                          SUP00677
      Y2 = 1.-Y1                                                        SUP00678
C                                                                       SUP00679
C ERROR IF MAP HAS ESSENTIALLY ZERO AREA                                SUP00680
C                                                                       SUP00681
  620 IF (AMIN1(X2-X1,Y2-Y1)*PLTRES .LT. RESLIM) GO TO 905              SUP00682
      CALL SET (X1,X2,Y1,Y2,UMIN,UMAX,VMIN,VMAX,1)                      SUP00683
      DS = SQU(((X2-X1)*PLTRES)/DU)                                     SUP00684
      DSRDI = SQRT(DS/DI)                                               SUP00685
C                                                                       SUP00686
C DO WE WRITE ANYTHING                                                  SUP00687
C                                                                       SUP00688
      IF (JLTS.LT.0 .AND. IUSGN.LT.0) GO TO 640                         SUP00689
C                                                                       SUP00690
C     CREATE THE LABLE                                                  SUP00691
C                                                                       SUP00692
      ENCODE (LABL,7000,LABA(1)) LPROJ,PHIA,POLONG,ROT,PLA1,PLA2,PLA3,  SUP00693
     1                           PLA4,JLTS,LGRID,IUS,IDOT               SUP00694
C                                                                       SUP00695
      IF (ILTS .EQ. 5) ENCODE (61,7010,LABB(1)) PLB1,PLB2,PLB3,PLB4     SUP00696
C                                                                       SUP00697
      IF (JLTS .LT. 0) GO TO 630                                        SUP00698
C                                                                       SUP00699
C WRITE SUPMAP CALL BENEATH THE MAP                                     SUP00700
C                                                                       SUP00701
      CALL PWRT (240,17,LABA(1),LABL,0,0)                               SUP00702
      IF (ILTS .EQ. 5) CALL PWRT (240,1,LABB(1),61,0,0)                 SUP00703
C                                                                       SUP00704
  630 IF (IUSGN .LT. 0) GO TO 640                                       SUP00705
C                                                                       SUP00706
C PRINT OUT THE CALL ET. AL.                                            SUP00707
C                                                                       SUP00708
      K = (LABL+9)/10                                                   SUP00709
      WRITE (6,6000) (LABA(I),I=1,K)                                    SUP00710
      IF (ILTS .EQ. 5) WRITE (6,6000) (LABB(I),I=1,7)                   SUP00711
C                                                                       SUP00712
      WRITE (6,6010) UMIN,UMAX,VMIN,VMAX                                SUP00713
C                                                                       SUP00714
C DRAW THE MAP                                                          SUP00715
C                                                                       SUP00716
  640 IF (IOUT.NE.0 .OR. IGRID.NE.0 .OR. JGR.GE.0) CALL MAPLOT          SUP00717
      IDOT = 0                                                          SUP00718
C                                                                       SUP00719
C RETURN IER                                                            SUP00720
C                                                                       SUP00721
  700 IER = IIER                                                        SUP00722
      RETURN                                                            SUP00723
C                                                                       SUP00724
C ERROR RETURNS                                                         SUP00725
C                                                                       SUP00726
  900 IIER = 35                                                         SUP00727
      CALL ULIBER (IIER,32H SUPMAP-ANGULAR LIMITS TOO GREAT,32)         SUP00728
      GO TO 700                                                         SUP00729
C                                                                       SUP00730
  905 IIER = 36                                                         SUP00731
      CALL ULIBER (IIER,25H SUPMAP-MAP HAS ZERO AREA,25)                SUP00732
      GO TO 700                                                         SUP00733
C                                                                       SUP00734
 7000 FORMAT (8H SUPMAP(,I3,7(1H,,F6.1),4(1H,,I3),1H))                  SUP00735
 7010 FORMAT (7X,1H(,24X,4(1H,,F6.1)1H))                                SUP00736
 6000 FORMAT (/12A10)                                                   SUP00737
 6010 FORMAT (/7X,7HUMIN = ,F11.6,5X,7HUMAX = ,F11.6,5X,7HVMIN = ,      SUP00738
     1        F11.6,5X,7HVMAX = ,F11.6)                                 SUP00739
C                                                                       SUP00740
      END                                                               SUP00741
      SUBROUTINE MAPLOT                                                 SUP00742
C                                                                       SUP00743
C THIS SUBROUTINE PLOTS THE CONTINENTAL AND U.S. STATE OUTLINES,        SUP00744
C MERIDIANS, PARALLELS, LIMBS WHERE APPROPRIATE. IT LABELS KEY MERIDIANSSUP00745
C AND POLES, AND IT DRAWS A BORDER.                                     SUP00746
      COMMON /SUPMP3/ KPT, XPOINT(50), YPOINT(50)                       SUP00747
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP00748
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP00749
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP00750
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP00751
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP00752
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP00753
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP00754
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP00755
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP00756
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP00757
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP00758
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP00759
     2                IIER                                              SUP00760
      COMMON /SUPMP2/ NPTS       ,MAXLAT     ,MINLAT     ,MAXLON     ,  SUP00761
     1                MINLON     ,PTS(200)                              SUP00762
C                                                                       SUP00763
      DIMENSION       SPLAT(2)   ,X(8)       ,Y(8)                      SUP00764
C                                                                       SUP00765
      REAL            MAXLAT     ,MINLAT     ,MAXLON     ,MINLON     ,  SUP00766
     1                MIDLAT     ,MIDLON                                SUP00767
C                                                                       SUP00768
      DATA   SINLMB,     COSLMB/                                        SUP00769
     1  0.017452406, 0.99984765/                                        SUP00770
      DATA FLOORC / 10000. /                                            SUP00771
C                                                                       SUP00772
      SQU(X) = (X)*(X)                                                  SUP00773
      FLOOR(X) = AINT(X+FLOORC)-FLOORC                                  SUP00774
      CLING(X) = FLOOR(X)+1.                                            SUP00775
C                                                                       SUP00776
C         IOUT      = 1       _WORLD OUTLINE                            SUP00777
C                   = 2       _ U.S. OUT/IN LINES                       SUP00778
C                   = 3       _ EXTENDED U.S. LINES                     SUP00779
      ISOLID = IOR(1,ISHIFT(32767,1))                                   SUP00780
      KPT = 0                                                           SUP00781
      CALL OPTN (2HDP, ISOLID)                                          SUP00782
      CALL OPTN (2HIN, 2HHI)                                            SUP00783
      ICF = 0                                                           SUP00784
      IF (ILTS .NE. 1) GO TO 10                                         SUP00785
      GO TO (10 ,10 ,10 ,5  ,10 ,5  ,905,5  ,5  ,5  ,                   SUP00786
     1       5  ,5  ,5  ),IPROJ                                         SUP00787
    5 ICF = 1                                                           SUP00788
   10 IF (IOUT .EQ. 0) GO TO 100                                        SUP00789
      REWIND 3                                                          SUP00790
      IF (IOUT-2) 30,20,25                                              SUP00791
C                                                                       SUP00792
C SKIP CONTINENTAL OUTLINES                                             SUP00793
C                                                                       SUP00794
   20 DO 21 K=1,284                                                     SUP00795
         READ (3,3005) NPTS,MAXLAT,MINLAT,MAXLON,MINLON,                SUP00796
     1                 (PTS(M),M=1,NPTS)                                SUP00797
   21 CONTINUE                                                          SUP00798
      GO TO 30                                                          SUP00799
C                                                                       SUP00800
C SKIP TO EXTENDED US OUTLINES.                                         SUP00801
C                                                                       SUP00802
   25 DO 26 K=1,610                                                     SUP00803
         READ (3,3005) NPTS,MAXLAT,MINLAT,MAXLON,MINLON,                SUP00804
     1                 (PTS(M),M=1,NPTS)                                SUP00805
   26 CONTINUE                                                          SUP00806
C                                                                       SUP00807
C READ THE NEXT LINE                                                    SUP00808
C                                                                       SUP00809
   30 READ (3,3000) NPTS,MAXLAT,MINLAT,MAXLON,MINLON,(PTS(M),M=1,NPTS)  SUP00810
      NPTS = NPTS/2                                                     SUP00811
      IF (NPTS .EQ. 0) GO TO 100                                        SUP00812
      IF (NPTS .LE. 16) GO TO 70                                        SUP00813
      IF (ICF .NE. 0) GO TO 70                                          SUP00814
C                                                                       SUP00815
C DOES THIS LINE INTERSECT THE SCREEN                                   SUP00816
C                                                                       SUP00817
C         1---2---3                                                     SUP00818
C         I       I                                                     SUP00819
C         4       5                                                     SUP00820
C         I       I                                                     SUP00821
C         6---7---8                                                     SUP00822
C                                                                       SUP00823
C                                                                       SUP00824
      MIDLAT = (MAXLAT+MINLAT)*0.5                                      SUP00825
      MIDLON = (MAXLON+MINLON)*0.5                                      SUP00826
      RLAT = MAXLAT                                                     SUP00827
      RLON = MAXLON                                                     SUP00828
      CALL QCON                                                         SUP00829
      X(3) = U                                                          SUP00830
      Y(3) = V                                                          SUP00831
      RLON = MIDLON                                                     SUP00832
      CALL QCON                                                         SUP00833
      X(2) = U                                                          SUP00834
      Y(2) = V                                                          SUP00835
      RLON = MINLON                                                     SUP00836
      CALL QCON                                                         SUP00837
      X(1) = U                                                          SUP00838
      Y(1) = V                                                          SUP00839
      RLAT = MIDLAT                                                     SUP00840
      CALL QCON                                                         SUP00841
      X(4) = U                                                          SUP00842
      Y(4) = V                                                          SUP00843
      RLON = MAXLON                                                     SUP00844
      CALL QCON                                                         SUP00845
      X(5) = U                                                          SUP00846
      Y(5) = V                                                          SUP00847
      RLAT = MINLAT                                                     SUP00848
      CALL QCON                                                         SUP00849
      X(8) = U                                                          SUP00850
      Y(8) = V                                                          SUP00851
      RLON = MIDLON                                                     SUP00852
      CALL QCON                                                         SUP00853
      X(7) = U                                                          SUP00854
      Y(7) = V                                                          SUP00855
      RLON = MINLON                                                     SUP00856
      CALL QCON                                                         SUP00857
      X(6) = U                                                          SUP00858
      Y(6) = V                                                          SUP00859
      XMN = AMIN1(X(1),X(2),X(3),X(4),X(5),X(6),X(7),X(8))              SUP00860
      XMX = AMAX1(X(1),X(2),X(3),X(4),X(5),X(6),X(7),X(8))              SUP00861
      YMN = AMIN1(Y(1),Y(2),Y(3),Y(4),Y(5),Y(6),Y(7),Y(8))              SUP00862
      YMX = AMAX1(Y(1),Y(2),Y(3),Y(4),Y(5),Y(6),Y(7),Y(8))              SUP00863
C                                                                       SUP00864
      DX = AMIN1(XMX-XMN,180.)                                          SUP00865
      DY = AMIN1(YMX-YMN,180.)                                          SUP00866
      XMX = XMX+0.01*DX                                                 SUP00867
      XMN = XMN-0.01*DX                                                 SUP00868
      YMX = YMX+0.01*DY                                                 SUP00869
      YMN = YMN-0.01*DY                                                 SUP00870
C                                                                       SUP00871
      IF (XMN.GT.UMAX .OR. XMX.LT.UMIN .OR. YMN.GT.VMAX .OR.            SUP00872
     1    YMX.LT.VMIN) GO TO 30                                         SUP00873
C                                                                       SUP00874
   70 RLAT = PTS(1)                                                     SUP00875
      RLON = PTS(2)                                                     SUP00876
      IFST = 1                                                          SUP00877
      IGO = 0                                                           SUP00878
      CALL QVEC                                                         SUP00879
      DO 75 J=2,NPTS                                                    SUP00880
         RLAT = PTS(2*J-1)                                              SUP00881
         RLON = PTS(2*J)                                                SUP00882
         CALL QVEC                                                      SUP00883
   75 CONTINUE                                                          SUP00884
      GO TO 30                                                          SUP00885
C                                                                       SUP00886
100   IF (KPT.GT.0)   CALL POINTS (XPOINT, YPOINT, KPT, 0, 0)           SUP00887
      KPT = 0                                                           SUP00888
      SPLAT(2) = 90.                                                    SUP00889
      SPLAT(1) = -90.                                                   SUP00890
      IF (IGRID .EQ. 0) GO TO 300                                       SUP00891
      IDOTS = IDOT                                                      SUP00892
      IDOT = 0                                                          SUP00893
C                                                                       SUP00894
C LETTER KEY MERIDEANS AND POLES                                        SUP00895
C                                                                       SUP00896
C     NORTH POLE                                                        SUP00897
      INPF = 0                                                          SUP00898
      IPF = 0                                                           SUP00899
      RLAT = 90.                                                        SUP00900
      RLON = 0.0                                                        SUP00901
      CALL QCON                                                         SUP00902
      IF ((U .GT. UMAX) .OR. (U .LT. UMIN) .OR. (V .GT. VMAX) .OR.      SUP00903
     1    (V .LT. VMIN)) GO TO 110                                      SUP00904
      UNP = U                                                           SUP00905
      VNP = V                                                           SUP00906
      INPF = 1                                                          SUP00907
      IPF = 1                                                           SUP00908
      CALL PWRT (U,V,2HNP,2,1,0)                                        SUP00909
C                                                                       SUP00910
C     SOUTH POLE                                                        SUP00911
C                                                                       SUP00912
  110 ISPF = 0                                                          SUP00913
      RLAT = -90.                                                       SUP00914
      CALL QCON                                                         SUP00915
      IF ((U .GT. UMAX) .OR. (U .LT. UMIN) .OR. (V .GT. VMAX) .OR.      SUP00916
     1    (V .LT. VMIN)) GO TO 120                                      SUP00917
      USP = U                                                           SUP00918
      VSP = V                                                           SUP00919
      ISPF = 1                                                          SUP00920
      IPF = 1                                                           SUP00921
      CALL PWRT (U,V,2HSP,2,1,0)                                        SUP00922
C                                                                       SUP00923
C     EQUATOR                                                           SUP00924
C                                                                       SUP00925
  120 RLON = PHIO-10.                                                   SUP00926
      RLAT = 0.0                                                        SUP00927
      DO 125 I=1,36                                                     SUP00928
         RLON = RLON+10.                                                SUP00929
         CALL QCON                                                      SUP00930
         IF (U.LE.UMAX .AND. U.GE.UMIN .AND. V.LE.VMAX .AND. V.GE.VMIN) SUP00931
     1       GO TO 130                                                  SUP00932
  125 CONTINUE                                                          SUP00933
      GO TO 140                                                         SUP00934
C                                                                       SUP00935
  130 CALL PWRT (U,V,2HEQ,2,1,0)                                        SUP00936
C                                                                       SUP00937
C GREENWICH MERIDEAN                                                    SUP00938
C                                                                       SUP00939
  140 RLAT = 85.                                                        SUP00940
      RLON = 0.0                                                        SUP00941
      DO 145 I=1,16                                                     SUP00942
         RLAT = RLAT-10.                                                SUP00943
         CALL QCON                                                      SUP00944
         IF (U.LE.UMAX .AND. U.GE.UMIN .AND. V.LE.VMAX .AND. V.GE.VMIN) SUP00945
     1       GO TO 150                                                  SUP00946
  145 CONTINUE                                                          SUP00947
      GO TO 160                                                         SUP00948
C                                                                       SUP00949
  150 CALL PWRT (U,V,2HGM,2,1,0)                                        SUP00950
C                                                                       SUP00951
C DATE LINE                                                             SUP00952
C                                                                       SUP00953
  160 RLAT = 85.                                                        SUP00954
      RLON = 180.                                                       SUP00955
      DO 165 I=1,16                                                     SUP00956
         RLAT = RLAT-10.                                                SUP00957
         CALL QCON                                                      SUP00958
         IF (U.LE.UMAX .AND. U.GE.UMIN .AND. V.LE.VMAX .AND. V.GE.VMIN) SUP00959
     1       GO TO 170                                                  SUP00960
  165 CONTINUE                                                          SUP00961
      GO TO 200                                                         SUP00962
C                                                                       SUP00963
  170 CALL PWRT (U,V,1HI,1,1,0)                                         SUP00964
C                                                                       SUP00965
  200 CALL OPTN (2HDP, 7967)                                            SUP00966
      RGRID = IGRID                                                     SUP00967
      CALL OPTN (2HIN, 2HLO)                                            SUP00968
C                                                                       SUP00969
C SHOULD WE BOTHER LIMITING GRID POINTS TRANSFORMED.                    SUP00970
C                                                                       SUP00971
      IF (ICF .NE. 0) GO TO 270                                         SUP00972
      IF (IPROJ.GE.8 .AND. IPROJ.LE.10) GO TO 270                       SUP00973
C                                                                       SUP00974
C SET UP TO FIND EXTREMA                                                SUP00975
C                                                                       SUP00976
      DLON = RGRID                                                      SUP00977
      STLON = FLOOR(POLONG/RGRID)*RGRID                                 SUP00978
      IF (ISPF.NE.0 .AND. INPF.EQ.0) STLON = STLON+180.                 SUP00979
      RLON = STLON-DLON                                                 SUP00980
      SPLON = STLON+360.                                                SUP00981
      J = 0                                                             SUP00982
      PSIGN = 1.                                                        SUP00983
C                                                                       SUP00984
C CHECK FOR SOUTH POLE                                                  SUP00985
C                                                                       SUP00986
      IF (ISPF .NE. 0) PSIGN = -1.                                      SUP00987
C                                                                       SUP00988
C DO WE GRID POLES SPECIALLY.                                           SUP00989
C                                                                       SUP00990
      SPLAT(2) = 90.*PSIGN                                              SUP00991
      SPLAT(1) = SPLAT(2)                                               SUP00992
C                                                                       SUP00993
C IF BOTH POLES WITHIN FRAME JUMP.                                      SUP00994
C                                                                       SUP00995
      IF (INPF.NE.0 .AND. ISPF.NE.0) GO TO 270                          SUP00996
C                                                                       SUP00997
C IF EITHER IN FRAME  USE AS BASE                                       SUP00998
C                                                                       SUP00999
      IF (INPF.NE.0 .OR. ISPF.NE.0) GO TO 230                           SUP01000
C                                                                       SUP01001
C NO POLE IS CLOSE TO THE WINDOW                                        SUP01002
C                                                                       SUP01003
      J = -1                                                            SUP01004
      SPLAT(2) = FLOOR(PHIA/RGRID)*RGRID                                SUP01005
      IF (ABS(SPLAT(2)) .EQ. 90.) SPLAT(2) = 0.0                        SUP01006
C                                                                       SUP01007
C SEARCH FOR FIRST POINT WITHIN FRAME.                                  SUP01008
C                                                                       SUP01009
  210 RLON = RLON+DLON                                                  SUP01010
      DLAT = RGRID                                                      SUP01011
      RLAT = SPLAT(2)-DLAT                                              SUP01012
  215 RLAT = RLAT+DLAT                                                  SUP01013
      CALL QCON                                                         SUP01014
      IF ((U .LE. UMAX) .AND. (U .GE. UMIN) .AND. (V .LE. VMAX) .AND.   SUP01015
     1    (V .GE. VMIN)) GO TO 225                                      SUP01016
      IF (ABS(RLAT) .LT. 90.) GO TO 215                                 SUP01017
      IF (DLAT .LT. 0.0) GO TO 220                                      SUP01018
C                                                                       SUP01019
C     REVERSE LATITUDE SEARCH DIRECTION                                 SUP01020
C                                                                       SUP01021
      RLAT = SPLAT(2)+DLAT                                              SUP01022
      DLAT = -DLAT                                                      SUP01023
      GO TO 215                                                         SUP01024
C                                                                       SUP01025
C     UPDATE LONGITUDE ! QUIT.                                          SUP01026
C                                                                       SUP01027
  220 J = 0                                                             SUP01028
      IF (RLON-SPLON) 210,300,300                                       SUP01029
C                                                                       SUP01030
C SET UP FOR LIMIT SEARCH                                               SUP01031
C                                                                       SUP01032
  225 J = J+1                                                           SUP01033
      STLON = RLON                                                      SUP01034
      RLON = STLON-DLON                                                 SUP01035
      IF (RLAT .EQ. 0.0) RLAT = SIGN(RLAT,-PSIGN)                       SUP01036
      SPLAT(2) = RLAT                                                   SUP01037
      SPLAT(1) = SPLAT(2)                                               SUP01038
C                                                                       SUP01039
C LONGITUDE LOOP                                                        SUP01040
C                                                                       SUP01041
C     IGF       FLAG TO SIGNAL NO POINTS WITHIN WINDOW.                 SUP01042
C     IPF       FLAG SIGNALS WHETHER A POLE LIES WITHIN THE FRAME.      SUP01043
C     ILF       FLAG SIGNALS WHETHER TO PLOT COMPLETE LONGITUDES        SUP01044
C               (I.E. TO POLE FOR ALL LATITUDES.)                       SUP01045
C                                                                       SUP01046
  230 RLON = RLON+DLON                                                  SUP01047
      IF (RLON.GE.SPLON .OR. RLON.LT.STLON) GO TO 285                   SUP01048
      I1 = IPF                                                          SUP01049
      I2 = MOD(I1+1,2)                                                  SUP01050
      TSA = PSIGN                                                       SUP01051
      DLAT = -PSIGN                                                     SUP01052
      DX = AMOD(90.,RGRID)                                              SUP01053
      IF (DX .EQ. 0.0) DX = RGRID                                       SUP01054
      XLAT = 90.-DX                                                     SUP01055
      IF (ILF.NE.0 .OR. AMOD(RLON,90.).EQ.0.0) XLAT = 90.               SUP01056
      OLAT = SIGN(AMIN1(ABS(SPLAT(I2+1)),XLAT),SPLAT(I2+1))             SUP01057
      IGF = 0                                                           SUP01058
  235 IFST = 1                                                          SUP01059
      IGO = 0                                                           SUP01060
      RLAT = OLAT                                                       SUP01061
      CALL QVEC                                                         SUP01062
C                                                                       SUP01063
C LATITUDE LOOP.                                                        SUP01064
C                                                                       SUP01065
  240 RLAT = RLAT+DLAT                                                  SUP01066
      IGF = MAX0(IGO,IGF)                                               SUP01067
      CALL QVEC                                                         SUP01068
      IF (IGO .NE. 0) GO TO 245                                         SUP01069
C                                                                       SUP01070
C THIS POINT OUTSIDE THE FRAME                                          SUP01071
C                                                                       SUP01072
      IF (RLAT*TSA .LE. SPLAT(I1+1)*TSA) GO TO 250                      SUP01073
  245 IF (ABS(RLAT) .LT. XLAT) GO TO 240                                SUP01074
      RLAT = SIGN(AMAX1(ABS(SPLAT(I1+1)),XLAT),SPLAT(I1+1))             SUP01075
C                                                                       SUP01076
C POSSIBLE NEW LATITUDE EXTREMA.                                        SUP01077
C                                                                       SUP01078
  250 SPLAT(I1+1) = RLAT                                                SUP01079
C                                                                       SUP01080
C     REVERSE LATITUDE SEARCH DIRECTION                                 SUP01081
C                                                                       SUP01082
      I1 = I2                                                           SUP01083
      I2 = MOD(I1+1,2)                                                  SUP01084
      TSA = -PSIGN                                                      SUP01085
      DLAT = PSIGN                                                      SUP01086
      IF (I1 .NE. 0) GO TO 235                                          SUP01087
C                                                                       SUP01088
C LATITUDE LOOP FINISHED.                                               SUP01089
C                                                                       SUP01090
      IF (ABS(SPLAT(I2+1)) .LT. 90.) GO TO 255                          SUP01091
      IPF = 1                                                           SUP01092
      PSIGN = SIGN(1.,SPLAT(I2+1))                                      SUP01093
      SPLAT(I2+1) = SPLAT(I1+1)                                         SUP01094
      SPLAT(I1+1) = 90.*PSIGN                                           SUP01095
  255 IF (IGF .NE. 0) GO TO 230                                         SUP01096
C                                                                       SUP01097
C LONGITUDE EXTREMA REACHED.                                            SUP01098
C                                                                       SUP01099
      IF (J .NE. 0) GO TO 260                                           SUP01100
C                                                                       SUP01101
C CHANGE LONGITUDE DIRECTION.                                           SUP01102
C                                                                       SUP01103
      J = 1                                                             SUP01104
      SPLON = RLON                                                      SUP01105
      RLON = STLON                                                      SUP01106
      DLON = -DLON                                                      SUP01107
      STLON = SPLON-360.                                                SUP01108
      GO TO 230                                                         SUP01109
C                                                                       SUP01110
C SET UP LAST LONGITUDE EXTREMA.                                        SUP01111
C                                                                       SUP01112
  260 IF (DLON .LT. 0.0) GO TO 265                                      SUP01113
      SPLON = RLON                                                      SUP01114
      GO TO 285                                                         SUP01115
  265 STLON = RLON                                                      SUP01116
      GO TO 285                                                         SUP01117
C                                                                       SUP01118
C DRAW ALL MERIDEANS.                                                   SUP01119
C                                                                       SUP01120
  270 DLON = RGRID                                                      SUP01121
      STLON = 0.0                                                       SUP01122
      SPLON = 360.                                                      SUP01123
      RLON = 0.0                                                        SUP01124
      SPLAT(2) = 90.                                                    SUP01125
      SPLAT(1) = -90.                                                   SUP01126
      DX = AMOD(90.,RGRID)                                              SUP01127
      IF (DX .EQ. 0.0) DX = RGRID                                       SUP01128
      OLAT = 90.-DX                                                     SUP01129
C                                                                       SUP01130
  275 RLON = RLON+DLON                                                  SUP01131
      IGO = 0                                                           SUP01132
      IFST = 1                                                          SUP01133
      XLAT = OLAT                                                       SUP01134
      IF (ILF.NE.0 .OR. AMOD(RLON,90.).EQ.0.0) XLAT = 90.               SUP01135
      RLAT = XLAT                                                       SUP01136
      CALL QVEC                                                         SUP01137
  280 RLAT = RLAT-1.                                                    SUP01138
      CALL QVEC                                                         SUP01139
      IF (RLAT .GT. -XLAT) GO TO 280                                    SUP01140
      IF (RLON .LT. SPLON) GO TO 275                                    SUP01141
C                                                                       SUP01142
C                                                                       SUP01143
C DRAW PARALLELS                                                        SUP01144
C                                                                       SUP01145
  285 DLAT = RGRID                                                      SUP01146
      RLAT = AMIN1(SPLAT(2),SPLAT(1))                                   SUP01147
      OLAT = AMAX1(SPLAT(2),SPLAT(1))                                   SUP01148
      SPLAT(2) = FLOOR(RLAT/RGRID)*RGRID                                SUP01149
      SPLAT(1) = AMIN1(CLING(OLAT/RGRID)*RGRID,90.)                     SUP01150
      RLAT = AMAX1(DLAT-90.,SPLAT(2))-DLAT                              SUP01151
      OLAT = AMIN1(90.-DLAT,SPLAT(1))                                   SUP01152
      DLON = 1.                                                         SUP01153
      IF (ILF .NE. 0) IPF = 0                                           SUP01154
C                                                                       SUP01155
  290 RLAT = RLAT+DLAT                                                  SUP01156
      IF (IPF .NE. 0) DLON = 1./COS(DTR*RLAT)                           SUP01157
      IGO = 0                                                           SUP01158
      IFST = 1                                                          SUP01159
      RLON = STLON                                                      SUP01160
      CALL QVEC                                                         SUP01161
C                                                                       SUP01162
  295 RLON = RLON+DLON                                                  SUP01163
      CALL QVEC                                                         SUP01164
      IF (RLON .LE. SPLON) GO TO 295                                    SUP01165
      IF (RLAT .LT. OLAT) GO TO 290                                     SUP01166
C                                                                       SUP01167
      IDOT = IDOTS                                                      SUP01168
      CALL OPTN (2HDP, ISOLID)                                          SUP01169
      CALL OPTN (2HIN, 2HHI)                                            SUP01170
C                                                                       SUP01171
C DRAW LIMB LINES                                                       SUP01172
C                                                                       SUP01173
  300 IDOTS = IDOT                                                      SUP01174
      IDOT = 0                                                          SUP01175
      GO TO (400,330,305,335,400,340,400,400,400,345,                   SUP01176
     1       400,400,345),IPROJ                                         SUP01177
C                                                                       SUP01178
C     LAMBERT CONFORMAL CONIC           [3]                             SUP01179
C                                                                       SUP01180
  305 DLAT = 1.                                                         SUP01181
      RLON = PHIO+CON2                                                  SUP01182
      OLAT = AMAX1(-90.,SPLAT(2)-DLAT)                                  SUP01183
      K = CLING(SPLAT(1)-SPLAT(2))                                      SUP01184
      DO 320 I=1,2                                                      SUP01185
         IGO = 0                                                        SUP01186
         IFST = 1                                                       SUP01187
         RLAT = OLAT                                                    SUP01188
         CALL QVEC                                                      SUP01189
         DO 310 J=1,K                                                   SUP01190
            RLAT = RLAT+DLAT                                            SUP01191
            CALL QVEC                                                   SUP01192
  310    CONTINUE                                                       SUP01193
         RLON = PHIO-CON2                                               SUP01194
  320 CONTINUE                                                          SUP01195
      GO TO 400                                                         SUP01196
C                                                                       SUP01197
C     ORTHOGRAPHIC                      [2]                             SUP01198
C                                                                       SUP01199
  330 RADIUS = 1.                                                       SUP01200
      AXIS = 1.                                                         SUP01201
      GO TO 350                                                         SUP01202
C                                                                       SUP01203
C     LAMBERT EQUAL AREA                [4]                             SUP01204
C                                                                       SUP01205
  335 RADIUS = 2.                                                       SUP01206
      AXIS = 1.                                                         SUP01207
      GO TO 350                                                         SUP01208
C                                                                       SUP01209
C     AZIMUTHAL EQUDISTANT              [6]                             SUP01210
C                                                                       SUP01211
  340 RADIUS = PI                                                       SUP01212
      AXIS = 1.                                                         SUP01213
      GO TO 350                                                         SUP01214
C                                                                       SUP01215
C     MOLLWEIDE.                        [10,13]                         SUP01216
C                                                                       SUP01217
  345 RADIUS = 2.                                                       SUP01218
      AXIS = 0.5                                                        SUP01219
C                                                                       SUP01220
  350 U = RADIUS                                                        SUP01221
      V = 0.0                                                           SUP01222
      W = 0.0                                                           SUP01223
      IGO = 0                                                           SUP01224
      IFST = 1                                                          SUP01225
      DO 370 I=1,361                                                    SUP01226
         V = AXIS*V                                                     SUP01227
         IF (U.LE.UMAX .AND. U.GE.UMIN .AND. V.LE.VMAX .AND. V.GE.VMIN) SUP01228
     1       GO TO 355                                                  SUP01229
         IGO = 0                                                        SUP01230
         GO TO 365                                                      SUP01231
  355    IF (IGO .NE. 0) GO TO 360                                      SUP01232
         CALL FRSTPT (U,V)                                              SUP01233
         IGO = 1                                                        SUP01234
         GO TO 365                                                      SUP01235
C                                                                       SUP01236
  360    CALL VECTOR (U,V)                                              SUP01237
  365    V = U*SINLMB+W*COSLMB                                          SUP01238
         U = U*COSLMB-W*SINLMB                                          SUP01239
         W = V                                                          SUP01240
  370 CONTINUE                                                          SUP01241
C                                                                       SUP01242
C DRAW BORDER                                                           SUP01243
C                                                                       SUP01244
  400 IF (JGR .GE. 0) CALL PERIM (1,1,1,1)                              SUP01245
      IDOT = IDOTS                                                      SUP01246
  477 IF (KPT.GT.0)  CALL POINTS (XPOINT, YPOINT, KPT, 0, 0)            SUP01247
      KPT = 0                                                           SUP01248
      RETURN                                                            SUP01249
C                                                                       SUP01250
  905 GOTO 477                                                          SUP01251
 3005 FORMAT (I4,14X,6A9,8X/(8A9,8X))                                   SUP01252
 3000 FORMAT (I4,14X,6F9.3,8X/(8F9.3,8X))                               SUP01253
      END                                                               SUP01254
      SUBROUTINE QCON                                                   SUP01255
C                                                                       SUP01256
C THIS SUBROUTINE TRANSFORMS THE POINT (RLAT,RLON), IN DEGREES,         SUP01257
C TO (U,V) ON THE MAP PLANE DEPENDENT UPON THE PROJECTION, IPROJ.       SUP01258
C                                                                       SUP01259
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01260
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01261
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01262
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01263
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01264
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01265
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01266
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01267
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01268
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01269
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01270
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01271
     2                IIER                                              SUP01272
C                                                                       SUP01273
      DATA OLDU,OLDV / 0.,0./                                           SUP01274
C                                                                       SUP01275
C                                                                       SUP01276
      U = AMOD(RLON+PHIOC,360.)-180.                                    SUP01277
C                                                                       SUP01278
      GO TO (50 ,50 ,130,50 ,50 ,50 ,170,50 ,50 ,50 ,                   SUP01279
     1       210,220,230),IPROJ                                         SUP01280
C                                                                       SUP01281
   50 T1 = U*DTR                                                        SUP01282
      T2 = RLAT*DTR                                                     SUP01283
      SINPH = SIN(T1)                                                   SUP01284
      SINLA = SIN(T2)                                                   SUP01285
      COSPH = COS(T1)                                                   SUP01286
      COSLA = SQRT(1.-SINLA*SINLA)                                      SUP01287
      TCOS = COSLA*COSPH                                                SUP01288
      COSA = SINLA*SINO+TCOS*COSO                                       SUP01289
      SINA = SQRT(CON1-COSA*COSA)                                       SUP01290
      OVSINA = 1./SINA                                                  SUP01291
      SINB = COSLA*SINPH*OVSINA                                         SUP01292
      COSB = (SINLA*COSO-TCOS*SINO)*OVSINA                              SUP01293
C                                                                       SUP01294
C PERFORM TRANSFORMATION APPROPRIATE TO THE PROJECTION                  SUP01295
C                                                                       SUP01296
      GO TO (110,120,130,140,150,160,170,180,190,200),IPROJ             SUP01297
C                                                                       SUP01298
C STEREOGRAPHIC                         [ 1 ]                           SUP01299
C                                                                       SUP01300
  110 R = (1.-COSA)*OVSINA                                              SUP01301
      GO TO 300                                                         SUP01302
C                                                                       SUP01303
C ORTHOGRAPHIC                          [ 2 ]                           SUP01304
C                                                                       SUP01305
  120 R = SINA                                                          SUP01306
      IF (COSA) 320,320,300                                             SUP01307
C                                                                       SUP01308
C LAMBERT CONFORMAL CONIC               [3]                             SUP01309
C                                                                       SUP01310
  130 UDIF = ABS(U-OLDU)                                                SUP01311
      OLDU = U                                                          SUP01312
      CHI = 90.-SGN*RLAT                                                SUP01313
      IF (CHI .GE. CON2) GO TO 320                                      SUP01314
      R = TAN(0.5*DTR*CHI)**CONE                                        SUP01315
      U = U*CONE*DTR                                                    SUP01316
      V = -R*SGN*COS(U)                                                 SUP01317
      U = R*SIN(U)                                                      SUP01318
      GO TO 310                                                         SUP01319
C                                                                       SUP01320
C LAMBERT EQUAL AREA                    [ 4 ]                           SUP01321
C                                                                       SUP01322
  140 IF (ABS(COSA+1.) .LT. 1.E-6) GO TO 320                            SUP01323
      R = (1.+COSA)*OVSINA                                              SUP01324
      R = 2./SQRT(1.+R*R)                                               SUP01325
      GO TO 300                                                         SUP01326
C                                                                       SUP01327
C GNOMONIC                              [ 5 ]                           SUP01328
C                                                                       SUP01329
  150 IF (COSA .LE. 0.0) GO TO 320                                      SUP01330
      R = SINA/COSA                                                     SUP01331
      GO TO 300                                                         SUP01332
C                                                                       SUP01333
C AZIMUTHAL EQUIDIDSANT                 [ 6 ]                           SUP01334
C                                                                       SUP01335
  160 IF (ABS(COSA+1.) .LT. 1.E-6) GO TO 320                            SUP01336
      R = ACOS(COSA)                                                    SUP01337
      GO TO 300                                                         SUP01338
C                                                                       SUP01339
C DUMMY   --  ERROR                                                     SUP01340
C                                                                       SUP01341
  170 IIER = 33                                                         SUP01342
      CALL ULIBER (IIER,                                                SUP01343
     1             46H SUPMAP-ATTEMPT TO USE NON-EXISTANT PROJECTION,46)SUP01344
      GO TO 320                                                         SUP01345
C                                                                       SUP01346
C CYLINDRICAL EQUIDISTANT,  ARBITRARY POLE AND ORIENTATION.             SUP01347
C                                                                       SUP01348
  180 IF (ABS(1.-COSA*COSA) .LT. 1.E-4) GO TO 320                       SUP01349
      U = ATAN2(SINB*COSR+COSB*SINR,SINB*SINR-COSB*COSR)*RTD            SUP01350
      V = 90.-ACOS(COSA)*RTD                                            SUP01351
      GO TO 305                                                         SUP01352
C                                                                       SUP01353
C MERCATOR, ARBITRARY POLE AND ORIENTATION.                             SUP01354
C                                                                       SUP01355
  190 IF ((1.-COSA*COSA) .LT. 2.E-6) GO TO 320                          SUP01356
      U = ATAN2(SINB*COSR+COSB*SINR,SINB*SINR-COSB*COSR)                SUP01357
      V = ALOG((1.+COSA)*OVSINA)                                        SUP01358
      GO TO 305                                                         SUP01359
C                                                                       SUP01360
C MOLLWEIDE, ARBITRARY POLE AND ORIENTATION.                            SUP01361
C                                                                       SUP01362
  200 IF (ABS(1.-COSA*COSA) .LT. 2.E-6) GO TO 320                       SUP01363
      U = ATAN2(SINB*COSR+COSB*SINR,SINB*SINR-COSB*COSR)*TOVPI          SUP01364
      UDIF = ABS(U-OLDU)                                                SUP01365
      OLDU = U                                                          SUP01366
      V = COSA                                                          SUP01367
      U = U*SQRT(1.-V*V)                                                SUP01368
      GO TO 310                                                         SUP01369
C                                                                       SUP01370
C CYLINDRICAL EQUIDISTANT FOR POLAT = ROT = 0.     [ 11 ]               SUP01371
C                                                                       SUP01372
  210 V = RLAT                                                          SUP01373
      GO TO 305                                                         SUP01374
C                                                                       SUP01375
C MERCATOR                              [ 12 ]                          SUP01376
C                                                                       SUP01377
  220 U = U*DTR                                                         SUP01378
      V = ALOG(TAN(0.00872664*(RLAT+90.0001)))                          SUP01379
      GO TO 305                                                         SUP01380
C                                                                       SUP01381
C MOLLWEIDE                             [ 13 ]                          SUP01382
C                                                                       SUP01383
  230 U = U*OV90                                                        SUP01384
      V = SIN(RLAT*DTR)                                                 SUP01385
      UDIF = ABS(U-OLDU)                                                SUP01386
      OLDU = U                                                          SUP01387
      U = U*SQRT(1.-V*V)                                                SUP01388
      GO TO 310                                                         SUP01389
C                                                                       SUP01390
C TERMINAL PHASE    (1,2,4,5,6)                                         SUP01391
C                                                                       SUP01392
  300 U = R*(SINB*COSR+COSB*SINR)                                       SUP01393
      V = R*(COSB*COSR-SINB*SINR)                                       SUP01394
C                                                                       SUP01395
C CHECK FOR CROSSOVER                                                   SUP01396
C                                                                       SUP01397
  305 UDIF = ABS(U-OLDU)                                                SUP01398
      OLDU = U                                                          SUP01399
  310 VDIF = ABS(V-OLDV)                                                SUP01400
      OLDV = V                                                          SUP01401
      ICROSS = 0                                                        SUP01402
      IF (UDIF.GT.UEPS .OR. VDIF.GT.VEPS) ICROSS = 1                    SUP01403
      RETURN                                                            SUP01404
C                                                                       SUP01405
C DISPENSE WITH UNDEFINED POINTS                                        SUP01406
C                                                                       SUP01407
  320 U = 1.E12                                                         SUP01408
      ICROSS = 0                                                        SUP01409
      IF (ABS(U-OLDU) .GT. UEPS) ICROSS = 1                             SUP01410
      OLDU = U                                                          SUP01411
      RETURN                                                            SUP01412
C                                                                       SUP01413
      END                                                               SUP01414
      SUBROUTINE QVEC                                                   SUP01415
C                                                                       SUP01416
C THIS SUBROUTINE TRANSFORMS  AND PLOTS LINE SEGMENTS                   SUP01417
C FOR SUPMAP AND OTHERS                                                 SUP01418
C                                                                       SUP01419
C INPUTS  (PASSED THROUGH COMMON.)                                      SUP01420
C                                                                       SUP01421
C     (RLAT,RLON)    NEXT POINT TO BE PLOTTED                           SUP01422
C     IFST    -  A FLAG USED TO SIGNAL THE FIRST POINT OF A LINE SEGMENTSUP01423
C          = 0  _    START A NEW LINE                                   SUP01424
C          = 1  _    CONTINUATION OF A LINE                             SUP01425
C                                                                       SUP01426
C OTHER VARIABLES                                                       SUP01427
C                                                                       SUP01428
C     (U,V)        NEXT POINT TRANSFORMED TO THE VIRTUAL SCREEN BY      SUP01429
C     SUPCONQ ICROSS   -  A FLAG RETURNED BY SUPCONQ FOR CYLINDRICAL    SUP01430
C     PROJECTIONS IGO  = 0  _    LAST POINT NOT PLOTTED                 SUP01431
C          = 1  _    LAST POINT WAS PLOTTED.                            SUP01432
C     (U1,V1),(U2,V2)   PARAMETERS PASSED TO SUPTRP.                    SUP01433
C                                                                       SUP01434
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01435
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01436
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01437
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01438
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01439
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01440
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01441
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01442
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01443
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01444
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01445
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01446
     2                IIER                                              SUP01447
C                                                                       SUP01448
      SQU(X) = (X)*(X)                                                  SUP01449
C                                                                       SUP01450
C                                                                       SUP01451
C TRANSFORM THE POINT                                                   SUP01452
C                                                                       SUP01453
      CALL QCON                                                         SUP01454
C                                                                       SUP01455
C     HAVE WE FLIPPED TO OTHER SIDE OF FRAME                            SUP01456
C                                                                       SUP01457
      IF (ICROSS .NE. 0) IGO = 0                                        SUP01458
C                                                                       SUP01459
C     ARE WE WITHIN THE FRAME.                                          SUP01460
C                                                                       SUP01461
      IF (U.GT.UMAX .OR. U.LT.UMIN .OR. V.GT.VMAX .OR. V.LT.VMIN)       SUP01462
     1    GO TO 20                                                      SUP01463
      IF (IGO .EQ. 0) GO TO 30                                          SUP01464
C                                                                       SUP01465
C CONTINUE LINE                                                         SUP01466
C     CHECK PROXIMITY TO PREVIOUS POINT.                                SUP01467
C                                                                       SUP01468
    5 IF ((SQU(U-UOLD)+SQU(V-VOLD))*DS .LE. DI) RETURN                  SUP01469
      CALL VECPLT                                                       SUP01470
   10 UOLD = U                                                          SUP01471
      VOLD = V                                                          SUP01472
      IGOLD = IGO                                                       SUP01473
      RETURN                                                            SUP01474
C                                                                       SUP01475
C THIS POINT LIES OUTSIDE THE FRAME                                     SUP01476
C                                                                       SUP01477
   20 IGO = 0                                                           SUP01478
      IF (IFST .NE. 0) GO TO 65                                         SUP01479
C                                                                       SUP01480
C                                                                       SUP01481
      IF (IGOLD .EQ. 0) GO TO 10                                        SUP01482
C                                                                       SUP01483
C     IT WAS INSIDE _ INTERPOLATE TO EDGE OF FRAME                      SUP01484
C     STATUS OF LAST POINT.   IF NOT INSIDE FRAME  GO ON                SUP01485
C                                                                       SUP01486
C IF UNINTERPOLATABLE                                                   SUP01487
C                                                                       SUP01488
      IF (ICROSS .NE. 0) GO TO 70                                       SUP01489
C                                                                       SUP01490
      U1 = UOLD                                                         SUP01491
      V1 = VOLD                                                         SUP01492
      U2 = U                                                            SUP01493
      V2 = V                                                            SUP01494
      CALL SUPTRP                                                       SUP01495
C                                                                       SUP01496
C     CHECK PROXIMITY TO PREVIOUS POINT.                                SUP01497
C                                                                       SUP01498
      IF ((SQU(U-UOLD)+SQU(V-VOLD))*DS .LE. DI) GO TO 25                SUP01499
      CALL VECPLT                                                       SUP01500
   25 UOLD = U2                                                         SUP01501
      VOLD = V2                                                         SUP01502
      IGOLD = 0                                                         SUP01503
      RETURN                                                            SUP01504
C                                                                       SUP01505
C     THIS POINT IS WITHIN THE FRAME                                    SUP01506
C                                                                       SUP01507
C     IS IT THE FIRST POINT OF A LINE                                   SUP01508
C                                                                       SUP01509
   30 IF (IFST .NE. 0) GO TO 60                                         SUP01510
      IF (IGOLD .EQ. 0) GO TO 50                                        SUP01511
C                                                                       SUP01512
C     THE PREVIOUS POINT WAS INSIDE THE FRAME ON THE OTHER SIDE.        SUP01513
C     START A NEW LINE                                                  SUP01514
C                                                                       SUP01515
   40 CALL FRSTPT (U,V)                                                 SUP01516
      IGO = 1                                                           SUP01517
      GO TO 10                                                          SUP01518
C                                                                       SUP01519
C                                                                       SUP01520
C     LAST POINT NOT IN FRAME - THIS ONE IS                             SUP01521
C                                                                       SUP01522
   50 IF (ICROSS .NE. 0) GO TO 40                                       SUP01523
C                                                                       SUP01524
C     INTERPOLATE BACK TO EDGE                                          SUP01525
C                                                                       SUP01526
      U1 = U                                                            SUP01527
      V1 = V                                                            SUP01528
      U2 = UOLD                                                         SUP01529
      V2 = VOLD                                                         SUP01530
      CALL SUPTRP                                                       SUP01531
      CALL FRSTPT (U,V)                                                 SUP01532
      IGO = 1                                                           SUP01533
      IGOLD = 1                                                         SUP01534
      UOLD = U                                                          SUP01535
      VOLD = V                                                          SUP01536
      U = U1                                                            SUP01537
      V = V1                                                            SUP01538
      GO TO 5                                                           SUP01539
C                                                                       SUP01540
C     FIRST POINT ON LINE SEGMENT  -  CHECK FOR DUPLICATION OF END POINTSUP01541
C                                                                       SUP01542
   60 IF (U.NE.UOLD .OR. V.NE.VOLD) CALL FRSTPT (U,V)                   SUP01543
      IGO = 1                                                           SUP01544
   65 IFST = 0                                                          SUP01545
      GO TO 10                                                          SUP01546
C                                                                       SUP01547
C IGNORE UNDEFINED POINT                                                SUP01548
C                                                                       SUP01549
   70 IFST = 1                                                          SUP01550
      GO TO 10                                                          SUP01551
C                                                                       SUP01552
      END                                                               SUP01553
      SUBROUTINE SUPTRP                                                 SUP01554
C                                                                       SUP01555
C THE INTERPOLATION ROUTINE                                             SUP01556
C     FINDS (U,V) ON THE EDGE OF THE FRAME NEAREST (U1,V1)              SUP01557
C     (U1,V1) MUST LIE WITHIN THE FRAME, (U2,V2) WITHOUT.               SUP01558
C                                                                       SUP01559
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01560
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01561
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01562
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01563
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01564
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01565
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01566
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01567
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01568
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01569
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01570
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01571
     2                IIER                                              SUP01572
C                                                                       SUP01573
      F(V) = (V-V2)*DU/DV+U2                                            SUP01574
      G(U) = (U-U2)*DV/DU+V2                                            SUP01575
C                                                                       SUP01576
C                                                                       SUP01577
C FIND INDEX TO (U2,V2)                                                 SUP01578
C                                                                       SUP01579
C             I     I                                                   SUP01580
C           5 I  4  I 6                                                 SUP01581
C             I     I                                                   SUP01582
C          -------------                                                SUP01583
C             I     I                                                   SUP01584
C           2 I  1  I 3                                                 SUP01585
C             I     I                                                   SUP01586
C          -------------                                                SUP01587
C             I     I                                                   SUP01588
C           8 I  7  I 9                                                 SUP01589
C             I     I                                                   SUP01590
C                                                                       SUP01591
      I = 1                                                             SUP01592
      DU = U1-U2                                                        SUP01593
      DV = V1-V2                                                        SUP01594
      A = U2-UMIN                                                       SUP01595
      B = U2-UMAX                                                       SUP01596
      C = V2-VMIN                                                       SUP01597
      D = V2-VMAX                                                       SUP01598
      IF (A) 110,140,120                                                SUP01599
  110 I = I+1                                                           SUP01600
      GO TO 140                                                         SUP01601
  120 IF (B) 140,140,130                                                SUP01602
  130 I = I+2                                                           SUP01603
  140 IF (C) 150,200,160                                                SUP01604
  150 I = I+6                                                           SUP01605
      GO TO 200                                                         SUP01606
  160 IF (D) 200,200,170                                                SUP01607
  170 I = I+3                                                           SUP01608
C                                                                       SUP01609
  200 GO TO (900,210,220,230,240,250,260,270,280),I                     SUP01610
C                                                                       SUP01611
  210 U = UMIN                                                          SUP01612
      GO TO 300                                                         SUP01613
  220 U = UMAX                                                          SUP01614
      GO TO 300                                                         SUP01615
  230 V = VMAX                                                          SUP01616
      GO TO 350                                                         SUP01617
  240 IF (F(VMAX)-UMIN) 210,230,230                                     SUP01618
  250 IF (F(VMAX)-UMAX) 230,230,220                                     SUP01619
  260 V = VMIN                                                          SUP01620
      GO TO 350                                                         SUP01621
  270 IF (F(VMIN)-UMIN) 210,260,260                                     SUP01622
  280 IF (F(VMIN)-UMAX) 260,260,220                                     SUP01623
C                                                                       SUP01624
C INTERPOLATE                                                           SUP01625
C                                                                       SUP01626
  300 V = G(U)                                                          SUP01627
      RETURN                                                            SUP01628
  350 U = F(V)                                                          SUP01629
      RETURN                                                            SUP01630
C                                                                       SUP01631
C ERROR EXIT                                                            SUP01632
C                                                                       SUP01633
  900 U = U2                                                            SUP01634
      V = V2                                                            SUP01635
      RETURN                                                            SUP01636
      END                                                               SUP01637
      SUBROUTINE VECPLT                                                 SUP01638
C PLOTS THE LINE SEGMENT FROM (UOLD,VOLD) TO (U,V)                      SUP01639
C INPUTS   (PASSED THROUGH COMMON)                                      SUP01640
C     (UOLD,VOLD)    THE LAST POINT PLOTTED                             SUP01641
C     (U,V)          THE NEXT POINT                                     SUP01642
C     IDOT           CONTROL FLAG  [ DOT ! PLOT ]                       SUP01643
      COMMON /SUPMP3/ KPT, XPOINT(50), YPOINT(50)                       SUP01644
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01645
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01646
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01647
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01648
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01649
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01650
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01651
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01652
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01653
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01654
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01655
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01656
     2                IIER                                              SUP01657
      IF (IDOT.NE.0) GO TO 10                                           SUP01658
C PLOT A VECTOR.                                                        SUP01659
      CALL VECTOR (U,V)                                                 SUP01660
      RETURN                                                            SUP01661
C DOT, STORE IT IN BUFFER FOR NOW.                                      SUP01662
   10 DU = U-UOLD                                                       SUP01663
      DV = V-VOLD                                                       SUP01664
      I = -1 + IFIX ((ABS(DU)+ABS(DV))*DSRDI)                           SUP01665
      IF (I .LE. 0) GO TO 30                                            SUP01666
      A = 1./FLOAT(I+1)                                                 SUP01667
      DU = DU*A                                                         SUP01668
      DV = DV*A                                                         SUP01669
      UO = U                                                            SUP01670
      VO = V                                                            SUP01671
      U = UOLD                                                          SUP01672
      V = VOLD                                                          SUP01673
      DO 20 K=1,I                                                       SUP01674
         U = U+DU                                                       SUP01675
         V = V+DV                                                       SUP01676
         IF (KPT.LT.50)  GOTO 15                                        SUP01677
         CALL POINTS (XPOINT, YPOINT, KPT, 0, 0)                        SUP01678
         KPT = 0                                                        SUP01679
15       KPT = KPT + 1                                                  SUP01680
         XPOINT(KPT) = U                                                SUP01681
         YPOINT(KPT) = V                                                SUP01682
   20 CONTINUE                                                          SUP01683
      U = UO                                                            SUP01684
      V = VO                                                            SUP01685
30    IF (KPT.LT.50)  GOTO 35                                           SUP01686
      CALL POINTS (XPOINT, YPOINT, KPT, 0, 0)                           SUP01687
      KPT = 0                                                           SUP01688
35    KPT = KPT + 1                                                     SUP01689
      XPOINT(KPT) = U                                                   SUP01690
      YPOINT(KPT) = V                                                   SUP01691
      RETURN                                                            SUP01692
      END                                                               SUP01693
      SUBROUTINE SUPVEC (XLAT,XLON)                                     SUP01694
C                                                                       SUP01695
C THIS SUBROUTINE ALLOWS THE USER TO DRAW LINES ON THE VIRTUAL SCREEN   SUP01696
C SET UP BY SUPMAP, UNENCUMBERED BY BY THE DECISIONS AS TO WHETHER IT   SUP01697
C WILL BE VISABLE THROUGH THE WINDOW.                                   SUP01698
C USE THE ENTRY POINTS SUPFST AND SUPVEC IN EXACTLY THE SAME MANNER     SUP01699
C AS FRSTPT AND VECTOR RESPECTIVELY.                                    SUP01700
C                                                                       SUP01701
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01702
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01703
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01704
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01705
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01706
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01707
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01708
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01709
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01710
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01711
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01712
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01713
     2                IIER                                              SUP01714
C                                                                       SUP01715
      RLAT = XLAT                                                       SUP01716
      RLON = XLON                                                       SUP01717
      CALL QVEC                                                         SUP01718
      RETURN                                                            SUP01719
      END                                                               SUP01720
      SUBROUTINE SUPFST (XLAT,XLON)                                     SUP01721
C                                                                       SUP01722
C                                                                       SUP01723
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01724
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01725
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01726
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01727
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01728
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01729
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01730
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01731
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01732
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01733
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01734
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01735
     2                IIER                                              SUP01736
C                                                                       SUP01737
      IGO = 0                                                           SUP01738
      IFST = 1                                                          SUP01739
      RLAT = XLAT                                                       SUP01740
      RLON = XLON                                                       SUP01741
      CALL QVEC                                                         SUP01742
      RETURN                                                            SUP01743
C                                                                       SUP01744
      END                                                               SUP01745
      SUBROUTINE SUPCON (XLAT,XLON,XU,XV)                               SUP01746
C                                                                       SUP01747
C THIS SUBROUTINE IS PROVIDED TO RETAIN COMPATIBILITY WITH USER'S       SUP01748
C PROGRAMS.                                                             SUP01749
C                                                                       SUP01750
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01751
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01752
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01753
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01754
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01755
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01756
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01757
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01758
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01759
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01760
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01761
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01762
     2                IIER                                              SUP01763
C                                                                       SUP01764
      RLAT = XLAT                                                       SUP01765
      RLON = XLON                                                       SUP01766
      CALL QCON                                                         SUP01767
      XU = U                                                            SUP01768
      XV = V                                                            SUP01769
      RETURN                                                            SUP01770
      END                                                               SUP01771
      BLOCKDATA                                                         SUP01772
      COMMON /SUPMP1/ DTR        ,ILF        ,PHIOC      ,SINR       ,  SUP01773
     1                U2         ,CONE       ,EPS        ,ILTS       ,  SUP01774
     2                PI         ,TOVPI      ,V          ,CON1       ,  SUP01775
     3                ICROSS     ,IOUT       ,POLONG     ,U          ,  SUP01776
     4                VEPS       ,CON2       ,IDOT       ,IPROJ      ,  SUP01777
     5                RLAT       ,UEPS       ,VMAX       ,COSO       ,  SUP01778
     6                IFST       ,JGR        ,RLON       ,UMAX       ,  SUP01779
     7                VMIN       ,COSR       ,IGO        ,OV90       ,  SUP01780
     8                RTD        ,UMIN       ,VOLD       ,DI         ,  SUP01781
     9                IGOLD      ,PHIA       ,SGN        ,UOLD       ,  SUP01782
     +                V1         ,DS         ,IGRID      ,PHIO       ,  SUP01783
     1                SINO       ,U1         ,V2         ,DSRDI      ,  SUP01784
     2                IIER                                              SUP01785
C                                                                       SUP01786
      DATA   CON1 / 1.000000000001/                                     SUP01787
      DATA   CON2 / 179.999999999999/                                   SUP01788
      DATA     DI / 16./                                                SUP01789
      DATA    DTR / 1.7453292519943E-2/                                 SUP01790
      DATA    EPS / 1.E-12/                                             SUP01791
      DATA   OV90 / 1.11111111111111E-2/                                SUP01792
      DATA     PI / 3.1415926535898/                                    SUP01793
      DATA    RTD / 57.295779513082/                                    SUP01794
      DATA  TOVPI / 0.63661977236758/                                   SUP01795
      DATA   UOLD / 0.0 /                                               SUP01796
      DATA VOLD / 0.0 /                                                 SUP01797
C                                                                       SUP01798
C====== REVISION HISTORY ==========================================     SUP01799
C                                                                       SUP01800
C OCT 82        ADDED BUFFERING OF POINTS FOR DOTTED CONTINENTAL        SUP01801
C               OUTLINES.  REVISED DASH PATTERNS TO 16 BITS FOR         SUP01802
C               GRID LINES.  CHANGED OBSOLETE DASHLN AND OPTION         SUP01803
C               CALLS TO OPTN CALLS.  INCLUDED IN COMMENTS ON           SUP01804
C               PORTABILITY A WARNING OF THE ASSUMPTION FOR JGRID       SUP01805
C               AND IUSOUT PARAMETERS THAT THE COMPUTER CAN             SUP01806
C               REPRESENT -0.  THIS NON-PORTABILITY IS A HOLDOVER       SUP01807
C               FROM 1'S COMPLEMENT COMPUTERS.                          SUP01808
C                                                                       SUP01809
C===================================================================    SUP01810
C                                                                       SUP01811
      END                                                               SUP01812
                                                                                                                                                                                                                                                                                                                                      