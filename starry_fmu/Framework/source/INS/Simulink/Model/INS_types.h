/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: INS_types.h
 *
 * Code generated for Simulink model 'INS'.
 *
 * Model version                  : 1.141
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Oct 28 12:14:29 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_INS_types_h_
#define RTW_HEADER_INS_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_IMU_Bus_
#define DEFINED_TYPEDEF_FOR_IMU_Bus_

typedef struct {
  real32_T gyro[3];
  real32_T accel[3];
} IMU_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Mag_Bus_
#define DEFINED_TYPEDEF_FOR_Mag_Bus_

typedef struct {
  real32_T mag[3];
} Mag_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_GPS_Bus_
#define DEFINED_TYPEDEF_FOR_GPS_Bus_

typedef struct {
  real32_T xy[2];
} GPS_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BARO_Bus_
#define DEFINED_TYPEDEF_FOR_BARO_Bus_

typedef struct {
  real32_T z;
} BARO_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_State_Bus_
#define DEFINED_TYPEDEF_FOR_State_Bus_

typedef struct {
  real32_T pos_O[3];
  real32_T vel_O[3];
  real32_T quat[4];
  real32_T gyr_bias[3];
  real32_T az_bias;
} State_Bus;

#endif

/* Custom Type definition for MATLAB Function: '<S3>/MATLAB Function' */
#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif                                 /*struct_tag_skA4KFEZ4HPkJJBOYCrevdH*/

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH_INS_T
#define typedef_skA4KFEZ4HPkJJBOYCrevdH_INS_T

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH_INS_T;

#endif                                 /*typedef_skA4KFEZ4HPkJJBOYCrevdH_INS_T*/

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 /*struct_tag_sJCxfmxS8gBOONUZjbjUd9E*/

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_INS_T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_INS_T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E_INS_T;

#endif                                 /*typedef_sJCxfmxS8gBOONUZjbjUd9E_INS_T*/

/* Custom Type definition for MATLAB Function: '<S15>/Correct' */
#ifndef struct_tag_sIY0rnKyHfDhNDWdUCMAeuG
#define struct_tag_sIY0rnKyHfDhNDWdUCMAeuG

struct tag_sIY0rnKyHfDhNDWdUCMAeuG
{
  char_T FcnName[14];
  boolean_T IsSimulinkFcn;
  real_T NumberOfExtraArgumentInports;
  boolean_T HasJacobian;
  char_T JacobianFcnName[15];
  boolean_T HasAdditiveNoise;
};

#endif                                 /*struct_tag_sIY0rnKyHfDhNDWdUCMAeuG*/

#ifndef typedef_sIY0rnKyHfDhNDWdUCMAeuG_INS_T
#define typedef_sIY0rnKyHfDhNDWdUCMAeuG_INS_T

typedef struct tag_sIY0rnKyHfDhNDWdUCMAeuG sIY0rnKyHfDhNDWdUCMAeuG_INS_T;

#endif                                 /*typedef_sIY0rnKyHfDhNDWdUCMAeuG_INS_T*/

/* Custom Type definition for MATLAB Function: '<S13>/Correct' */
#ifndef struct_tag_sU6tjw74S3mIDlJBFoBIZQD
#define struct_tag_sU6tjw74S3mIDlJBFoBIZQD

struct tag_sU6tjw74S3mIDlJBFoBIZQD
{
  char_T FcnName[15];
  boolean_T IsSimulinkFcn;
  real_T NumberOfExtraArgumentInports;
  boolean_T HasJacobian;
  char_T JacobianFcnName[16];
  boolean_T HasAdditiveNoise;
};

#endif                                 /*struct_tag_sU6tjw74S3mIDlJBFoBIZQD*/

#ifndef typedef_sU6tjw74S3mIDlJBFoBIZQD_INS_T
#define typedef_sU6tjw74S3mIDlJBFoBIZQD_INS_T

typedef struct tag_sU6tjw74S3mIDlJBFoBIZQD sU6tjw74S3mIDlJBFoBIZQD_INS_T;

#endif                                 /*typedef_sU6tjw74S3mIDlJBFoBIZQD_INS_T*/

/* Custom Type definition for MATLAB Function: '<S17>/Predict' */
#ifndef struct_tag_sDcXNRkCIZmlJZ9dkNCiirF
#define struct_tag_sDcXNRkCIZmlJZ9dkNCiirF

struct tag_sDcXNRkCIZmlJZ9dkNCiirF
{
  char_T FcnName[11];
  boolean_T IsSimulinkFcn;
  real_T NumberOfExtraArgumentInports;
  char_T JacobianFcnName[19];
  real_T HasJacobian;
  boolean_T HasAdditiveNoise;
};

#endif                                 /*struct_tag_sDcXNRkCIZmlJZ9dkNCiirF*/

#ifndef typedef_sDcXNRkCIZmlJZ9dkNCiirF_INS_T
#define typedef_sDcXNRkCIZmlJZ9dkNCiirF_INS_T

typedef struct tag_sDcXNRkCIZmlJZ9dkNCiirF sDcXNRkCIZmlJZ9dkNCiirF_INS_T;

#endif                                 /*typedef_sDcXNRkCIZmlJZ9dkNCiirF_INS_T*/

/* Forward declaration for rtModel */
typedef struct tag_RTM_INS_T RT_MODEL_INS_T;

#endif                                 /* RTW_HEADER_INS_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
