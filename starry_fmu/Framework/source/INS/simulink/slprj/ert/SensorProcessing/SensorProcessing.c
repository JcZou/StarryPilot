/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: SensorProcessing.c
 *
 * Code generated for Simulink model 'SensorProcessing'.
 *
 * Model version                  : 1.174
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Sun Dec 16 13:15:05 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "SensorProcessing.h"
#include "SensorProcessing_private.h"
#include "rt_powd_snf.h"

int_T SensorProcessing_GlobalTID[2];
const rtTimingBridge* SensorProcessing_TimingBrdg;
MdlrefDW_SensorProcessing_T SensorProcessing_MdlrefDW;

/* Block signals (default storage) */
B_SensorProcessing_c_T SensorProcessing_B;

/* Block states (default storage) */
DW_SensorProcessing_f_T SensorProcessing_DW;

/* System initialize for referenced model: 'SensorProcessing' */
void SensorProcessing_Init(void)
{
	/* SystemInitialize for Enabled SubSystem: '<S1>/Pressure_Altitude' */
	/* InitializeConditions for Memory: '<S11>/Memory3' */
	SensorProcessing_DW.Memory3_PreviousInput = true;

	/* End of SystemInitialize for SubSystem: '<S1>/Pressure_Altitude' */
}

/* Output and update for referenced model: 'SensorProcessing' */
void SensorProcessing(const real32_T rtu_IMU1_gyr_radPs_B[3], const real32_T
                      rtu_IMU1_acc_mPs2_B[3], const uint32_T
                      *rtu_IMU1_timestamp_ms, const real32_T rtu_Mag_mag_ga_B[3],
                      const uint32_T* rtu_Mag_timestamp_ms, const uint8_T
                      *rtu_GPS_uBlox_fixType, const uint8_T* rtu_GPS_uBlox_numSV,
                      const int32_T* rtu_GPS_uBlox_lon, const int32_T
                      *rtu_GPS_uBlox_lat, const int32_T* rtu_GPS_uBlox_height,
                      const uint32_T* rtu_GPS_uBlox_hAcc, const int32_T
                      *rtu_GPS_uBlox_velN, const int32_T* rtu_GPS_uBlox_velE,
                      const int32_T* rtu_GPS_uBlox_velD, const uint32_T
                      *rtu_GPS_uBlox_sAcc, const uint32_T
                      *rtu_GPS_uBlox_timestamp_ms, const int32_T
                      *rtu_Baro_pressure_Pa, const real32_T
                      *rtu_Baro_temperature_deg, const uint32_T
                      *rtu_Baro_timestamp_ms, const real32_T
                      rtu_Sensor_Param_gyr_rotM[9], const real32_T
                      rtu_Sensor_Param_gyr_bias[3], const real32_T
                      rtu_Sensor_Param_acc_rotM[9], const real32_T
                      rtu_Sensor_Param_acc_bias[3], const real32_T
                      rtu_Sensor_Param_mag_rotM[9], const real32_T
                      rtu_Sensor_Param_mag_bias[3], real32_T
                      rty_Sensor_Data_IMU_Data_rot_ra[3], real32_T
                      rty_Sensor_Data_IMU_Data_sfor_m[3], boolean_T
                      *rty_Sensor_Data_IMU_Data_valid, real32_T
                      rty_Sensor_Data_Mag_Data_mag_ga[3], boolean_T
                      *rty_Sensor_Data_Mag_Data_valid, real32_T
                      *rty_Sensor_Data_GPS_Data_pos_qu, real32_T
                      *rty_Sensor_Data_GPS_Data_vel_qu, real32_T
                      *rty_Sensor_Data_GPS_Data_status, real_T
                      *rty_Sensor_Data_GPS_Data_lon_ra, real_T
                      *rty_Sensor_Data_GPS_Data_lat_ra, real32_T
                      *rty_Sensor_Data_GPS_Data_height, real32_T
                      *rty_Sensor_Data_GPS_Data_velN_m, real32_T
                      *rty_Sensor_Data_GPS_Data_velE_m, real32_T
                      *rty_Sensor_Data_GPS_Data_velD_m, uint16_T
                      *rty_Sensor_Data_GPS_Data_numSV, boolean_T
                      *rty_Sensor_Data_Baro_Data_valid, real32_T
                      *rty_Sensor_Data_Baro_Data_altit, real32_T
                      *rty_Sensor_Data_Baro_Data_relat)
{
	uint8_T rtb_Compare_pz;
	uint8_T rtb_Compare_n;
	real32_T rtb_Sum3;
	boolean_T rtb_init_ref_alt;
	real_T rtb_Divide2;
	boolean_T rtb_LogicalOperator4_n;
	real32_T rtu_IMU1_acc_mPs2_B_0[3];
	real32_T rtu_Mag_mag_ga_B_0[3];
	int32_T i;
	real32_T rtu_IMU1_gyr_radPs_B_idx_0;
	real32_T rtu_IMU1_gyr_radPs_B_idx_1;
	real32_T rtu_IMU1_gyr_radPs_B_idx_2;

	/* DiscreteIntegrator: '<S23>/Discrete-Time Integrator' incorporates:
	 *  RelationalOperator: '<S25>/FixPt Relational Operator'
	 *  UnitDelay: '<S25>/Delay Input1'
	 *
	 * Block description for '<S25>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	if(*rtu_IMU1_timestamp_ms != SensorProcessing_DW.DelayInput1_DSTATE) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE = 0U;
	}

	/* DiscreteIntegrator: '<S31>/Discrete-Time Integrator' incorporates:
	 *  RelationalOperator: '<S33>/FixPt Relational Operator'
	 *  UnitDelay: '<S33>/Delay Input1'
	 *
	 * Block description for '<S33>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	if(*rtu_Mag_timestamp_ms != SensorProcessing_DW.DelayInput1_DSTATE_b) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_m = 0U;
	}

	if(rtmIsSampleHit(1, 0)) {
		/* UnitDelay: '<S18>/Unit Delay' */
		SensorProcessing_B.UnitDelay = SensorProcessing_DW.UnitDelay_DSTATE;

		/* Rounding: '<S18>/Rounding Function' incorporates:
		 *  Gain: '<S18>/Gain1'
		 */
		SensorProcessing_B.RoundingFunction = floorf(0.001F * (real32_T)
		                                      SensorProcessing_B.UnitDelay);
	}

	/* RelationalOperator: '<S20>/Compare' incorporates:
	 *  Constant: '<S20>/Constant'
	 */
	rtb_Compare_pz = (uint8_T)(*rtu_GPS_uBlox_fixType == 3);

	/* DiscreteIntegrator: '<S12>/Discrete-Time Integrator' incorporates:
	 *  RelationalOperator: '<S16>/FixPt Relational Operator'
	 *  UnitDelay: '<S16>/Delay Input1'
	 *
	 * Block description for '<S16>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	if(*rtu_GPS_uBlox_timestamp_ms != SensorProcessing_DW.DelayInput1_DSTATE_g) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e = 0U;
	}

	/* RelationalOperator: '<S21>/Compare' incorporates:
	 *  Constant: '<S21>/Constant'
	 *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator'
	 */
	rtb_Compare_n = (uint8_T)(SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e <
	                          500U);

	/* Sum: '<S18>/Sum3' incorporates:
	 *  Logic: '<S18>/Logical Operator1'
	 */
	rtb_Sum3 = (real32_T)((SensorProcessing_B.RoundingFunction != 0.0F) &&
	                      (rtb_Compare_pz != 0) && (rtb_Compare_n != 0)) +
	           SensorProcessing_B.RoundingFunction;

	/* RelationalOperator: '<S14>/Compare' incorporates:
	 *  Constant: '<S14>/Constant'
	 */
	rtb_init_ref_alt = (rtb_Sum3 > 1.0F);

	/* DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' incorporates:
	 *  RelationalOperator: '<S17>/FixPt Relational Operator'
	 *  UnitDelay: '<S17>/Delay Input1'
	 *
	 * Block description for '<S17>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	if(*rtu_GPS_uBlox_timestamp_ms != SensorProcessing_DW.DelayInput1_DSTATE_c) {
		SensorProcessing_DW.DiscreteTimeIntegrator1_DSTATE = 0U;
	}

	/* DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
	 *  RelationalOperator: '<S8>/FixPt Relational Operator'
	 *  UnitDelay: '<S8>/Delay Input1'
	 *
	 * Block description for '<S8>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	if(*rtu_Baro_timestamp_ms != SensorProcessing_DW.DelayInput1_DSTATE_c0) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_c = 0U;
	}

	/* Logic: '<S5>/Logical Operator4' incorporates:
	 *  Constant: '<S10>/Lower Limit'
	 *  Constant: '<S10>/Upper Limit'
	 *  Constant: '<S7>/Constant'
	 *  Constant: '<S9>/Lower Limit'
	 *  Constant: '<S9>/Upper Limit'
	 *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
	 *  Logic: '<S10>/AND'
	 *  Logic: '<S5>/Logical Operator'
	 *  Logic: '<S9>/AND'
	 *  RelationalOperator: '<S10>/Lower Test'
	 *  RelationalOperator: '<S10>/Upper Test'
	 *  RelationalOperator: '<S7>/Compare'
	 *  RelationalOperator: '<S9>/Lower Test'
	 *  RelationalOperator: '<S9>/Upper Test'
	 */
	rtb_LogicalOperator4_n = ((1000 < *rtu_Baro_pressure_Pa) &&
	                          (*rtu_Baro_pressure_Pa < 130000) && ((-40.0F < *rtu_Baro_temperature_deg) &&
	                                  (*rtu_Baro_temperature_deg < 85.0F)) &&
	                          (SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_c < 200U));

	/* Outputs for Enabled SubSystem: '<S1>/Pressure_Altitude' incorporates:
	 *  EnablePort: '<S6>/Enable'
	 */
	if(rtb_LogicalOperator4_n) {
		/* Product: '<S6>/Divide' incorporates:
		 *  Constant: '<S6>/p1'
		 *  Gain: '<S6>/Gain'
		 */
		rtb_Divide2 = 0.0010000000002037268 * (real_T) * rtu_Baro_pressure_Pa /
		              101.325;

		/* Math: '<S6>/Pow' */
		if((rtb_Divide2 < 0.0) && (SensorProcessing_ConstB.Divide1 > floor
		                           (SensorProcessing_ConstB.Divide1))) {
			rtb_Divide2 = -rt_powd_snf(-rtb_Divide2, SensorProcessing_ConstB.Divide1);
		} else {
			rtb_Divide2 = rt_powd_snf(rtb_Divide2, SensorProcessing_ConstB.Divide1);
		}

		/* End of Math: '<S6>/Pow' */

		/* DataTypeConversion: '<S6>/Data Type Conversion' incorporates:
		 *  Constant: '<S6>/T1'
		 *  Constant: '<S6>/a'
		 *  Product: '<S6>/Divide2'
		 *  Product: '<S6>/Multiply1'
		 *  Sum: '<S6>/Subtract'
		 */
		SensorProcessing_B.DataTypeConversion = (real32_T)((rtb_Divide2 * 288.15 -
		                                        288.15) / -0.0065);

		/* Switch: '<S11>/Switch' incorporates:
		 *  Memory: '<S11>/Memory3'
		 */
		if(SensorProcessing_DW.Memory3_PreviousInput) {
			SensorProcessing_DW.ref_alt_PreviousInput =
			    SensorProcessing_B.DataTypeConversion;
		}

		/* End of Switch: '<S11>/Switch' */

		/* Sum: '<S11>/Subtract1' */
		SensorProcessing_B.Subtract1 = SensorProcessing_B.DataTypeConversion -
		                               SensorProcessing_DW.ref_alt_PreviousInput;

		/* Update for Memory: '<S11>/Memory3' incorporates:
		 *  Constant: '<S11>/Constant2'
		 */
		SensorProcessing_DW.Memory3_PreviousInput = false;
	}

	/* End of Outputs for SubSystem: '<S1>/Pressure_Altitude' */

	/* Sum: '<S22>/Subtract' */
	rtu_IMU1_gyr_radPs_B_idx_0 = rtu_IMU1_gyr_radPs_B[0] -
	                             rtu_Sensor_Param_gyr_bias[0];
	rtu_IMU1_gyr_radPs_B_idx_1 = rtu_IMU1_gyr_radPs_B[1] -
	                             rtu_Sensor_Param_gyr_bias[1];
	rtu_IMU1_gyr_radPs_B_idx_2 = rtu_IMU1_gyr_radPs_B[2] -
	                             rtu_Sensor_Param_gyr_bias[2];

	for(i = 0; i < 3; i++) {
		/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
		 *  Product: '<S22>/Multiply'
		 */
		rty_Sensor_Data_IMU_Data_rot_ra[i] = 0.0F;
		rty_Sensor_Data_IMU_Data_rot_ra[i] += rtu_Sensor_Param_gyr_rotM[i] *
		                                      rtu_IMU1_gyr_radPs_B_idx_0;
		rty_Sensor_Data_IMU_Data_rot_ra[i] += rtu_Sensor_Param_gyr_rotM[i + 3] *
		                                      rtu_IMU1_gyr_radPs_B_idx_1;
		rty_Sensor_Data_IMU_Data_rot_ra[i] += rtu_Sensor_Param_gyr_rotM[i + 6] *
		                                      rtu_IMU1_gyr_radPs_B_idx_2;

		/* Sum: '<S22>/Subtract1' */
		rtu_IMU1_acc_mPs2_B_0[i] = rtu_IMU1_acc_mPs2_B[i] -
		                           rtu_Sensor_Param_acc_bias[i];
	}

	for(i = 0; i < 3; i++) {
		/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
		 *  Product: '<S22>/Multiply1'
		 */
		rty_Sensor_Data_IMU_Data_sfor_m[i] = 0.0F;
		rty_Sensor_Data_IMU_Data_sfor_m[i] += rtu_Sensor_Param_acc_rotM[i] *
		                                      rtu_IMU1_acc_mPs2_B_0[0];
		rty_Sensor_Data_IMU_Data_sfor_m[i] += rtu_Sensor_Param_acc_rotM[i + 3] *
		                                      rtu_IMU1_acc_mPs2_B_0[1];
		rty_Sensor_Data_IMU_Data_sfor_m[i] += rtu_Sensor_Param_acc_rotM[i + 6] *
		                                      rtu_IMU1_acc_mPs2_B_0[2];

		/* Sum: '<S29>/Subtract1' */
		rtu_Mag_mag_ga_B_0[i] = rtu_Mag_mag_ga_B[i] - rtu_Sensor_Param_mag_bias[i];
	}

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Product: '<S29>/Multiply1'
	 */
	for(i = 0; i < 3; i++) {
		rty_Sensor_Data_Mag_Data_mag_ga[i] = 0.0F;
		rty_Sensor_Data_Mag_Data_mag_ga[i] += rtu_Sensor_Param_mag_rotM[i] *
		                                      rtu_Mag_mag_ga_B_0[0];
		rty_Sensor_Data_Mag_Data_mag_ga[i] += rtu_Sensor_Param_mag_rotM[i + 3] *
		                                      rtu_Mag_mag_ga_B_0[1];
		rty_Sensor_Data_Mag_Data_mag_ga[i] += rtu_Sensor_Param_mag_rotM[i + 6] *
		                                      rtu_Mag_mag_ga_B_0[2];
	}

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Constant: '<S24>/Constant'
	 *  Constant: '<S26>/Lower Limit'
	 *  Constant: '<S26>/Upper Limit'
	 *  Constant: '<S28>/Lower Limit'
	 *  Constant: '<S28>/Upper Limit'
	 *  DiscreteIntegrator: '<S23>/Discrete-Time Integrator'
	 *  Logic: '<S23>/Logical Operator'
	 *  Logic: '<S23>/Logical Operator1'
	 *  Logic: '<S23>/Logical Operator2'
	 *  Logic: '<S23>/Logical Operator4'
	 *  Logic: '<S26>/AND'
	 *  Logic: '<S28>/AND'
	 *  RelationalOperator: '<S24>/Compare'
	 *  RelationalOperator: '<S26>/Lower Test'
	 *  RelationalOperator: '<S26>/Upper Test'
	 *  RelationalOperator: '<S28>/Lower Test'
	 *  RelationalOperator: '<S28>/Upper Test'
	 */
	*rty_Sensor_Data_IMU_Data_valid = ((-38.3972435F < rtu_IMU1_gyr_radPs_B[0]) &&
	                                   (rtu_IMU1_gyr_radPs_B[0] < 38.3972435F) && ((-38.3972435F <
	                                           rtu_IMU1_gyr_radPs_B[1]) && (rtu_IMU1_gyr_radPs_B[1] < 38.3972435F)) &&
	                                   ((-38.3972435F < rtu_IMU1_gyr_radPs_B[2]) && (rtu_IMU1_gyr_radPs_B[2] <
	                                           38.3972435F)) && ((-98.1F < rtu_IMU1_acc_mPs2_B[0]) && (rtu_IMU1_acc_mPs2_B
	                                                   [0] < 98.1F) && ((-98.1F < rtu_IMU1_acc_mPs2_B[1]) && (rtu_IMU1_acc_mPs2_B[1]
	                                                           < 98.1F)) && ((-98.1F < rtu_IMU1_acc_mPs2_B[2]) && (rtu_IMU1_acc_mPs2_B[2] <
	                                                                   98.1F))) && SensorProcessing_ConstB.AND &&
	                                   (SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE < 100U));

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Constant: '<S32>/Constant'
	 *  Constant: '<S35>/Lower Limit'
	 *  Constant: '<S35>/Upper Limit'
	 *  DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
	 *  Logic: '<S31>/Logical Operator'
	 *  Logic: '<S31>/Logical Operator1'
	 *  Logic: '<S31>/Logical Operator4'
	 *  Logic: '<S35>/AND'
	 *  RelationalOperator: '<S32>/Compare'
	 *  RelationalOperator: '<S35>/Lower Test'
	 *  RelationalOperator: '<S35>/Upper Test'
	 */
	*rty_Sensor_Data_Mag_Data_valid = ((-2.0F < rtu_Mag_mag_ga_B[0]) &&
	                                   (rtu_Mag_mag_ga_B[0] < 2.0F) && ((-2.0F < rtu_Mag_mag_ga_B[1]) &&
	                                           (rtu_Mag_mag_ga_B[1] < 2.0F)) && ((-2.0F < rtu_Mag_mag_ga_B[2]) &&
	                                                   (rtu_Mag_mag_ga_B[2] < 2.0F)) && SensorProcessing_ConstB.AND_m &&
	                                   (SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_m < 1000U));

	/* Gain: '<S12>/Gain2' incorporates:
	 *  Constant: '<S12>/Constant1'
	 *  Sum: '<S12>/Sum1'
	 */
	rtu_IMU1_gyr_radPs_B_idx_0 = (3.0F - (real32_T) * rtu_GPS_uBlox_hAcc) * 0.4F;

	/* Saturate: '<S12>/Saturation1' */
	if(rtu_IMU1_gyr_radPs_B_idx_0 > 1.0F) {
		rtu_IMU1_gyr_radPs_B_idx_0 = 1.0F;
	} else {
		if(rtu_IMU1_gyr_radPs_B_idx_0 < 0.0F) {
			rtu_IMU1_gyr_radPs_B_idx_0 = 0.0F;
		}
	}

	/* End of Saturate: '<S12>/Saturation1' */

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Constant: '<S15>/Constant'
	 *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator'
	 *  Product: '<S12>/Product1'
	 *  RelationalOperator: '<S15>/Compare'
	 */
	*rty_Sensor_Data_GPS_Data_pos_qu = (real32_T)
	                                   (SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e < 5000U) *
	                                   rtu_IMU1_gyr_radPs_B_idx_0 * (real32_T)rtb_init_ref_alt;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' */
	*rty_Sensor_Data_GPS_Data_numSV = *rtu_GPS_uBlox_numSV;

	/* Gain: '<S12>/Gain3' incorporates:
	 *  Constant: '<S12>/Constant3'
	 *  Sum: '<S12>/Sum'
	 */
	rtu_IMU1_gyr_radPs_B_idx_0 = (0.5F - (real32_T) * rtu_GPS_uBlox_sAcc) * 2.5F;

	/* Saturate: '<S12>/Saturation' */
	if(rtu_IMU1_gyr_radPs_B_idx_0 > 1.0F) {
		rtu_IMU1_gyr_radPs_B_idx_0 = 1.0F;
	} else {
		if(rtu_IMU1_gyr_radPs_B_idx_0 < 0.0F) {
			rtu_IMU1_gyr_radPs_B_idx_0 = 0.0F;
		}
	}

	/* End of Saturate: '<S12>/Saturation' */

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Constant: '<S13>/Constant'
	 *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
	 *  Product: '<S12>/Product2'
	 *  RelationalOperator: '<S13>/Compare'
	 */
	*rty_Sensor_Data_GPS_Data_vel_qu = (real32_T)
	                                   (SensorProcessing_DW.DiscreteTimeIntegrator1_DSTATE < 5000U) *
	                                   rtu_IMU1_gyr_radPs_B_idx_0 * (real32_T)rtb_init_ref_alt;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  BusCreator: '<Root>/BusConversion_InsertedFor_Bus Creator_at_inport_2'
	 */
	*rty_Sensor_Data_GPS_Data_status = rtb_Sum3;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Gain: '<S2>/Scalefactor2'
	 */
	*rty_Sensor_Data_GPS_Data_lon_ra = 1.745329251783001E-9 * (real_T)
	                                   * rtu_GPS_uBlox_lon;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Gain: '<S2>/Scalefactor1'
	 */
	*rty_Sensor_Data_GPS_Data_lat_ra = 1.745329251783001E-9 * (real_T)
	                                   * rtu_GPS_uBlox_lat;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Gain: '<S2>/Scalefactor3'
	 */
	*rty_Sensor_Data_GPS_Data_height = 0.001F * (real32_T) * rtu_GPS_uBlox_height;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Gain: '<S2>/Scalefactor4'
	 */
	*rty_Sensor_Data_GPS_Data_velN_m = (real32_T) * rtu_GPS_uBlox_velN;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Gain: '<S2>/Scalefactor5'
	 */
	*rty_Sensor_Data_GPS_Data_velE_m = (real32_T) * rtu_GPS_uBlox_velE;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  Gain: '<S2>/Scalefactor6'
	 */
	*rty_Sensor_Data_GPS_Data_velD_m = (real32_T) * rtu_GPS_uBlox_velD;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  BusCreator: '<Root>/BusConversion_InsertedFor_Bus Creator_at_inport_3'
	 */
	*rty_Sensor_Data_Baro_Data_valid = rtb_LogicalOperator4_n;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  BusCreator: '<Root>/BusConversion_InsertedFor_Bus Creator_at_inport_3'
	 */
	*rty_Sensor_Data_Baro_Data_altit = SensorProcessing_B.DataTypeConversion;

	/* SignalConversion: '<Root>/BusConversion_InsertedFor_Sensor_Data_at_inport_0' incorporates:
	 *  BusCreator: '<Root>/BusConversion_InsertedFor_Bus Creator_at_inport_3'
	 */
	*rty_Sensor_Data_Baro_Data_relat = SensorProcessing_B.Subtract1;

	/* Update for UnitDelay: '<S25>/Delay Input1'
	 *
	 * Block description for '<S25>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	SensorProcessing_DW.DelayInput1_DSTATE = *rtu_IMU1_timestamp_ms;

	/* Update for DiscreteIntegrator: '<S23>/Discrete-Time Integrator' incorporates:
	 *  Constant: '<S23>/Constant'
	 */
	if((real32_T)SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE + 2.0F <
	        4.2949673E+9F) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE = (uint32_T)((real32_T)
		        SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE + 2.0F);
	} else {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE = MAX_uint32_T;
	}

	/* End of Update for DiscreteIntegrator: '<S23>/Discrete-Time Integrator' */

	/* Update for UnitDelay: '<S33>/Delay Input1'
	 *
	 * Block description for '<S33>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	SensorProcessing_DW.DelayInput1_DSTATE_b = *rtu_Mag_timestamp_ms;

	/* Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' incorporates:
	 *  Constant: '<S31>/Constant'
	 */
	if((real32_T)SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_m + 2.0F <
	        4.2949673E+9F) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_m = (uint32_T)((real32_T)
		        SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_m + 2.0F);
	} else {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_m = MAX_uint32_T;
	}

	/* End of Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' */
	if(rtmIsSampleHit(1, 0)) {
		/* Sum: '<S18>/Sum2' incorporates:
		 *  Constant: '<S19>/Constant'
		 *  Logic: '<S18>/Logical Operator'
		 *  RelationalOperator: '<S19>/Compare'
		 */
		SensorProcessing_DW.UnitDelay_DSTATE = ((*rtu_GPS_uBlox_hAcc < 5000U) &&
		                                        (rtb_Compare_pz != 0) && (rtb_Compare_n != 0)) +
		                                       SensorProcessing_B.UnitDelay;

		/* Saturate: '<S18>/Saturation2' */
		if(SensorProcessing_DW.UnitDelay_DSTATE > 1111) {
			/* Sum: '<S18>/Sum2' incorporates:
			 *  UnitDelay: '<S18>/Unit Delay'
			 */
			SensorProcessing_DW.UnitDelay_DSTATE = 1111;
		} else {
			if(SensorProcessing_DW.UnitDelay_DSTATE < 0) {
				/* Sum: '<S18>/Sum2' incorporates:
				 *  UnitDelay: '<S18>/Unit Delay'
				 */
				SensorProcessing_DW.UnitDelay_DSTATE = 0;
			}
		}

		/* End of Saturate: '<S18>/Saturation2' */
	}

	/* Update for UnitDelay: '<S16>/Delay Input1'
	 *
	 * Block description for '<S16>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	SensorProcessing_DW.DelayInput1_DSTATE_g = *rtu_GPS_uBlox_timestamp_ms;

	/* Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator' incorporates:
	 *  Constant: '<S12>/Constant'
	 */
	if((real32_T)SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e + 2.0F <
	        4.2949673E+9F) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e = (uint32_T)((real32_T)
		        SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e + 2.0F);
	} else {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_e = MAX_uint32_T;
	}

	/* End of Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator' */

	/* Update for UnitDelay: '<S17>/Delay Input1'
	 *
	 * Block description for '<S17>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	SensorProcessing_DW.DelayInput1_DSTATE_c = *rtu_GPS_uBlox_timestamp_ms;

	/* Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' incorporates:
	 *  Constant: '<S12>/Constant2'
	 */
	if((real32_T)SensorProcessing_DW.DiscreteTimeIntegrator1_DSTATE + 2.0F <
	        4.2949673E+9F) {
		SensorProcessing_DW.DiscreteTimeIntegrator1_DSTATE = (uint32_T)((real32_T)
		        SensorProcessing_DW.DiscreteTimeIntegrator1_DSTATE + 2.0F);
	} else {
		SensorProcessing_DW.DiscreteTimeIntegrator1_DSTATE = MAX_uint32_T;
	}

	/* End of Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' */

	/* Update for UnitDelay: '<S8>/Delay Input1'
	 *
	 * Block description for '<S8>/Delay Input1':
	 *
	 *  Store in Global RAM
	 */
	SensorProcessing_DW.DelayInput1_DSTATE_c0 = *rtu_Baro_timestamp_ms;

	/* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
	 *  Constant: '<S5>/Constant'
	 */
	if((real32_T)SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_c + 2.0F <
	        4.2949673E+9F) {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_c = (uint32_T)((real32_T)
		        SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_c + 2.0F);
	} else {
		SensorProcessing_DW.DiscreteTimeIntegrator_DSTATE_c = MAX_uint32_T;
	}

	/* End of Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
}

/* Model initialize function */
void SensorProcessing_initialize(const char_T** rt_errorStatus, const
                                 rtTimingBridge* timingBridge, int_T mdlref_TID0, int_T mdlref_TID1)
{
	RT_MODEL_SensorProcessing_T* const SensorProcessing_M =
	    &(SensorProcessing_MdlrefDW.rtm);

	/* Registration code */

	/* initialize non-finites */
	rt_InitInfAndNaN(sizeof(real_T));

	/* setup the global timing engine */
	SensorProcessing_GlobalTID[0] = mdlref_TID0;
	SensorProcessing_GlobalTID[1] = mdlref_TID1;
	SensorProcessing_TimingBrdg = timingBridge;

	/* initialize error status */
	rtmSetErrorStatusPointer(SensorProcessing_M, rt_errorStatus);

	/* block I/O */
	(void) memset(((void*) &SensorProcessing_B), 0,
	              sizeof(B_SensorProcessing_c_T));

	/* states (dwork) */
	(void) memset((void*)&SensorProcessing_DW, 0,
	              sizeof(DW_SensorProcessing_f_T));
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
