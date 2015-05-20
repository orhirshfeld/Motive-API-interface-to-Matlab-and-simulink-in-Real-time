/* Include files */

#include <stddef.h>
#include "blas.h"
#include "Optitrack_Simulink_TrackableLocation_sfun.h"
#include "c2_Optitrack_Simulink_TrackableLocation.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Optitrack_Simulink_TrackableLocation_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[22] = { "X", "Y", "Z", "yaw", "pitch",
  "roll", "qx", "qy", "qz", "qw", "TimeStamp1", "nargin", "nargout",
  "Trackable_index", "x", "y", "z", "yaw1", "pitch1", "roll1", "TimeStamp",
  "DeltaTime" };

/* Function Declarations */
static void initialize_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void initialize_params_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void enable_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void disable_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void set_sim_state_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_st);
static void finalize_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void sf_gateway_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void c2_chartstep_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void initSimStructsc2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static real32_T c2_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_X, const char_T *c2_identifier);
static real32_T c2_b_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static real_T c2_c_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_calllib, const char_T *c2_identifier);
static real_T c2_d_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_e_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_f_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_Optitrack_Simulink_TrackableLocation, const char_T
   *c2_identifier);
static uint8_T c2_g_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static real_T c2_get_old_TimeStamp
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance,
   uint32_T c2_b);
static void c2_set_old_TimeStamp
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance,
   uint32_T c2_b, real_T c2_c);
static real_T *c2_access_old_TimeStamp
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance,
   uint32_T c2_b);
static void init_dsm_address_info
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_Optitrack_Simulink_TrackableLocation = 0U;
}

static void initialize_params_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_u;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  real_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T c2_d_hoistedGlobal;
  real_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  real_T c2_e_hoistedGlobal;
  real_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real_T c2_f_hoistedGlobal;
  real_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  real_T c2_g_hoistedGlobal;
  real_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real_T c2_h_hoistedGlobal;
  real_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  uint8_T c2_i_hoistedGlobal;
  uint8_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  real_T *c2_DeltaTime;
  real_T *c2_TimeStamp;
  real_T *c2_pitch1;
  real_T *c2_roll1;
  real_T *c2_x;
  real_T *c2_k_y;
  real_T *c2_yaw1;
  real_T *c2_z;
  c2_DeltaTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_TimeStamp = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_roll1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_pitch1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_yaw1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_z = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_k_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(9, 1), false);
  c2_hoistedGlobal = *c2_DeltaTime;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_TimeStamp;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *c2_pitch1;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = *c2_roll1;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_hoistedGlobal = *c2_x;
  c2_e_u = c2_e_hoistedGlobal;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_hoistedGlobal = *c2_k_y;
  c2_f_u = c2_f_hoistedGlobal;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_g_hoistedGlobal = *c2_yaw1;
  c2_g_u = c2_g_hoistedGlobal;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_h_hoistedGlobal = *c2_z;
  c2_h_u = c2_h_hoistedGlobal;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_i_hoistedGlobal =
    chartInstance->c2_is_active_c2_Optitrack_Simulink_TrackableLocation;
  c2_i_u = c2_i_hoistedGlobal;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 8, c2_j_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T *c2_DeltaTime;
  real_T *c2_TimeStamp;
  real_T *c2_pitch1;
  real_T *c2_roll1;
  real_T *c2_x;
  real_T *c2_y;
  real_T *c2_yaw1;
  real_T *c2_z;
  c2_DeltaTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_TimeStamp = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_roll1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_pitch1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_yaw1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_z = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *c2_DeltaTime = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 0)), "DeltaTime");
  *c2_TimeStamp = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 1)), "TimeStamp");
  *c2_pitch1 = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 2)), "pitch1");
  *c2_roll1 = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 3)), "roll1");
  *c2_x = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 4)),
    "x");
  *c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 5)),
    "y");
  *c2_yaw1 = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u,
    6)), "yaw1");
  *c2_z = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 7)),
    "z");
  chartInstance->c2_is_active_c2_Optitrack_Simulink_TrackableLocation =
    c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 8)),
    "is_active_c2_Optitrack_Simulink_TrackableLocation");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_Optitrack_Simulink_TrackableLocation(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  real_T *c2_Trackable_index;
  real_T *c2_x;
  real_T *c2_y;
  real_T *c2_z;
  real_T *c2_yaw1;
  real_T *c2_pitch1;
  real_T *c2_roll1;
  real_T *c2_TimeStamp;
  real_T *c2_DeltaTime;
  c2_DeltaTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_TimeStamp = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_roll1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_pitch1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_yaw1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_z = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_Trackable_index = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c2_Trackable_index, 0U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_Optitrack_Simulink_TrackableLocation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_Optitrack_Simulink_TrackableLocationMachineNumber_,
     chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*c2_x, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_y, 2U);
  _SFD_DATA_RANGE_CHECK(*c2_z, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_yaw1, 4U);
  _SFD_DATA_RANGE_CHECK(*c2_pitch1, 5U);
  _SFD_DATA_RANGE_CHECK(*c2_roll1, 6U);
  _SFD_DATA_RANGE_CHECK(*c2_TimeStamp, 7U);
  _SFD_DATA_RANGE_CHECK(*c2_DeltaTime, 8U);
}

static void c2_chartstep_c2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_Trackable_index;
  uint32_T c2_debug_family_var_map[22];
  real32_T c2_X;
  real32_T c2_Y;
  real32_T c2_Z;
  real32_T c2_yaw;
  real32_T c2_pitch;
  real32_T c2_roll;
  real32_T c2_qx;
  real32_T c2_qy;
  real32_T c2_qz;
  real32_T c2_qw;
  real_T c2_TimeStamp1;
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 8.0;
  real_T c2_x;
  real_T c2_y;
  real_T c2_z;
  real_T c2_yaw1;
  real_T c2_pitch1;
  real_T c2_roll1;
  real_T c2_TimeStamp;
  real_T c2_DeltaTime;
  int32_T c2_i0;
  static char_T c2_cv0[15] = { 'N', 'P', 'T', 'r', 'a', 'c', 'k', 'i', 'n', 'g',
    'T', 'o', 'o', 'l', 's' };

  char_T c2_u[15];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i1;
  static char_T c2_cv1[20] = { 'T', 'T', '_', 'U', 'p', 'd', 'a', 't', 'e', 'S',
    'i', 'n', 'g', 'l', 'e', 'F', 'r', 'a', 'm', 'e' };

  char_T c2_b_u[20];
  const mxArray *c2_c_y = NULL;
  int32_T c2_i2;
  char_T c2_c_u[15];
  const mxArray *c2_d_y = NULL;
  int32_T c2_i3;
  static char_T c2_cv2[20] = { 'T', 'T', '_', 'T', 'r', 'a', 'c', 'k', 'a', 'b',
    'l', 'e', 'L', 'o', 'c', 'a', 't', 'i', 'o', 'n' };

  char_T c2_d_u[20];
  const mxArray *c2_e_y = NULL;
  real_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real32_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  real32_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real32_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real32_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  real32_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  real32_T c2_k_u;
  const mxArray *c2_l_y = NULL;
  real32_T c2_l_u;
  const mxArray *c2_m_y = NULL;
  real32_T c2_m_u;
  const mxArray *c2_n_y = NULL;
  real32_T c2_n_u;
  const mxArray *c2_o_y = NULL;
  real32_T c2_o_u;
  const mxArray *c2_p_y = NULL;
  const mxArray *c2_b_roll = NULL;
  const mxArray *c2_b_pitch = NULL;
  const mxArray *c2_b_yaw = NULL;
  const mxArray *c2_b_qw = NULL;
  const mxArray *c2_b_qz = NULL;
  const mxArray *c2_b_qy = NULL;
  const mxArray *c2_b_qx = NULL;
  const mxArray *c2_b_Z = NULL;
  const mxArray *c2_b_Y = NULL;
  const mxArray *c2_b_X = NULL;
  int32_T c2_i4;
  char_T c2_p_u[15];
  const mxArray *c2_q_y = NULL;
  int32_T c2_i5;
  static char_T c2_cv3[17] = { 'T', 'T', '_', 'F', 'r', 'a', 'm', 'e', 'T', 'i',
    'm', 'e', 'S', 't', 'a', 'm', 'p' };

  char_T c2_q_u[17];
  const mxArray *c2_r_y = NULL;
  real_T *c2_b_Trackable_index;
  real_T *c2_b_x;
  real_T *c2_s_y;
  real_T *c2_b_z;
  real_T *c2_b_yaw1;
  real_T *c2_b_pitch1;
  real_T *c2_b_roll1;
  real_T *c2_b_TimeStamp;
  real_T *c2_b_DeltaTime;
  c2_b_DeltaTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
  c2_b_TimeStamp = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
  c2_b_roll1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
  c2_b_pitch1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_b_yaw1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_b_z = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_s_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_Trackable_index = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_Trackable_index;
  c2_Trackable_index = c2_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 22U, 22U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_X, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Y, 1U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Z, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_yaw, 3U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_pitch, 4U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_roll, 5U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_qx, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_qy, 7U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_qz, 8U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_qw, 9U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_TimeStamp1, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_Trackable_index, 13U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x, 14U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y, 15U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_z, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_yaw1, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_pitch1, 18U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_roll1, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_TimeStamp, 20U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_DeltaTime, 21U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  for (c2_i0 = 0; c2_i0 < 15; c2_i0++) {
    c2_u[c2_i0] = c2_cv0[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 15),
                false);
  for (c2_i1 = 0; c2_i1 < 20; c2_i1++) {
    c2_b_u[c2_i1] = c2_cv1[c2_i1];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 20),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "calllib", 0U, 2U, 14, c2_b_y,
                    14, c2_c_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_X = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_Y = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  c2_Z = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_yaw = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_pitch = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_roll = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_qx = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_qy = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_qz = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_qw = 0.0F;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_TimeStamp1 = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
  for (c2_i2 = 0; c2_i2 < 15; c2_i2++) {
    c2_c_u[c2_i2] = c2_cv0[c2_i2];
  }

  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", c2_c_u, 10, 0U, 1U, 0U, 2, 1, 15),
                false);
  for (c2_i3 = 0; c2_i3 < 20; c2_i3++) {
    c2_d_u[c2_i3] = c2_cv2[c2_i3];
  }

  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", c2_d_u, 10, 0U, 1U, 0U, 2, 1, 20),
                false);
  c2_e_u = c2_Trackable_index - 1.0;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 0, 0U, 0U, 0U, 0), false);
  c2_f_u = c2_X;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 1, 0U, 0U, 0U, 0), false);
  c2_g_u = c2_Y;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 1, 0U, 0U, 0U, 0), false);
  c2_h_u = c2_Z;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 1, 0U, 0U, 0U, 0), false);
  c2_i_u = c2_qx;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 1, 0U, 0U, 0U, 0), false);
  c2_j_u = c2_qy;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 1, 0U, 0U, 0U, 0), false);
  c2_k_u = c2_qz;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_k_u, 1, 0U, 0U, 0U, 0), false);
  c2_l_u = c2_qw;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_l_u, 1, 0U, 0U, 0U, 0), false);
  c2_m_u = c2_yaw;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_m_u, 1, 0U, 0U, 0U, 0), false);
  c2_n_u = c2_pitch;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_n_u, 1, 0U, 0U, 0U, 0), false);
  c2_o_u = c2_roll;
  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_o_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "calllib", 10U, 13U, 14, c2_d_y,
                    14, c2_e_y, 14, c2_f_y, 14, c2_g_y, 14, c2_h_y, 14, c2_i_y,
                    14, c2_j_y, 14, c2_k_y, 14, c2_l_y, 14, c2_m_y, 14, c2_n_y,
                    14, c2_o_y, 14, c2_p_y, &c2_b_X, &c2_b_Y, &c2_b_Z, &c2_b_qx,
                    &c2_b_qy, &c2_b_qz, &c2_b_qw, &c2_b_yaw, &c2_b_pitch,
                    &c2_b_roll);
  c2_X = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_X), "X");
  c2_Y = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Y), "Y");
  c2_Z = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_Z), "Z");
  c2_qx = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_qx), "qx");
  c2_qy = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_qy), "qy");
  c2_qz = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_qz), "qz");
  c2_qw = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_qw), "qw");
  c2_yaw = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_yaw), "yaw");
  c2_pitch = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_pitch), "pitch");
  c2_roll = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_roll), "roll");
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 18);
  for (c2_i4 = 0; c2_i4 < 15; c2_i4++) {
    c2_p_u[c2_i4] = c2_cv0[c2_i4];
  }

  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_create("y", c2_p_u, 10, 0U, 1U, 0U, 2, 1, 15),
                false);
  for (c2_i5 = 0; c2_i5 < 17; c2_i5++) {
    c2_q_u[c2_i5] = c2_cv3[c2_i5];
  }

  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", c2_q_u, 10, 0U, 1U, 0U, 2, 1, 17),
                false);
  c2_TimeStamp1 = c2_c_emlrt_marshallIn(chartInstance, sf_mex_call_debug
    (sfGlobalDebugInstanceStruct, "calllib", 1U, 2U, 14, c2_q_y, 14, c2_r_y),
    "calllib");
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_x = c2_X;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_y = c2_Y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 21);
  c2_z = c2_Z;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  c2_yaw1 = c2_yaw;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  c2_pitch1 = c2_pitch;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  c2_roll1 = c2_roll;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 23);
  c2_TimeStamp = c2_TimeStamp1;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  c2_DeltaTime = c2_TimeStamp - c2_get_old_TimeStamp(chartInstance, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
  c2_set_old_TimeStamp(chartInstance, 0, c2_TimeStamp);
  ssUpdateDataStoreLog(chartInstance->S, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -27);
  _SFD_SYMBOL_SCOPE_POP();
  sf_mex_destroy(&c2_b_X);
  sf_mex_destroy(&c2_b_Y);
  sf_mex_destroy(&c2_b_Z);
  sf_mex_destroy(&c2_b_qx);
  sf_mex_destroy(&c2_b_qy);
  sf_mex_destroy(&c2_b_qz);
  sf_mex_destroy(&c2_b_qw);
  sf_mex_destroy(&c2_b_yaw);
  sf_mex_destroy(&c2_b_pitch);
  sf_mex_destroy(&c2_b_roll);
  *c2_b_x = c2_x;
  *c2_s_y = c2_y;
  *c2_b_z = c2_z;
  *c2_b_yaw1 = c2_yaw1;
  *c2_b_pitch1 = c2_pitch1;
  *c2_b_roll1 = c2_roll1;
  *c2_b_TimeStamp = c2_TimeStamp;
  *c2_b_DeltaTime = c2_DeltaTime;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_Optitrack_Simulink_TrackableLocation
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_calllib;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    chartInstanceVoid;
  c2_calllib = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_calllib), &c2_thisId);
  sf_mex_destroy(&c2_calllib);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_X;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y;
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    chartInstanceVoid;
  c2_X = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_X), &c2_thisId);
  sf_mex_destroy(&c2_X);
  *(real32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray
  *sf_c2_Optitrack_Simulink_TrackableLocation_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c2_nameCaptureInfo;
}

static real32_T c2_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_X, const char_T *c2_identifier)
{
  real32_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_X), &c2_thisId);
  sf_mex_destroy(&c2_X);
  return c2_y;
}

static real32_T c2_b_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real32_T c2_y;
  real32_T c2_f0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_f0, 1, 1, 0U, 0, 0U, 0);
  c2_y = c2_f0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static real_T c2_c_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_calllib, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_calllib), &c2_thisId);
  sf_mex_destroy(&c2_calllib);
  return c2_y;
}

static real_T c2_d_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_e_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i6;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i6, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i6;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_f_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_b_is_active_c2_Optitrack_Simulink_TrackableLocation, const char_T
   *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_Optitrack_Simulink_TrackableLocation), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Optitrack_Simulink_TrackableLocation);
  return c2_y;
}

static uint8_T c2_g_emlrt_marshallIn
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance, const
   mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static real_T c2_get_old_TimeStamp
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance,
   uint32_T c2_b)
{
  ssReadFromDataStoreElement(chartInstance->S, 0, NULL, c2_b);
  if (chartInstance->c2_old_TimeStamp_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'old_TimeStamp\' (#39) in the initialization routine of the chart.\n");
  }

  return *chartInstance->c2_old_TimeStamp_address;
}

static void c2_set_old_TimeStamp
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance,
   uint32_T c2_b, real_T c2_c)
{
  ssWriteToDataStoreElement(chartInstance->S, 0, NULL, c2_b);
  if (chartInstance->c2_old_TimeStamp_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'old_TimeStamp\' (#39) in the initialization routine of the chart.\n");
  }

  *chartInstance->c2_old_TimeStamp_address = c2_c;
}

static real_T *c2_access_old_TimeStamp
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance,
   uint32_T c2_b)
{
  real_T *c2_c;
  ssReadFromDataStore(chartInstance->S, 0, NULL);
  if (chartInstance->c2_old_TimeStamp_address == 0) {
    sf_mex_error_message("Invalid access to Data Store Memory data \'old_TimeStamp\' (#39) in the initialization routine of the chart.\n");
  }

  c2_c = chartInstance->c2_old_TimeStamp_address;
  if (c2_b == 0) {
    ssWriteToDataStore(chartInstance->S, 0, NULL);
  }

  return c2_c;
}

static void init_dsm_address_info
  (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance)
{
  ssGetSFcnDataStoreNameAddrIdx(chartInstance->S, "old_TimeStamp", (void **)
    &chartInstance->c2_old_TimeStamp_address,
    &chartInstance->c2_old_TimeStamp_index);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_Optitrack_Simulink_TrackableLocation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3013464511U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2271892163U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(573560044U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2614298040U);
}

mxArray *sf_c2_Optitrack_Simulink_TrackableLocation_get_autoinheritance_info
  (void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("IZ45u0HmjbJIGOhx2G0BrG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,8,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_Optitrack_Simulink_TrackableLocation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_Optitrack_Simulink_TrackableLocation_updateBuildInfo_args_info
  (void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray
  *sf_get_sim_state_info_c2_Optitrack_Simulink_TrackableLocation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[13],T\"DeltaTime\",},{M[1],M[11],T\"TimeStamp\",},{M[1],M[9],T\"pitch1\",},{M[1],M[10],T\"roll1\",},{M[1],M[5],T\"x\",},{M[1],M[6],T\"y\",},{M[1],M[8],T\"yaw1\",},{M[1],M[7],T\"z\",},{M[8],M[0],T\"is_active_c2_Optitrack_Simulink_TrackableLocation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Optitrack_Simulink_TrackableLocation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Optitrack_Simulink_TrackableLocationMachineNumber_,
           2,
           1,
           1,
           0,
           10,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation
          (_Optitrack_Simulink_TrackableLocationMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _Optitrack_Simulink_TrackableLocationMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Optitrack_Simulink_TrackableLocationMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"Trackable_index");
          _SFD_SET_DATA_PROPS(1,2,0,1,"x");
          _SFD_SET_DATA_PROPS(2,2,0,1,"y");
          _SFD_SET_DATA_PROPS(3,2,0,1,"z");
          _SFD_SET_DATA_PROPS(4,2,0,1,"yaw1");
          _SFD_SET_DATA_PROPS(5,2,0,1,"pitch1");
          _SFD_SET_DATA_PROPS(6,2,0,1,"roll1");
          _SFD_SET_DATA_PROPS(7,2,0,1,"TimeStamp");
          _SFD_SET_DATA_PROPS(8,2,0,1,"DeltaTime");
          _SFD_SET_DATA_PROPS(9,11,0,0,"old_TimeStamp");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1110);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);

        {
          real_T *c2_Trackable_index;
          real_T *c2_x;
          real_T *c2_y;
          real_T *c2_z;
          real_T *c2_yaw1;
          real_T *c2_pitch1;
          real_T *c2_roll1;
          real_T *c2_TimeStamp;
          real_T *c2_DeltaTime;
          c2_DeltaTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 8);
          c2_TimeStamp = (real_T *)ssGetOutputPortSignal(chartInstance->S, 7);
          c2_roll1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 6);
          c2_pitch1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c2_yaw1 = (real_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c2_z = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c2_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c2_Trackable_index = (real_T *)ssGetInputPortSignal(chartInstance->S,
            0);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_Trackable_index);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_x);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_y);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_z);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_yaw1);
          _SFD_SET_DATA_VALUE_PTR(5U, c2_pitch1);
          _SFD_SET_DATA_VALUE_PTR(6U, c2_roll1);
          _SFD_SET_DATA_VALUE_PTR(7U, c2_TimeStamp);
          _SFD_SET_DATA_VALUE_PTR(8U, c2_DeltaTime);
          _SFD_SET_DATA_VALUE_PTR(9U, chartInstance->c2_old_TimeStamp_address);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Optitrack_Simulink_TrackableLocationMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "XkE8tzkHIRLQv1Rqag1TaG";
}

static void sf_opaque_initialize_c2_Optitrack_Simulink_TrackableLocation(void
  *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*)
      chartInstanceVar)->S,0);
  initialize_params_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*) chartInstanceVar);
  initialize_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_Optitrack_Simulink_TrackableLocation(void
  *chartInstanceVar)
{
  enable_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_Optitrack_Simulink_TrackableLocation(void
  *chartInstanceVar)
{
  disable_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_Optitrack_Simulink_TrackableLocation(void
  *chartInstanceVar)
{
  sf_gateway_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*) chartInstanceVar);
}

extern const mxArray*
  sf_internal_get_sim_state_c2_Optitrack_Simulink_TrackableLocation(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*)
     chartInfo->chartInstance);        /* raw sim ctx */
  prhs[3] = (mxArray*)
    sf_get_sim_state_info_c2_Optitrack_Simulink_TrackableLocation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_Optitrack_Simulink_TrackableLocation
  (SimStruct* S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*)
    sf_get_sim_state_info_c2_Optitrack_Simulink_TrackableLocation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*)
     chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray*
  sf_opaque_get_sim_state_c2_Optitrack_Simulink_TrackableLocation(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_Optitrack_Simulink_TrackableLocation(S);
}

static void sf_opaque_set_sim_state_c2_Optitrack_Simulink_TrackableLocation
  (SimStruct* S, const mxArray *st)
{
  sf_internal_set_sim_state_c2_Optitrack_Simulink_TrackableLocation(S, st);
}

static void sf_opaque_terminate_c2_Optitrack_Simulink_TrackableLocation(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Optitrack_Simulink_TrackableLocation_optimization_info();
    }

    finalize_c2_Optitrack_Simulink_TrackableLocation
      ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*)
       chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Optitrack_Simulink_TrackableLocation
    ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Optitrack_Simulink_TrackableLocation
  (SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c2_Optitrack_Simulink_TrackableLocation
      ((SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_Optitrack_Simulink_TrackableLocation(SimStruct
  *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct =
      load_Optitrack_Simulink_TrackableLocation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,8);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=8; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4071483844U));
  ssSetChecksum1(S,(2896496511U));
  ssSetChecksum2(S,(3515733478U));
  ssSetChecksum3(S,(2915397822U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,0);
}

static void mdlRTW_c2_Optitrack_Simulink_TrackableLocation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Optitrack_Simulink_TrackableLocation(SimStruct *S)
{
  SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct *)
    utMalloc(sizeof(SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc2_Optitrack_Simulink_TrackableLocationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_Optitrack_Simulink_TrackableLocation;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_Optitrack_Simulink_TrackableLocation_method_dispatcher(SimStruct *S,
  int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Optitrack_Simulink_TrackableLocation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Optitrack_Simulink_TrackableLocation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Optitrack_Simulink_TrackableLocation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Optitrack_Simulink_TrackableLocation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
