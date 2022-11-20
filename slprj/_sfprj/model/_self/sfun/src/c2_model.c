/* Include files */

#include "model_sfun.h"
#include "c2_model.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(S);
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

/* Forward Declarations */

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static emlrtRSInfo c2_emlrtRSI = { 6,  /* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#model:1"                           /* pathName */
};

static emlrtRSInfo c2_b_emlrtRSI = { 7,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#model:1"                           /* pathName */
};

static emlrtRSInfo c2_c_emlrtRSI = { 8,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#model:1"                           /* pathName */
};

static emlrtRSInfo c2_d_emlrtRSI = { 11,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#model:1"                           /* pathName */
};

static emlrtRSInfo c2_e_emlrtRSI = { 12,/* lineNo */
  "MATLAB Function",                   /* fcnName */
  "#model:1"                           /* pathName */
};

static emlrtRSInfo c2_f_emlrtRSI = { 26,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_g_emlrtRSI = { 31,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_h_emlrtRSI = { 32,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_i_emlrtRSI = { 33,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_j_emlrtRSI = { 35,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_k_emlrtRSI = { 49,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_l_emlrtRSI = { 64,/* lineNo */
  "arduino",                           /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/arduinoio/+coder/arduino."
  "m"                                  /* pathName */
};

static emlrtRSInfo c2_m_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+hwsdk/"
  "controller.m"                       /* pathName */
};

static emlrtRSInfo c2_n_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+dio/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_o_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+adc/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_p_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+pwm/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_q_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+i2c/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_r_emlrtRSI = { 1,/* lineNo */
  "controller_base",                   /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+i2c/co"
  "ntroller_base.m"                    /* pathName */
};

static emlrtRSInfo c2_s_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+spi/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_t_emlrtRSI = { 1,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+serial"
  "/controller.m"                      /* pathName */
};

static emlrtRSInfo c2_u_emlrtRSI = { 1,/* lineNo */
  "SensorCodegenUtilities",            /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/sensors/+matlabshared/+sensors/+co"
  "der/+matlab/SensorCodegenUtilities.m"/* pathName */
};

static emlrtRSInfo c2_v_emlrtRSI = { 1,/* lineNo */
  "ArduinoDigitalIO",                  /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/sharedarduino/+arduinodri"
  "ver/ArduinoDigitalIO.p"             /* pathName */
};

static emlrtRSInfo c2_w_emlrtRSI = { 1,/* lineNo */
  "DigitalIO",                         /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/shared/devicedrivers_coder/+matlabshared/+devicedr"
  "ivers/+coder/DigitalIO.p"           /* pathName */
};

static emlrtRSInfo c2_x_emlrtRSI = { 1,/* lineNo */
  "ArduinoAnalogInput",                /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/sharedarduino/+arduinodri"
  "ver/ArduinoAnalogInput.p"           /* pathName */
};

static emlrtRSInfo c2_y_emlrtRSI = { 1,/* lineNo */
  "AnalogInSingle",                    /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/shared/devicedrivers_coder/+matlabshared/+devicedr"
  "ivers/+coder/AnalogInSingle.p"      /* pathName */
};

static emlrtRSInfo c2_ab_emlrtRSI = { 1,/* lineNo */
  "PWM",                               /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/shared/devicedrivers_coder/+matlabshared/+devicedr"
  "ivers/+coder/PWM.p"                 /* pathName */
};

static emlrtRSInfo c2_bb_emlrtRSI = { 1,/* lineNo */
  "ArduinoSPI",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/sharedarduino/+arduinodri"
  "ver/ArduinoSPI.p"                   /* pathName */
};

static emlrtRSInfo c2_cb_emlrtRSI = { 1,/* lineNo */
  "SPI",                               /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/shared/devicedrivers_coder/+matlabshared/+devicedr"
  "ivers/+coder/SPI.p"                 /* pathName */
};

static emlrtRSInfo c2_db_emlrtRSI = { 1,/* lineNo */
  "ArduinoPinMap",                     /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/supportpackages/arduinobase/+codertarget/+arduinob"
  "ase/+internal/ArduinoPinMap.p"      /* pathName */
};

static emlrtRSInfo c2_eb_emlrtRSI = { 1,/* lineNo */
  "Due",                               /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/supportpackages/arduinobase/+codertarget/+arduinob"
  "ase/+internal/+pinMaps/Due.p"       /* pathName */
};

static emlrtRSInfo c2_fb_emlrtRSI = { 1,/* lineNo */
  "ArduinoI2C",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/supportpackages/sharedarduino/+arduinodri"
  "ver/ArduinoI2C.p"                   /* pathName */
};

static emlrtRSInfo c2_gb_emlrtRSI = { 1,/* lineNo */
  "I2C",                               /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/shared/devicedrivers_coder/+matlabshared/+devicedr"
  "ivers/+coder/I2C.p"                 /* pathName */
};

static emlrtRSInfo c2_hb_emlrtRSI = { 31,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+hwsdk/"
  "controller.m"                       /* pathName */
};

static emlrtRSInfo c2_ib_emlrtRSI = { 74,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+hwsdk/"
  "controller.m"                       /* pathName */
};

static emlrtRSInfo c2_jb_emlrtRSI = { 69,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+hwsdk/"
  "controller.m"                       /* pathName */
};

static emlrtRSInfo c2_kb_emlrtRSI = { 28,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+dio/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_lb_emlrtRSI = { 38,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+dio/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_mb_emlrtRSI = { 19,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+dio/co"
  "ntroller.m"                         /* pathName */
};

static emlrtRSInfo c2_nb_emlrtRSI = { 34,/* lineNo */
  "controller",                        /* fcnName */
  "/Users/kojirotanakamba/Documents/MATLAB/SupportPackages/R2022a/toolbox/matlab/hardware/shared/hwsdk/+matlabshared/+coder/+dio/co"
  "ntroller.m"                         /* pathName */
};

/* Function Declarations */
static void initialize_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void initialize_params_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void mdl_start_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void mdl_terminate_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void mdl_setup_runtime_resources_c2_model(SFc2_modelInstanceStruct
  *chartInstance);
static void mdl_cleanup_runtime_resources_c2_model(SFc2_modelInstanceStruct
  *chartInstance);
static void enable_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void disable_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void sf_gateway_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void ext_mode_exec_c2_model(SFc2_modelInstanceStruct *chartInstance);
static void c2_update_jit_animation_c2_model(SFc2_modelInstanceStruct
  *chartInstance);
static void c2_do_animation_call_c2_model(SFc2_modelInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_model(SFc2_modelInstanceStruct
  *chartInstance);
static void set_sim_state_c2_model(SFc2_modelInstanceStruct *chartInstance,
  const mxArray *c2_st);
static void initSimStructsc2_model(SFc2_modelInstanceStruct *chartInstance);
static uint8_T c2_emlrt_marshallIn(SFc2_modelInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_model, const char_T *c2_identifier);
static uint8_T c2_b_emlrt_marshallIn(SFc2_modelInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_slStringInitializeDynamicBuffers(SFc2_modelInstanceStruct
  *chartInstance);
static const mxArray *c2_chart_data_browse_helper(SFc2_modelInstanceStruct
  *chartInstance);
static void init_dsm_address_info(SFc2_modelInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc2_modelInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_arduinoObj_not_empty = false;
  chartInstance->c2_is_active_c2_model = 0U;
}

static void initialize_params_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_start_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_terminate_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void mdl_setup_runtime_resources_c2_model(SFc2_modelInstanceStruct
  *chartInstance)
{
  static const uint32_T c2_decisionTxtEndIdx = 0U;
  static const uint32_T c2_decisionTxtStartIdx = 0U;
  setDebuggerFlag(chartInstance->S, true);
  setDataBrowseFcn(chartInstance->S, (void *)&c2_chart_data_browse_helper);
  chartInstance->c2_RuntimeVar = sfListenerCacheSimStruct(chartInstance->S);
  sfListenerInitializeRuntimeVars(chartInstance->c2_RuntimeVar,
    &chartInstance->c2_IsDebuggerActive,
    &chartInstance->c2_IsSequenceViewerPresent, 0, 0,
    &chartInstance->c2_mlFcnLineNumber, &chartInstance->c2_IsHeatMapPresent, 0);
  sim_mode_is_external(chartInstance->S);
  covrtCreateStateflowInstanceData(chartInstance->c2_covrtInstance, 1U, 0U, 1U,
    0U);
  covrtChartInitFcn(chartInstance->c2_covrtInstance, 0U, false, false, false);
  covrtStateInitFcn(chartInstance->c2_covrtInstance, 0U, 0U, false, false, false,
                    0U, &c2_decisionTxtStartIdx, &c2_decisionTxtEndIdx);
  covrtTransInitFcn(chartInstance->c2_covrtInstance, 0U, 0, NULL, NULL, 0U, NULL);
  covrtEmlInitFcn(chartInstance->c2_covrtInstance, "", 4U, 0U, 1U, 0U, 1U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  covrtEmlFcnInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U,
                     "eML_blk_kernel", 0, -1, 312);
  covrtEmlIfInitFcn(chartInstance->c2_covrtInstance, 4U, 0U, 0U, 53, 76, -1, 229,
                    false);
}

static void mdl_cleanup_runtime_resources_c2_model(SFc2_modelInstanceStruct
  *chartInstance)
{
  sfListenerLightTerminate(chartInstance->c2_RuntimeVar);
  covrtDeleteStateflowInstanceData(chartInstance->c2_covrtInstance);
}

static void enable_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void sf_gateway_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  c2_arduino *c2_b_obj;
  c2_arduino *c2_b_this;
  c2_arduino *c2_c_obj;
  c2_arduino *c2_c_this;
  c2_arduino *c2_d_obj;
  c2_arduino *c2_d_this;
  c2_arduino *c2_e_obj;
  c2_arduino *c2_e_this;
  c2_arduino *c2_f_obj;
  c2_arduino *c2_f_this;
  c2_arduino *c2_g_obj;
  c2_arduino *c2_g_this;
  c2_arduino *c2_h_obj;
  c2_arduino *c2_h_this;
  c2_arduino *c2_i_obj;
  c2_arduino *c2_j_obj;
  c2_arduino *c2_k_obj;
  c2_arduino *c2_obj;
  c2_arduino *c2_this;
  c2_arduinodriver_ArduinoI2C *c2_rv[2];
  c2_arduinodriver_ArduinoI2C *c2_i_this;
  c2_arduinodriver_ArduinoI2C *c2_j_this;
  c2_arduinodriver_ArduinoI2C *c2_l_obj;
  c2_arduinodriver_ArduinoI2C *c2_m_obj;
  c2_arduinodriver_ArduinoI2C *c2_n_obj;
  c2_arduinodriver_ArduinoI2C *c2_o_obj;
  c2_arduinodriver_ArduinoI2C *c2_r;
  c2_arduinodriver_ArduinoI2C *c2_r1;
  int32_T c2_i;
  chartInstance->c2_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_sfEvent = CALL_EVENT;
  covrtEmlFcnEval(chartInstance->c2_covrtInstance, 4U, 0, 0);
  if (covrtEmlIfEval(chartInstance->c2_covrtInstance, 4U, 0, 0,
                     !chartInstance->c2_arduinoObj_not_empty)) {
    c2_obj = &chartInstance->c2_arduinoObj;
    c2_b_obj = c2_obj;
    c2_c_obj = c2_b_obj;
    c2_b_obj = c2_c_obj;
    c2_this = c2_b_obj;
    c2_b_obj = c2_this;
    c2_d_obj = c2_b_obj;
    c2_b_obj = c2_d_obj;
    c2_b_this = c2_b_obj;
    c2_b_obj = c2_b_this;
    c2_e_obj = c2_b_obj;
    c2_b_obj = c2_e_obj;
    c2_c_this = c2_b_obj;
    c2_b_obj = c2_c_this;
    c2_f_obj = c2_b_obj;
    c2_b_obj = c2_f_obj;
    c2_d_this = c2_b_obj;
    c2_b_obj = c2_d_this;
    c2_g_obj = c2_b_obj;
    c2_b_obj = c2_g_obj;
    c2_h_obj = c2_b_obj;
    c2_b_obj = c2_h_obj;
    c2_e_this = c2_b_obj;
    c2_b_obj = c2_e_this;
    c2_i_obj = c2_b_obj;
    c2_b_obj = c2_i_obj;
    c2_f_this = c2_b_obj;
    c2_b_obj = c2_f_this;
    c2_j_obj = c2_b_obj;
    c2_b_obj = c2_j_obj;
    c2_g_this = c2_b_obj;
    c2_b_obj = c2_g_this;
    c2_k_obj = c2_b_obj;
    c2_b_obj = c2_k_obj;
    c2_h_this = c2_b_obj;
    c2_b_obj = c2_h_this;
    c2_b_obj->Aref = 3.3;
    c2_l_obj = &c2_b_obj->_pobj0[0];
    c2_r = c2_l_obj;
    c2_m_obj = c2_r;
    c2_r = c2_m_obj;
    c2_i_this = c2_r;
    c2_r = c2_i_this;
    c2_r->MW_I2C_HANDLE = NULL;
    c2_n_obj = &c2_b_obj->_pobj0[1];
    c2_r1 = c2_n_obj;
    c2_o_obj = c2_r1;
    c2_r1 = c2_o_obj;
    c2_j_this = c2_r1;
    c2_r1 = c2_j_this;
    c2_r1->MW_I2C_HANDLE = NULL;
    c2_rv[0] = c2_r;
    c2_rv[1] = c2_r1;
    for (c2_i = 0; c2_i < 2; c2_i++) {
      c2_b_obj->I2CDriverObj[c2_i] = c2_rv[c2_i];
    }

    chartInstance->c2_arduinoObj_not_empty = true;
    digitalIOSetup(1, 1);
    digitalIOSetup(2, 0);
  }

  writeDigitalPin(1, 1);
  readDigitalPin(2);
  c2_do_animation_call_c2_model(chartInstance);
}

static void ext_mode_exec_c2_model(SFc2_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c2_update_jit_animation_c2_model(SFc2_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c2_do_animation_call_c2_model(SFc2_modelInstanceStruct
  *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static const mxArray *get_sim_state_c2_model(SFc2_modelInstanceStruct
  *chartInstance)
{
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_st = NULL;
  const mxArray *c2_y = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(1, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y",
    &chartInstance->c2_is_active_c2_model, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_model(SFc2_modelInstanceStruct *chartInstance,
  const mxArray *c2_st)
{
  const mxArray *c2_u;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  chartInstance->c2_is_active_c2_model = c2_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 0)), "is_active_c2_model");
  sf_mex_destroy(&c2_u);
  sf_mex_destroy(&c2_st);
}

static void initSimStructsc2_model(SFc2_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

const mxArray *sf_c2_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  const char_T *c2_data[25] = {
    "789ced5dcb6feb581d4ee10edc05335381e05e048c0666788842d2a64d7b2b21a13c9a266d92e6e6d974b873edc44ee2c6b113c749d32e467783c402663abc34"
    "42081860015b40620742b3603be22f188d58b0419a250b84c8c3278d7d737a32b173ec1c9fb371dd9fcff97ebf739cef3b2fdb9eb57872cde3f13ce719a777ff",
    "3d3e3eab9daf6bc78f78f4c9685fd38e5f339c83f48ce78e2e1fb0bfa11d2bb2a4f27d757c22b14d7e9293939b82c44a6aeeb2c57b14be238b3d9e1b59aa82c8"
    "e784269f9d3e490dcf9ad129d3e464681afe1daef39546b6dbf428f5ce8d87e2f4c9a43efe0589f7ce9cf5f15d487dac1becafc4c38f7cf90eaf747c0df95c50",
    "649595d806db2cb3be885ce9367949edf892c15c2218f265bbad96aca869b6d2606b7cc797f16ffafdac4f9565b12cf77d4d5615d9b2afce2adc05abf0becef8"
    "f216b87cf0ffae20c982ecdba8c81caf807f789b53717fcf64dc2f21e206f6b1b39d81b33ce71db9e3ad5f74b88677784328b228f2cae83ae017b3a05fc604f3",
    "0b2480f7c18278a0fc6b041eb02fb3fd4795eb1b55ab6f63bac2c10db031b6dd54b9b789aaefe7e78cdf78bcb9feeee8f8e6bd2fb0c3232e3c90dc82d7879437"
    "effdfb1908debac1deeca9a9aa9aeb07c5683db05b89706aba1d8ddcf89146e0a0fcf040ce71954f7560b1b8bf88881bd867e80027c84615585d1d781d8107ec",
    "f6eac0a0ca752a808fb79e501d582a1e2e1d78284b39556c8bfdf6de96bf9fc871c956a77d407580eac0ed71dfa2032c57a13a80570706556e930ec85407968a"
    "874b0702a7db47655e49259347f54cafbe590a87738518d501aa03b7c77d8b0eb42e9a5407f0eac0a0ca6dd28167ee531d58261e361dd86b9f8b829228f1d16e",
    "f4a055de8a166a2982c603eee001c10feb0f9ad583af20e207f6197a30706a4a0f1e97d90e6f9f1efc67413c50fe8f1078c0eea4fb6054e5a3de013e1efb131d"
    "1f2c150f972eb4cf76b252fc2a717e9c893ce86de78e7267d5ca2139ba40c7078bc56d627ca0d703ed3a3a3eb0a75fc040e2b38ab7dea63ab0543cb33af06908",
    "debac17eaee4f896ba7f904b052bbde04124e98f46040fd501aa03b7c77d8b0e745a02d501bc3a30a8729b74204de789968a876b3c5062f3398e3dcb1fedb772"
    "57d29ea48a013e43c703aed78197117103fb2c1de01581150d52b0aaf3436f22f080dd661d1855b9410a74f5c040e2b48abfa2540f968a876d1f119fe8d70fda",
    "45a512ef874eb3c95a24900d87a91eacba1ebc6532ee0788b8815da7071d5eeac84a47d385b1c99b1dfd333cf8578d97f2aa200aaac077ecd287bb6b8be181f2"
    "7f87c003f6e5eb8356d9468598fc5b930a2df7ec56c0f7fcc1fb9137a85e2c110fdbf30742ff62375bc82be162397f7ab623f2c27196a0e70f1848fe65f1516b",
    "41bc35c3ff6178c0ae6df11c2b85c7be78ddaac70c242ecaafab81878b5fb381237fbbb7c9fab9ead5beca9d578f73723844f9d598e6e59b45f140f91f87e28d"
    "2d8051ed8aefbd05f140f9af21f080dd163e9d22525cbf7bb5f8b1f2f0e8165ec38d878b474faed8662cd3c8372adb5234133be973835b8c201e058645ebf167",
    "90f2d70d768cbffbf10856fbcdfb36b43f3845e80dfa54c1f15944a8092a2bc64fbc2d7d7d5c43e29db73ebe8ea80f60d7cd77707c4fa8f0630fc1acc7c443bd"
    "7fcc82fe19d3bcba60f6fef839020fd82dbc3f5456a9f12a98ded0d5ed63dd640698ead05d023adf863b0417cf3d7e87ecfef7a37f7ef4314e3c9070eb462c9b",
    "d96c258e83dba174a914cfe5847ea017a2eba32b3ffe7e6232ee17107103bb4eb5bc46d55a5d3d20adbfc040e2b58abfde79704db41efcef6fdfb4353e5c7ac0"
    "f6c2b9a3ee5659e47782f9ed6ec6cf173bc528397ab082fd442b782128b1a25c8b4badaeaaef279a1d47f810f501ecc87104f0302b483591b74d37ccae97be8d",
    "c00376c78d23f4f50fe4031bdfbd4af878e2db07f6c6874b3fc48e3fbeb797e8f175e9f4544a463379365a21483fe87862b1b85f44c40dec33c71353eab5aae3"
    "09d2fa0d0c245e3a9e980feffd9ffce3114e3c9070eb817a14524f93977bd2febeb87bd0ee6d9e8b6c80a0fd336ed5035bf6dfcf1a37a48b498fc7be7926b3fb",
    "ef7f8ac00376c78d1706f53e54045cfcc5103e3e78a17e77a5d71be67d2e37dd0c241b8781aaa416d30f9bd9fa45aa2aed78a81eb85d0fbe84881bd8f5e303ed"
    "6cc045418e4b086585552ec7d7adea38e117083c6077ca3861560be0d405d2c7097f7ff7bfafe2c40309b72e6c1d4a67d19dc4f156743358528ae9cba0742079",
    "c8d105b3fdc41f43ca5f37d89dc20bdafc41361dd7cf1b3866dc30f06cda2f6641bf8c898e1b10e306ed8ea0e3066bf0bef3971fb8621ee9f0f8a01ade0d4891"
    "523aca72d5a4b2e74f64e83cd2ca8f1bccde3ff7117103fbcc75850117ad2aff93d21f602071d2f1c17c7887bf7dd9d6f870f17ffd48ce97435d492d2643a7c5",
    "837e34502866e87b1b569effaf4dc6ed45c40dec230fc63d58306b347acbb020a9bc22b1229084b42025d939faa7f3fa77c7707ee3dfd8521954dff048c0fea3"
    "d9cd3fac65ed06d02edcd0594003f8740d807dffd1bd6f91ad1376e3e1d289783b972d674e15a5150b5d3e8c878bd507e92e41df87a1fc309b1fcceac83710f5",
    "02ec481d698dfceb78235d7eda3f6641ff8c09d73ac4af1078c0eebcfb64436b01dfa005bc2d5c7cf712d58fa5e2e1d28fae5c3a3f393aac558474215660d940"
    "217b7648d077655c3aef10f7871dba0e31f06cda2f6641bf8c89ae4320d621b43b82ae435883e796e71bb6778eafaef289a3b3b3ca45e33cca96d58c7a489f6f",
    "58f979285bd721065cb4aafc4f4a7f8081c449d721e6c38b7c75df15fb94c4b3b8d48f574a6939b7992986cb7eb992b9f290c3ff0c24bf53df07fa09041eb057"
    "64a92ad4ba0a9f16242bf19f359c1bf1811df63e52b3f334f3ef173db1f305fe63dbf4fbfbd7f42d422a2fba0d0f573f3c7f962aec1e6623e7a9583b7f5ebb4c",
    "b3db6992bef3c840f23b958761f1ad1b8ed3bffa982c379cc2c366d7557e8dc0077667f3f0b045c67eba85af48c7a37c6c4df90c24ffb2f8f8c98278a0fccf23"
    "f080bdc78a02c7aabcf6e2b00107a4bacd32af002620859f7f83c007f6a5f2f3dc13536ba866710d7f918e47f9d99af219487eb7f0f3b5497f96bd8ff24593fc",
    "fd57847fc06e257f2f67e3d2eddcee16de231d8ff2ba35e5d3fd8eb3f73b9addbff26544bd00fb8cefdf4ea85efbba5e3ce2b14f87cdae5ffe108107ecafc453"
    "364edb4c6e8a4995830fe0e2e237f5b5efdbfa9c29e978d8decf59afd4cb49b1122b641a6d31f320990e6d75e9fe954979abba7f85eac138513d1827061227d5",
    "83d5c0a37a604df91f40f2cf5b8fd786738fe13a605ffef7af3fdcb21ed58571a2baa0af07061227d585d5c0a3ba604df96ed705b3eb05f33e2f7bcbf34f137d"
    "c81622b9cb16df195ebfaafaf016020fd8f13c078578026a2213a0e6f13d27fb1ee1fbe0edc6c3a50ffdddcded68a2948ab595bd879c785ae042b12641ef6363",
    "20f99dba9efc39041eb04f76fbddac3ec63536b0d41fabbf1f79dfe43ad09f11fe003bcefd3f1ff2838e6bb7b59d5bf88d743cba6e6c4df90c24bf53f7cf3f6f"
    "3837e201fb85224cef1c71ccfe4c8f497efe25021fd86dde3fcf09b27ef73cac3d1848bca4f014e9789487ad299f81e4776a3ffab3083c60376ce23be9aaadae",
    "1a6155d6e08fddbcbc6e92977f8fc00776a7f1f2bcedc340e22785c748c7a33c6d4df966e7c35f8794bf6eb0db3b1f6ea409ba7f0624ba4eaaaf070612275d27"
    "5d0d3cba4e6a4df90c24ffb2f8c76cbb7d0a8107ec8671fb53cf55dadd6f27e27957f3f329f47957c2f0687fdd9af219487ea7f2f23d041eb01b7eff4fad6f3d",
    "31e987d3d625ff88f007d89dbc2e096bb361720baf918e4779db9af219487ea7ae4b3e673837e201bbc2b3dcd4329863fad11e93fcbc22ef577caa1f0d6b0f06"
    "122f293c453a1ee5616bca6720f99dda7ffe24020fd8f5bffba7dfe767371fbbe43d8b083ea6ef59240d8ff2b235e53390fc4ee5e579db4dfffbbf1922933aaf",
    "f107843fc0eee4790d589b0d935b788d743ccadbe6caff3f6a87c97c", "" };

  c2_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c2_data[0], 60904U, &c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static uint8_T c2_emlrt_marshallIn(SFc2_modelInstanceStruct *chartInstance,
  const mxArray *c2_b_is_active_c2_model, const char_T *c2_identifier)
{
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y;
  c2_thisId.fIdentifier = (const char_T *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_is_active_c2_model),
    &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_model);
  return c2_y;
}

static uint8_T c2_b_emlrt_marshallIn(SFc2_modelInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_b_u;
  uint8_T c2_y;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b_u, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_b_u;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_slStringInitializeDynamicBuffers(SFc2_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c2_chart_data_browse_helper(SFc2_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
  return NULL;
}

static void init_dsm_address_info(SFc2_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_modelInstanceStruct *chartInstance)
{
  chartInstance->c2_covrtInstance = (CovrtStateflowInstance *)
    sfrtGetCovrtInstance(chartInstance->S);
  chartInstance->c2_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c2_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(363714329U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2087289523U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1079368181U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1575494981U);
}

mxArray *sf_c2_model_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,3);
  mxSetCell(mxcell3p, 0, mxCreateString("arduinodriver.ArduinoDigitalIO"));
  mxSetCell(mxcell3p, 1, mxCreateString("arduinodriver.ArduinoAnalogInput"));
  mxSetCell(mxcell3p, 2, mxCreateString("arduinodriver.arduinoPWMAddLibrary"));
  return(mxcell3p);
}

mxArray *sf_c2_model_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("late");
  mxArray *fallbackReason = mxCreateString("ir_function_calls");
  mxArray *hiddenFallbackType = mxCreateString("");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("digitalIOSetup");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_model_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPiA2YGRgYAPSHEDMxAABrFA+IxKGiLPAxRWAuKSyIBUkXlyU7JkCpPMSc8H8xNI"
    "Kz7y0fLD5FgwI89kImM8JFYcABQfK9EP854GknwWLfiEk/QJQfmZxfGJySWZZanyyUXxufkpqDs"
    "I8EAAAg1AShQ=="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sFCAAvrN3GnboVOwXdl2t7";
}

static void sf_opaque_initialize_c2_model(void *chartInstanceVar)
{
  initialize_params_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
  initialize_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_model(void *chartInstanceVar)
{
  enable_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_model(void *chartInstanceVar)
{
  disable_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_model(void *chartInstanceVar)
{
  sf_gateway_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_model(SimStruct* S)
{
  return get_sim_state_c2_model((SFc2_modelInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_model(SimStruct* S, const mxArray *st)
{
  set_sim_state_c2_model((SFc2_modelInstanceStruct*)sf_get_chart_instance_ptr(S),
    st);
}

static void sf_opaque_cleanup_runtime_resources_c2_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_modelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_model_optimization_info();
    }

    mdl_cleanup_runtime_resources_c2_model((SFc2_modelInstanceStruct*)
      chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c2_model(void *chartInstanceVar)
{
  mdl_start_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c2_model(void *chartInstanceVar)
{
  mdl_terminate_c2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_model((SFc2_modelInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_model((SFc2_modelInstanceStruct*)
      sf_get_chart_instance_ptr(S));
    initSimStructsc2_model((SFc2_modelInstanceStruct*)sf_get_chart_instance_ptr
      (S));
  }
}

const char* sf_c2_model_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [62] = {
    "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",
    ""
  };

  static char newstr [4561] = "";
  newstr[0] = '\0';
  for (i = 0; i < 62; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c2_model(SimStruct *S)
{
  const char* newstr = sf_c2_model_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(4233770986U));
  ssSetChecksum1(S,(1593589324U));
  ssSetChecksum2(S,(3165501813U));
  ssSetChecksum3(S,(280005752U));
}

static void mdlRTW_c2_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c2_model(SimStruct *S)
{
  SFc2_modelInstanceStruct *chartInstance;
  chartInstance = (SFc2_modelInstanceStruct *)utMalloc(sizeof
    (SFc2_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc2_modelInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_model;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_model;
  chartInstance->chartInfo.mdlStart = sf_opaque_mdl_start_c2_model;
  chartInstance->chartInfo.mdlTerminate = sf_opaque_mdl_terminate_c2_model;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c2_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_model;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_model;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_model;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_model;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->chartInfo.chartStateSetterFcn = NULL;
  chartInstance->chartInfo.chartStateGetterFcn = NULL;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0,
    chartInstance->c2_JITStateAnimation,
    chartInstance->c2_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c2_model(chartInstance);
}

void c2_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c2_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
