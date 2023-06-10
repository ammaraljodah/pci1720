#define     S_FUNCTION_LEVEL  2
#undef      S_FUNCTION_NAME
#define     S_FUNCTION_NAME   advantechpci1720

#include "simstruc.h"

#ifdef  MATLAB_MEX_FILE
#include    "mex.h"
#endif

#ifndef MATLAB_MEX_FILE
#include    <windows.h>
#include    "io_xpcimport.h"
#include    "pci_xpcimport.h"
#include    "util_xpcimport.h"
#include    "xpctarget.h"
#endif

#define NUM_ARGS        7
#define CHANNEL_ARG     ssGetSFcnParam(S,0) // vector of 1:16
#define RANGE_ARG       ssGetSFcnParam(S,1) // vector of {5, 10, -5, -10}
#define RESET_ARG       ssGetSFcnParam(S,2) // boolean 0 do not reset, 1 reset
#define INIT_VAL_ARG    ssGetSFcnParam(S,3) // vector of double
#define SAMP_TIME_ARG   ssGetSFcnParam(S,4) // seconds
#define PCI_SLOT_ARG    ssGetSFcnParam(S,5) // integer
#define PCI_BUS_ARG     ssGetSFcnParam(S,6) // integer



#define SAMP_TIME_IND   (0)

#define NUM_I_WORKS     (1) // just to store the base address
#define BASE_ADDR_I_IND (0)

#define NUM_R_WORKS     (0)

static char_T msg[256];

// card required param
#define AO_GAIN_V_0To5          0x00 // channels gains values. Each channel has two bits that control its range
#define AO_GAIN_V_Neg5To5       0x02 // the ranges of all channels make a single byte
#define AO_GAIN_V_0To10         0x01 // thus this byte needs to be combosed
#define AO_GAIN_V_Neg10To10     0x03


#define DR_AO_DATA_BASE         0x00 // the data base address
#define DR_AO_SYNC_CTL          0x0f // the address to control SYNC_STROBE , writing 1 to it make the card sync the writing for the channels
#define DR_AO_SYNC_STROBE       0x09 // the address to activate strobe (strobe means sending all the votages at the same time to the output)
#define DR_AO_CHCSR             0x08 // the address that control the range for the channels


static uint16_T volt2count(real_T volt,  size_t i);


static void mdlInitializeSizes(SimStruct *S)
{
    size_t i;
    
#ifndef MATLAB_MEX_FILE
#include    "io_xpcimport.c"
#include    "pci_xpcimport.c"
#include    "util_xpcimport.c"
#endif
    
    ssSetNumSFcnParams(S, NUM_ARGS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        sprintf(msg, "%d input args expected, %d passed", NUM_ARGS, ssGetSFcnParamsCount(S));
        ssSetErrorStatus(S, msg);
        return;
    }
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    if( !ssSetNumOutputPorts(S, 0) ) return;
    
    if( !ssSetNumInputPorts(S, (int_T)mxGetN(CHANNEL_ARG)) ) return;
    for (i = 0; i < mxGetN(CHANNEL_ARG); i++) {
        ssSetInputPortWidth(S, i, 1);
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, NUM_R_WORKS);
    ssSetNumIWork(S, NUM_I_WORKS);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    for (i = 0; i < NUM_ARGS; i++)
        ssSetSFcnParamTunable(S, i, 0);
    
    ssSetSimStateCompliance(S, HAS_NO_SIM_STATE);
    
    ssSetOptions(S, SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME | SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    if (mxGetPr(SAMP_TIME_ARG)[0] == -1.0) {
        ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
        ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
        ssSetModelReferenceSampleTimeInheritanceRule(S, USE_DEFAULT_FOR_DISCRETE_INHERITANCE);
    } else {
        ssSetSampleTime(S, 0, mxGetPr(SAMP_TIME_ARG)[0]);
        ssSetOffsetTime(S, 0, 0.0);
    }
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
    int_T  *IWork    = ssGetIWork(S);
    real_T *RWork    = ssGetRWork(S);
    int_T   pciSlot  = (int_T)mxGetPr(PCI_SLOT_ARG)[0];
    int_T   pciBus  = (int_T)mxGetPr(PCI_BUS_ARG)[0];
    size_t  nChans   = mxGetN(CHANNEL_ARG);
    char_T  *deviceName;
    size_t  i;
    int_T   base, deviceId,vendorId;
    uint16_T value;
    real_T  voltage;
    uint16_T count,chan;
    real_T rangevalue;
    
    PCIDeviceInfo pciInfo;
    
    vendorId = 0x13fe; // vender id for Advantec
    deviceId = 0x1720; // device id for PCI-1716
    deviceName = "PCI-1723"; // PCI-1716 Name
    
    //do auto search, this code is to find the pci slot and pci bus
    if (pciSlot < 0) {
        if (rl32eGetPCIInfo((uint16_T) vendorId,
                (uint16_T) deviceId, &pciInfo)) {
            sprintf(msg, "No %s found", deviceName);
            ssSetErrorStatus(S, msg);
            return;
        }
        else
            sprintf(msg, "%s has been found", deviceName);
    }
    // if you know the bus and slot you can specify them, you can use "getxpcpci all" in matlab command window to know the bus and slot number
    else if (rl32eGetPCIInfoAtSlot((uint16_T)vendorId, (uint16_T)deviceId,
            pciSlot + pciBus * 256, &pciInfo)) {
        sprintf(msg, "No %s at bus %d slot %d", deviceName, pciBus, pciSlot);
        ssSetErrorStatus(S, msg);
        return;
    }
    // get the base address, for Advantech cards the I/O base is in location 2
    base = pciInfo.BaseAddress[2];
    // store the base address in the integer work vector
    IWork[BASE_ADDR_I_IND] = base;
    
    // find the coresponding code for the range
    value = 0x00; // the byte to be writen on the range register
    for (i = 0; i < nChans; i++)
    {
        chan = (uint16_T) mxGetPr(CHANNEL_ARG)[i];
        rangevalue=((real_T)mxGetPr(RANGE_ARG)[i]);
        value &= ~(0x3 << (chan * 2)); // nulling the two bits that control the channel
        if (rangevalue==5)
            value |= Ao_GAIN_V_0To5 << (chan * 2); // oring the two bits to set the range for the channel chan
        if (rangevalue==-5)
            value |= AO_GAIN_V_Neg5To5 << (chan * 2);
        if (rangevalue==10)
            value |= AO_GAIN_V_0To10 << (chan * 2);
        if (rangevalue==-10)
            value |= AO_GAIN_V_Neg10To10 << (chan * 2);
    }
    rl32eOutpB(base + DR_AO_CHCSR, value); // set the range by writing it to the range register
    
    // enable synchronized output
    rl32eOutpW(base + DR_AO_SYNC_CTL,  0x0001);
    
    //output the intial vals
    for (i = 0; i < nChans; i++)
    {
        chan = (uint16_T) mxGetPr(CHANNEL_ARG)[i];
        voltage = mxGetPr(INIT_VAL_ARG)[i];
        count=volt2count(voltage, i);
        rl32eOutpW(base + DR_AO_DATA_BASE + (chan*2) ,  count);
    }
    // strobe the synchronized output
    rl32eOutpW(base + DR_AO_SYNC_STROBE,  0x0000);
#endif
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE
    real_T *RWork    = ssGetRWork(S);
    int_T  *IWork    = ssGetIWork(S);
    int_T   base     = ssGetIWorkValue(S, BASE_ADDR_I_IND);
    size_t  nChans   = mxGetN(CHANNEL_ARG);
    size_t  i;
    uint16_T count,chan;
    real_T voltage;
    InputRealPtrsType uPtrs;
    
    for (i = 0; i < nChans; i++)
    {
        chan = (uint16_T) mxGetPr(CHANNEL_ARG)[i];
        uPtrs = ssGetInputPortRealSignalPtrs(S, i);
        voltage = *uPtrs[0];
        count=volt2count(voltage, i);
        rl32eOutpW(base + DR_AO_DATA_BASE + (chan*2) ,  count);
    }
    // strobe the synchronized output
    rl32eOutpW(base + DR_AO_SYNC_STROBE,  0x0000);
    
#endif
}

static void mdlTerminate(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
    int_T   base     = ssGetIWorkValue(S, BASE_ADDR_I_IND);
    size_t  nChans   = mxGetN(CHANNEL_ARG);
    size_t  i;
    uint16_T count,chan;
    real_T voltage;
    
    
    //output the intial vals
    for (i = 0; i < nChans; i++)
    {
        if (!xpcIsModelInit() && !(uint_T)mxGetPr(RESET_ARG)[i]) // checking if this channel needs to be rested as per the user given paramter
            continue;
        chan = (uint16_T) mxGetPr(CHANNEL_ARG)[i];
        voltage = mxGetPr(INIT_VAL_ARG)[i];
        count=volt2count(voltage, i);
        rl32eOutpW(base + DR_AO_DATA_BASE + (chan*2) ,  count);
    }
    // strobe the synchronized output
    rl32eOutpW(base + DR_AO_SYNC_STROBE,  0x0000);
    
#endif
}

uint16_T volt2count(real_T volt, size_t i)
{
#ifndef MATLAB_MEX_FILE
    real_T rangevalue = (real_T)mxGetPr(RANGE_ARG)[i];
    uint16_T out;
    real_T MAX_VOLT,MIN_VOLT,FS;
    
    if (rangevalue==5)
    {MIN_VOLT = 0; MAX_VOLT = 5; FS = 4095;} // min voltage, max voltage, full scale 
    if (rangevalue==-5)
    {MIN_VOLT = -5; MAX_VOLT = 5;FS = 2047;}
    if (rangevalue==10)
    {MIN_VOLT = 0; MAX_VOLT = 10; FS = 4095;}
    if (rangevalue==-10)
    {MIN_VOLT = -10; MAX_VOLT = 10;FS = 2047;}
    
    if (volt>MAX_VOLT)
        volt=MAX_VOLT;
    if (volt<MIN_VOLT)
        volt=MIN_VOLT;    
    
    if (MIN_VOLT == 0)
        out= (uint16_T)(volt * FS/MAX_VOLT); // unipolar 
    else
        if (volt > 0)  // Bipolar 
            out=(uint16_T)(2048+ volt*FS/MAX_VOLT);
        else
            out=(uint16_T)((MAX_VOLT + volt)*((FS+1)/MAX_VOLT));
    
    return out;
#endif
}

#ifdef MATLAB_MEX_FILE  /* Is this file being compiled as a MEX-file? */
#include "simulink.c"   /* Mex glue */
#else
#include "cg_sfun.h"    /* Code generation glue */
#endif


