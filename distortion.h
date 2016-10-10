//------------------------------------------------------------------------------
// VST Effect Plug-in
//
// Filename     : Distortion.h
// Created by   : music424
// Edited by    : Victoria Grace
// Company      : CCRMA - Stanford University
// Description  : 
//
// Date         : 5/10/16
//------------------------------------------------------------------------------

#ifndef __Distortion__
#define __Distortion__

#include "public.sdk/source/vst2.x/audioeffectx.h"
#include <math.h>

#ifndef dB
// if below -100dB, set to -100dB to prevent taking log of zero
#define dB(x)               20.0 * ((x) > 0.00001 ? log10(x) : log10(0.00001))
#endif

#ifndef dB2mag
#define dB2mag(x)           pow( 10.0, (x) / 20.0 )
#endif

#define kMaxLen			32

#define pi 3.14159265358979


//------------------------------------------------------------------------------
// signal processing functions
struct Biquad {
    //  biquad filter section
    double	b0, b1, b2, a1, a2, z1, z2;
    
    Biquad() {
        this->b0=1.0;
        this->b1=0.0;
        this->b2=0.0;
        this->a1=0.0;
        this->a2=0.0;
        reset();
    }
    void setCoefs(double* coefs) {
        // set filter coefficients [b0 b1 b2 a1 a2]
        this->b0=*(coefs);
        this->b1=*(coefs+1);
        this->b2=*(coefs+2);
        this->a1=*(coefs+3);
        this->a2=*(coefs+4);
    }
    void reset() {
        // reset filter state
        z1=0;
        z2=0;
    }
    void process (double input, double& output) {
        // process input sample, direct form II transposed
        output = z1 + input*b0;
        z1 = z2 + input*b1 - output*a1;
        z2 = input*b2 - output*a2;
    }
};


//------------------------------------------------------------------------------
class Distortion : public AudioEffectX
{
public:
	Distortion (audioMasterCallback audioMaster);
	~Distortion ();
    
	// Processing
	virtual void processReplacing (float** inputs, float** outputs, 
                                   VstInt32 sampleFrames);
    
	// Program
	virtual void setProgramName (char* name);
	virtual void getProgramName (char* name);
    
	// Parameters
	virtual void setParameter (VstInt32 index, float value);
	virtual float getParameter (VstInt32 index);
	virtual void getParameterLabel (VstInt32 index, char* label);
	virtual void getParameterDisplay (VstInt32 index, char* text);
	virtual void getParameterName (VstInt32 index, char* text);
    
	void bilinearTransform(double acoefs[], double dcoeffs[]);
	void designParametric(double* peqcofs, double center, double gain, double qval);
    
	virtual bool getEffectName (char* name);
	virtual bool getVendorString (char* text);
	virtual bool getProductString (char* text);
	virtual VstInt32 getVendorVersion ();
    
    
protected:
	char	programName[kVstMaxProgNameLen + 1];
    
	// configuration
	enum { 
		kNumProgs	= 1,
		kNumInputs	= 2,
		kNumOutputs	= 2
	};
    
	// user interface parameters
	enum {
		kParamDrive,
		kParamLevel,
		kParamGainIn,
		kParamFcIn,
		kParamQIn,
		kParamGainOut,
		kParamFcOut,
		kParamQOut,
		kNumParams
	};
    
	float DriveKnob, DriveValue;	// input gain, dB
	float LevelKnob, LevelValue;	// output gain, dB
    
	float GainInKnob, GainInValue;	// input filter gain, dB
	float FcInKnob, FcInValue;	// input filter center frequency, Hz
	float QInKnob, QInValue;	// input filter resonance, ratio
    
	float GainOutKnob, GainOutValue;	// output filter gain, dB
	float FcOutKnob, FcOutValue;	// output filter center frequency, Hz
	float QOutKnob, QOutValue;	// output filter resonance, ratio
    
    
    // signal processing parameters and state
	double fs;	// sampling rate, Hz
    
	double drive, level;	// input, output gains, amplitude
    
	double InCoefs[5];	// input filter coefficients
	Biquad InFilter;	// input filter
    
	double OutCoefs[5];	// input filter coefficients
	Biquad OutFilter;	// output filter
    
	enum{kUSRatio = 8};	// upsampling factor, sampling rate ratio
	enum{kAAOrder = 6};	// antialiasing/antiimaging filter order, biquads
    enum{kDCOrder= 2};
    
	Biquad AIFilter[kAAOrder];	// antiimaging filter
	Biquad AAFilter[kAAOrder];	// antialiasing filter
    Biquad DCFilter[kDCOrder];
    
};


// antialiasing/antiimaging filter coefficients
// Needs modification (cutoff freq, number of biquads)
// If number of biquads changed, make sure to change kAAOrder above.


// TODO: Design the Anti-alias/Anti-imaging filter in matlab and input here
///////////////START//////////////////
const static double DCCoefs[2][5]={
    {0.9996, -0.9996, 0, -0.9993, 0 },
    {0.9996, -0.9996, 0, -0.9993, 0 },
};


const static double AACoefs[6][5] = {
    
   

    


// conservative
    {0.000018821,0.000014982,0.000018821,-1.4399,0.5200},
    {1.0000,-1.1492,1.0000,-1.4973,0.5737},
    {1.0000,-1.6141,1.0000,-1.5896,0.6604},
    {1.0000,-1.7634,1.0000,-1.6921,0.7579},
    {1.0000,-1.8228,1.0000,-1.7920,0.8548},
    {1.0000,-1.8452,1.0000,-1.8881,0.9507},
    
// // smaller cutoff
//    {0.000016347,0.000095469,0.000016347,-1.4956,0.5607},
//    {1.0000,-1.2995,1.0000,-1.5493,0.6111},
//    {1.0000,-1.6904,1.0000,-1.6348,0.6918},
//    {1.0000,-1.8117,1.0000,-1.7290,0.7816},
//    {1.0000,-1.8594,1.0000,-1.8199,0.8698},
//    {1.0000,-1.8774,1.0000,-1.9065,0.9560},
//
    //20th order
//    {0.000021106,0.000030846,0.000021106,-1.0708,0.2893},
//    {1.0000,-0.3175,1.0000,-1.1382,0.3448},
//    {1.0000,-1.1492,1.0000,-1.2483,0.4363},
//    {1.0000,-1.4937,1.0000,-1.3782,0.5462},
//    {1.0000,-1.6570,1.0000,-1.5059,0.6262},
//    {1.0000,-1.7438,1.0000,-1.5236,0.6781},
//    {1.0000,-1.7935,1.0000,-1.6672,0.8099},
//    {1.0000,-1.8228,1.0000,-1.7942,0.8786},
//    {1.0000,-1.8393,1.0000,-1.8047,0.9287},
//    {1.0000,-1.8469,1.0000,-1.8786,0.9845},

    
};

////////////////END/////////////////////







// input drive limits, dB; taper, exponent
const static float DriveLimits[2] = {-24.0, 24.0};
const static float DriveTaper = 1.0;

// output level limits, dB; taper, exponent
const static float LevelLimits[2] = {-48.0, 24.0};
const static float LevelTaper = 1.0;


// input filter gain limits, dB; taper, exponent
const static float GainInLimits[2] = {-24.0, 24.0};
const static float GainInTaper = 1.0;

// input filter center frequency limits, hz; taper, exponent
const static float FcInLimits[2] = {50.0, 5000.0};
const static float FcInTaper = -1.0;

// input filter resonance limits, dB; taper, exponent
const static float QInLimits[2] = {0.25, 32.0};
const static float QInTaper = -1.0;


// output filter gain limits, dB; taper, exponent
const static float GainOutLimits[2] = {-24.0, 24.0};
const static float GainOutTaper = 1.0;

// output filter center frequency limits, hz; taper, exponent
const static float FcOutLimits[2] = {50.0, 5000.0};
const static float FcOutTaper = -1.0;

// output filter resonance limits, dB; taper, exponent
const static float QOutLimits[2] = {0.25, 32.0};
const static float QOutTaper = -1.0;


//------------------------------------------------------------------------------
// "static" class to faciliate the knob handling
class SmartKnob {
public:
    // convert knob on [0,1] to value in [limits[0],limits[1]] according to taper
    static float knob2value(float knob, const float *limits, float taper)
    {
        float value;
        if (taper > 0.0) {  // algebraic taper
            value = limits[0] + (limits[1] - limits[0]) * pow(knob, taper);
        } else {            // exponential taper
            value = limits[0] * exp(log(limits[1]/limits[0]) * knob);
        }
        return value;
    };
    
    // convert value in [limits[0],limits[1]] to knob on [0,1] according to taper
    static float value2knob(float value, const float *limits, float taper)
    {
        float knob;
        if (taper > 0.0) {  // algebraic taper
            knob = pow((value - limits[0])/(limits[1] - limits[0]), 1.0/taper);
        } else {            // exponential taper
            knob = log(value/limits[0])/log(limits[1]/limits[0]);
        }
        return knob;
    };
    
};


#endif	// __Distortion_HPP


