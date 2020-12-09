/*
 * MiniPID.cpp
 *
 *  Created on: 5 apr. 2018
 *      Author: Tijs (not original author)
 */

/**
* Small, easy to use PID implementation with advanced controller capability.<br>
* Minimal usage:<br>
* setPID(p,i,d); <br>
* ...looping code...{ <br>
* output=getOutput(sensorvalue,target); <br>
* }
*
* @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
*/
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include "MiniPID.h"

//**********************************
//Constructor functions
//**********************************
MiniPID::MiniPID()
{
	init();
};

MiniPID::MiniPID(const double p, const double i, const double d)
{
	init();
	P=p; 
	I=i; 
	D=d;
};

MiniPID::MiniPID(const double p, const double i, const double d, const double f)
{
	init();
	P=p; 
	I=i; 
	D=d; 
	F=f;
};

void MiniPID::init()
{
	P=0.0;
	I=0.0;
	D=0.0;
	F=0.0;

	maxIOutput=0;
	errorSum=0;
	maxOutput=0;
	minOutput=0;
	setpoint=0;
	lastError=0;
	firstRun=true;
	reversed=false;
	outputRampRate=0;
	lastOutput=0;
	outputFilter=0;
	setpointRange=0;
}

//**********************************
//Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter. <br>
 * this->responds quicly to changes in setpoint, and provides most of the initial driving force
 * to make corrections. <br>
 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
 * For position based controllers, this->is the first parameter to tune, with I second. <br>
 * For rate controlled systems, this->is often the second after F.
 *
 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>
 */
void MiniPID::setParameters(const double p, const double i, const double d)
{
	P=p;
	I=i;
	D=d;
	F=0;
	checkSigns();
};

void MiniPID::setParameters(const double p, const double i, const double d, const double f)
{
	P=p;
	I=i;
	D=d;
	F=f;
	checkSigns();
};

/**Set the maximum output value contributed by the I component of the system
 * this->can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void MiniPID::setMaxIOutput(const double maximum)
{
	/* Internally maxError and Izone are similar, but scaled for different purposes.
	 * The maxError is generated for simplifying math, since calculations against
	 * the max error are far more common than changing the I term or Izone.
	 */
	maxIOutput=maximum;
}

/**Specify a maximum output. If a single parameter is specified, the minimum is
 * set to (-maximum).
 * @param output
 */
void MiniPID::setOutputLimits(const double output)
{ 
	setOutputLimits(-output,output);
};

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void MiniPID::setOutputLimits(const double minimum, const double maximum)
{
	if(maximum<minimum)
		return;
	maxOutput=maximum;
	minOutput=minimum;

	// Ensure the bounds of the I term are within the bounds of the allowable output swing
	if(maxIOutput==0 || maxIOutput>(maximum-minimum) )
	{
		setMaxIOutput(maximum-minimum);
	};
};

/** Set the operating direction of the PID controller
 * @param reversed Set true to reverse PID output
 */
void MiniPID::setReversed(bool reversed)
{
	this->reversed = reversed;
};

//**********************************
//Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void MiniPID::setSetpoint(double setpoint)
{
	this->setpoint = setpoint;
#ifdef MINIPID_SUPPRESS_DKICK
	this->lastError += setpoint;
#endif
};

double MiniPID::getSetpoint()
{
	return setpoint;
};

/** Calculate the PID value needed to hit the target setpoint.
* Automatically re-calculates the output at each call.
* @param actual The monitored value
* @param target The target value
* @return calculated output value for driving the actual to the target
*/
double MiniPID::getOutput(const double actual)
{
	double output;

	double sp = setpoint;

#ifdef MINIPID_RAMP_SETPOINT
	//Ramp the setpoint used for calculations if user has opted to do so
	if(setpointRange!=0)
	{
		sp=clamp(setpoint,actual-setpointRange,actual+setpointRange);
	}
#endif

#ifdef MINIPID_DT_CALC
	static struct timeval last, now;
	gettimeofday(&now, NULL);
	if(firstRun)
		last = now;
	double dt = TIMEVAL_DIFF_SEC(now, last);
	last = now;
#else
	double dt = 1.0;
#endif

	//Do the simple parts of the calculations
	double error = sp-actual;

	//Calculate F output. Notice, this depends only on the setpoint, and not the error.
	double Foutput = F*sp;

	//Calculate P term
	double Poutput = P*error;

	//If this->is our first time running this-> we don't actually have a previous input or output.
	//For sensor, sanely assume it was exactly where it is now.
	//For last output, we can assume it's the current time-independent outputs.
	if(firstRun)
	{
		lastError = error;
		errorSum = 0;
		lastOutput = Poutput + Foutput;
		firstRun = false;
	};

	//Calculate D Term
	//Note, this->is negative. this->actually "slows" the system if it's doing
	//the correct thing, and small values helps prevent output spikes and overshoot
	double Doutput= D*(error-lastError) / dt;
	lastError=error;

	//The Iterm is more complex. There's several things to factor in to make it easier to deal with.
	// 1. The multiplication with I-gain is done when adding to the sum, this prevents the bump on I-gain changes
	// 2. prevent windup by not increasing errorSum if output is output=maxOutput or output=minOutput
	// 3. prevent windup by not increasing errorSum if we're already running against our max Ioutput
	// 3b. But only if the outputclamp and error have the same sign (direction)
	bool freeze_integral = outputClampedByRamprate || (outputClampedByMinMax && outputClampedByMinMax*error > 0);

	// If all good, increase integral
	if(!freeze_integral)
		errorSum+=I*error*dt;

	// 3. maxIoutput restricts the amount of output contributed by the Iterm.
	if(maxIOutput != 0)
		clamp(&errorSum, -maxIOutput, maxIOutput);

	// Now our I output term is just the sum as the I factor is already processed while adding to the sum previously
	double Ioutput = errorSum;

	//And, finally, we can just add the terms up
	output = Foutput + Poutput + Ioutput + Doutput;

	// Limit the output by ramprate
	outputClampedByRamprate = false;
	if(outputRampRate !=0 )
	{
		outputClampedByRamprate = clamp(&output, lastOutput-outputRampRate, lastOutput+outputRampRate);
	};

	// Limit the output by min/maxOutput
	outputClampedByMinMax = false;
	if(minOutput != maxOutput)
	{
		outputClampedByMinMax = clamp(&output, minOutput, maxOutput);
	};

	// Filter the Output
	if(outputFilter!=0)
	{
		output=lastOutput*outputFilter+output*(1-outputFilter);
	};

	// Gather statusdata
	_last.input = actual;
	_last.Fterm = Foutput;
	_last.Pterm = Poutput;
	_last.Iterm = Ioutput;
	_last.Dterm = Doutput;
	_last.output = output;
	_last.error = error;

	lastOutput = output;
	return output;
}

/**
 * Calculates the PID value using the last provided setpoint and actual valuess
 * @return calculated output value for driving the actual to the target
 */
double MiniPID::getOutput(const double actual, const double setpoint)
{
	setSetpoint(setpoint);
	return getOutput(actual);
};

/**
 * Resets the controller. this erases the I term buildup, and removes D gain on the next loop.
 */
void MiniPID::reset()
{
	firstRun=true;
	errorSum=0;
};

/**Set the maximum rate the output can increase per cycle.
 * @param rate
 */
void MiniPID::setOutputRampRate(double rate)
{
	outputRampRate = rate;
};

/** Set a limit on how far the setpoint can be from the current position
 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range.
 * <br>this->limits the reactivity of P term, and restricts impact of large D term
 * during large setpoint adjustments. Increases lag and I term if range is too small.
 * @param range
 */
void MiniPID::setSetpointRange(double range)
{
	setpointRange = range;
};

/**Set a filter on the output to reduce sharp oscillations. <br>
 * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I values.
 * Uses an exponential rolling sum filter, according to a simple <br>
 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
 * @param output valid between [0..1), meaning [current output only.. historical output only)
 */
void MiniPID::setOutputFilter(double strength)
{
	if(strength < 0 || strength >= 1)
		return;
	outputFilter = strength;
};

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise
 */
int MiniPID::clamp(double* value, double min, double max)
{
	if(*value > max)
	{
		*value = max;
		return 1;
	};
	if(*value < min)
	{
		*value = min;
		return -1;
	};
	return 0;
};

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool MiniPID::isbetween(double value, double min, double max)
{
		return (min<value) && (value<max);
};

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed value
 */
void MiniPID::checkSigns()
{
	if(reversed)
	{	//all values should be below zero
		if(P>0) P*=-1;
		if(I>0) I*=-1;
		if(D>0) D*=-1;
		if(F>0) F*=-1;
	}else{	//all values should be above zero
		if(P<0) P*=-1;
		if(I<0) I*=-1;
		if(D<0) D*=-1;
		if(F<0) F*=-1;
	};
};

