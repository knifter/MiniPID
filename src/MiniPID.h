/*
 * MiniPID.h
 *
 *  Created on: 5 apr. 2018
 *      Author: Tijs
 */

#ifndef __MINIPID_H_
#define __MINIPID_H_


class MiniPID{
public:
	MiniPID();
	MiniPID(const double, const double, const double);
	MiniPID(const double, const double, const double, const double);
	void setParameters(const double p, const double i, const double d);
	void setParameters(const double p, const double i, const double d, const double f);

	void setMaxIOutput(const double);
	void setOutputLimits(const double);
	void setOutputLimits(const double, const double);
	void getOutputLimits(double*, double*);
	void setReversed(bool);
	void setSetpoint(const double setpoint);
	double getSetpoint();
	void reset();
	void setOutputRampRate(double);
	void setSetpointRange(double);
	void setOutputFilter(double);
	double getOutput(const double actual);
	double getOutput(const double actual, const double setpoint);

	typedef struct
	{
		double Pterm;
		double Iterm;
		double Dterm;
		double Fterm;
		double input;
		double error;
		double output;
	} StatusPoint;

protected:
	int clamp(double* v, double min, double max);
	bool isbetween(double v, double min ,double max);
	void checkSigns();
	void init();
	double P;
	double I;
	double D;
	double F;

	double maxIOutput;
	double errorSum;

	double maxOutput;
	double minOutput;

	double setpoint;

	double lastError;

	bool firstRun;
	bool reversed;

	double outputRampRate;
	double lastOutput;

	double outputFilter;

	double setpointRange;

	int outputClampedByRamprate = 0;
	int outputClampedByMinMax = 0;

	StatusPoint _last;
};

#endif /* __MINIPID_H_ */
