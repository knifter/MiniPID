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
	void setOutput(double);
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

	double _kP;
	double _kI;
	double _kD;
	double _kF;

	double _maxIOutput;
	double _errorSum;

	double _maxOutput;
	double _minOutput;

	double _setpoint;

	double _lastError;

	bool _firstRun;
	bool _reversed;

	double _outputRampRate;
	double _lastOutput;

	double _outputFilter;

	double _setpointRange;

	int _outputClampedByRamprate = 0;
	int _outputClampedByMinMax = 0;

	StatusPoint _last;
};

#endif /* __MINIPID_H_ */
