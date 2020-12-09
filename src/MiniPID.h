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
	MiniPID(double, double, double);
	MiniPID(double, double, double, double);
	void setP(double);
	void setI(double);
	void setD(double);
	void setF(double);
	void setPID(double, double, double);
	void setPID(double, double, double, double);
	void setMaxIOutput(double);
	void setOutputLimits(double);
	void setOutputLimits(double,double);
	void getOutputLimits(double*, double*);
	void setDirection(bool);
	void setSetpoint(double);
	double getSetpoint();
	void reset();
	void setOutputRampRate(double);
	void setSetpointRange(double);
	void setOutputFilter(double);
	double getOutput(double actual);
	double getOutput(double actual, double setpoint);

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
	bool between(double v, double min ,double max);
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
