#include "pid.h"

//PID controller parameters
static double Kp = 1000.0; //Tunedp1: 877.465 / agreesivep2: 342.6155
static double Ki = 0.31733; //Tunedi1: 0.31733 / agresivei2: 0.068568
static double Kd = 0.0014929; //Tunedd1: 0.0014929 / agressived2: 0.017142

//PID error parameters
static double previous_error = 0;	//previous error for D control calculation
static double PID_value = 0;		//store PID calculation

// Anti Wind-up and Clamping Variables
static double limMin = -100;
static double limMax = 100;

static double proportional = 0;
static double integrator = 0;
static double differentiator =0;


double PID(double* theta_error, double* dt, double* angle, double* previous_angle){
	double limMaxInt,limMinInt;

	//set the error components for P, I, D

	// P
	proportional = Kp * *theta_error;

	// I
	//error_sum = error_sum + (*theta_error * *dt);
	integrator = integrator + (Ki * *dt * (*theta_error + previous_error));

	// D
	if(*dt == 0){
		differentiator = 0;
		}
	else{
		differentiator = (Kd * (*angle - *previous_angle)) / *dt;
	}


	//section for capping integral windup
	if (limMax > proportional){
		limMaxInt = limMax - proportional;
	} else {
		limMaxInt = 0.0;
	}

	if (limMin < proportional){
		limMinInt = limMin - proportional;
	} else {
		limMinInt = 0.0;
	}

	// Clamp
	if(integrator > limMaxInt){
		integrator = limMaxInt;
	}
	else if(integrator< limMinInt){
		integrator = limMinInt;
	}

	previous_error = *theta_error;

	//PID

	PID_value = proportional + integrator + differentiator;
	return PID_value;

}
