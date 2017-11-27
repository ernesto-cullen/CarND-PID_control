#ifndef PID_H
#define PID_H

class PID {
public:
  /*
   * previous cte
   */
  double prev_cte;

  /*
   * sum of cte over time (integral)
   */
  double sum_cte;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp, Kpi;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);


  double getSteerValue(double cte);

};

#endif /* PID_H */
