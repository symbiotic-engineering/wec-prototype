
  #define gearRatio 10.0    // RM5 gear ratio
  #define kp 3000           // controller stiffness             [N/m]
  #define bp 1500            // controller damping               [Ns/m]
  #define k_b 11.5           // motor constant               [Amp/Nm]
  #define r_eff 0.010/(2*3.14)  // effective radius of the lead screw = lead/2pi           [m]
  #define I_thresh 13   // current threshold                [Amp]
  float v ;
  float x ;


float calcVoltage(float I, float w_motor, float w_wave, float theta, int RMtype) {

  if (RMtype = 3) {
    v = w_motor * r_eff;
    x = theta * r_eff;
  } else if (RMtype = 5) {
    v = w_motor * gearRatio;
    x = theta * gearRatio;
  }

  float F_desired = kp*x + bp*v;

  float I_desired = F_desired * r_eff * k_b;

  Serial.print("kp*x = ");
  Serial.println(kp*x);
  Serial.print("bp*v = ");
  Serial.println(bp*v);
  Serial.print("I_desired = ");
  Serial.println(I_desired);

  if (abs(I_desired) > I_thresh) {
    I_desired = 0.0; // if spinning uncontrollably, stop commanding force
  }

  return I_desired;
}
