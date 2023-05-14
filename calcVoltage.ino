
  #define gearRatio 10.0    // RM5 gear ratio
  #define kp 600           // controller stiffness             [N/m]
  #define bp 300            // controller damping               [Ns/m]
  #define k_b 5           // motor constant                   [Nm/Amp (?)]
  #define r_pinion 0.01  // lead of the lead screw           [m]
  #define I_thresh 13   // current threshold                [Amp]
  float v ;
  float x ;


float calcVoltage(float I, float w_motor, float w_wave, float theta, int RMtype) {

  if (RMtype = 3) {
    v = w_motor * r_pinion;
    x = theta * r_pinion;
  } else if (RMtype = 5) {
    v = w_motor * gearRatio;
    x = theta * gearRatio;
  }

  float F_desired = kp*x + bp*v;

  float I_desired = F_desired * r_pinion * k_b;

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
