  #define gearRatio 26    // RM5 gear ratio
  //#define kp 3000           // controller stiffness             [N/m]
  //#define bp 1500            // controller damping               [Ns/m]
  #define k_b 11.5           // motor constant               [Amp/Nm]
  #define r_eff 35/2000  // radius of pinion         [m]
  #define I_thresh 13   // current threshold                [Amp]
  float v ;
  float x ;

if (RMtype = 3) {   // for RM3 point absorber
    M = 5.3         // mass [kg]
    K = 643.7       // stiffness [N/m]
    if (frequency_in = 1){      // frequency = 1 Hz
        A = 5.36    // added mass [kg]
        B = 10.47   // radiation damping [kg/s]
    }
    if (frequency_in = 2){      // frequency = 0.8333 Hz
        A = 5.65
        B = 9.33
    }
    if (frequency_in = 3){      // frequency = 0.719 Hz
        A = 6.13
        B = 8.081
    }
// -----------------------------------------------------------//
else{                 // for RM5 flap
    M = 0.0082        // mass moment of inertia [kg-m^2]
    K = -1636.98      // stiffness [N/m]
    if (frequency_in = 1){      // frequency = 1 Hz
        A = 0.0167    // added mass [kg-m^2]
        B = 7.35   // radiation damping [kg/s]
    }
    if (frequency_in = 2){      // frequency = 0.8333 Hz
        A = 0.01676
        B = 2.862
    }
    if (frequency_in = 3){      // frequency = 0.719 Hz
        A = 0.0167
        B = 1.253
    }
}
}

// define the PTO coefficients
bp = B                  // PTO damping
kp = sqrt(K/(M + A))    // PTO stiffness

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