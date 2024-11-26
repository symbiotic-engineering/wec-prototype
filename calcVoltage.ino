
  #define gearRatio 10.0    // RM5 gear ratio
  #define kp 3000           // controller stiffness             [N/m]
                                                                        
  #define bp 1500            // controller damping               [Ns/m] 
  #define k_b 11.5           // motor constant               [Amp/Nm]   
  #define r_eff 0.010/(2*3.14)  // effective radius of the lead screw = lead/2pi           [m]
  #define I_thresh 13   // current threshold                [Amp]
  float v ;
  float x ;

 /*void set_parameters(int frequency, int converterType) {
    
    // Set kp and bp based on frequency
    switch (frequency) {
        case frequency1:
            kp = 10.0;
            bp = 1.0;
            break;
        case frequency2:
            kp = 20.0;
            bp = 2.0;
            break;
        case frequency3:
            kp = 30.0;
            bp = 3.0;
            break;
        default:
            printf("Invalid frequency\n");
            return;
    }

  void setConverterParameters(int converterType, int frequencyType) {
    // Initialize variables
    double mass = 0.0;
    double added_mass = 0.0;
    double radiation_damping = 0.0;
    double stiffness = 0.0;

    // Set mass, added mass, radiation damping, and stiffness based on converter type and frequency type
    switch (converterType) {
        case 0: // Point Absorber
            switch (frequencyType) {
                case 0: // Low frequency
                    mass = 5.0;
                    added_mass = 1.0;
                    radiation_damping = 0.5;
                    stiffness = 100.0;
                    break;
                case 1: // Medium frequency
                    mass = 6.0;
                    added_mass = 1.5;
                    radiation_damping = 0.6;
                    stiffness = 150.0;
                    break;
                case 2: // High frequency
                    mass = 7.0;
                    added_mass = 2.0;
                    radiation_damping = 0.7;
                    stiffness = 200.0;
                    break;
                default:
                    printf("Invalid frequency type for Point Absorber\n");
                    return;
            }
            break;

        case 1: // Oscillating Water Column
            switch (frequencyType) {
                case 0: // Low frequency
                    mass = 10.0;
                    added_mass = 2.0;
                    radiation_damping = 1.0;
                    stiffness = 200.0;
                    break;
                case 1: // Medium frequency
                    mass = 12.0;
                    added_mass = 2.5;
                    radiation_damping = 1.2;
                    stiffness = 250.0;
                    break;
                case 2: // High frequency
                    mass = 15.0;
                    added_mass = 3.0;
                    radiation_damping = 1.5;
                    stiffness = 300.0;
                    break;
                default:
                    printf("Invalid frequency type for Oscillating Water Column\n");
                    return;
            }
            break;

        default:
            printf("Invalid converter type\n");
            return;
    }*/

  //need to add white noise function to send signal to motor, collect data on power being produced

float calcVoltage(float I, float w_motor, float w_wave, float theta, int RMtype) {

  if (RMtype = 3) {
    v = w_motor * r_eff;  // instead of r_eff, r_pin (radius of rpin is hardcode number, find out)
    x = theta   * r_eff;
  } else if (RMtype = 5) {
    v = w_motor * gearRatio;  
    x = theta   * gearRatio;
  }

  float F_desired = kp*x + bp*v;

  float I_desired = F_desired * r_eff * k_b;

  //Serial.print("kp*x = ");
  //Serial.println(kp*x);
  //Serial.print("bp*v = ");
  //Serial.println(bp*v);
  //Serial.print("I_desired = ");
  //Serial.println(I_desired);

  if (abs(I_desired) > I_thresh) {
    I_desired = 0.0; // if spinning uncontrollably, stop commanding force
  }

  return I_desired;
}
