package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final double gravity_acceleration = -9.8; // m per second^2
  public static final double top_limit = 0.208;   // Upper physical stop in rotations
  public static final double bottom_limit = 0;   // Bottom physical stop in rotations
  public static final double kArmReduction = 20;
  public static final double kArmMass = Units.lbsToKilograms(7); // Kilograms
  public static final double kArmLength = Units.inchesToMeters(36);  // 0.6096 m = 2"
  public static final double kStartingAngle = 0.0;
  public static final boolean direction = true;  // true is clockwise, false is counter clockwise

  public static final double kS = 0.1; // To account for friction, add 0.1 V of static feedforward
  public static final double kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second

  public static final double kP = 40.0; // An error of 1 rotation results in 2.4 V output
  public static final double kI = 0.0; // No output for integrated error 
  public static final double kD = 0.01;

  public static final double k_PeakForwardVoltage = 15;
  public static final double k_PeakReverseVoltage = -15;
  public static final double k_VoltageClosedLoopRampPeriod = 0.0;
  public static final boolean k_StatorCurrentLimitEnable = true;    
  public static final double k_StatorCurrentLimit = 120;
  public static final boolean k_SupplyCurrentLimitEnable = true;    
  public static final double k_SupplyCurrentLimit = 40;
  public static final double desiredRotations = 0.0;
}