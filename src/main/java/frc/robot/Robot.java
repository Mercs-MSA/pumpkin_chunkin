// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sim.PhysicsSim;

// This is all simulation stuff
import com.ctre.phoenix6.sim.TalonFXSimState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final TalonFX my_KrakenX60_Motor = new TalonFX(20);
  private TalonFXConfiguration configs = new TalonFXConfiguration();

  /* Start at position 0, use slot 0 for those settings */
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  /* Start at velocity 0, use slot 1 for those settings */
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(1);
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);
  private final XboxController m_joystick = new XboxController(0);

  private boolean motor_homing_enabled = false;
  private boolean within_time_window = false;
  private final NeutralModeValue m_brake = NeutralModeValue.Brake;
  private double m_timer = 0.0;

  // Simulation Stuff
  private final Mechanisms m_mechanism = new Mechanisms();
  private final DCMotor m_armGearbox = DCMotor.getKrakenX60(1);
  public static final double kMinAngleRads = Units.rotationsToRadians(Constants.bottom_limit);
  public static final double kMaxAngleRads = Units.rotationsToRadians(Constants.top_limit);
  private final TalonFXSimState motorSim = my_KrakenX60_Motor.getSimState();
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
          Constants.kArmLength,
          kMinAngleRads,
          kMaxAngleRads,
          true,
          Constants.kStartingAngle
          );

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Common configuration settings that affect the behavior of the motor and control loops
    configs.Voltage.PeakForwardVoltage = Constants.k_PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = Constants.k_PeakReverseVoltage;
    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.k_VoltageClosedLoopRampPeriod;
    configs.CurrentLimits.StatorCurrentLimitEnable = Constants.k_StatorCurrentLimitEnable;    
    configs.CurrentLimits.StatorCurrentLimit = Constants.k_StatorCurrentLimit;
    configs.CurrentLimits.SupplyCurrentLimitEnable = Constants.k_SupplyCurrentLimitEnable;    
    configs.CurrentLimits.SupplyCurrentLimit = Constants.k_SupplyCurrentLimit;
    configs.MotorOutput.Inverted = Constants.direction ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Configuration settings specific to Slot 0, which is the position control loop in our setup
    configs.Slot0.kS = Constants.kS; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kV = Constants.kV; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second

    configs.Slot0.kP = Constants.kP; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = Constants.kI; // No output for integrated error 
    configs.Slot0.kD = Constants.kD;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = my_KrakenX60_Motor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    /* Make sure we start at position 0; this affects */
    //my_KrakenX60_Motor.setPosition(0);
  }

  @Override
  public void robotPeriodic() {
    //m_mechanism.update(my_KrakenX60_Motor.getPosition());
    SmartDashboard.putNumber("Motor Position", my_KrakenX60_Motor.getPosition().getValueAsDouble());  
    SmartDashboard.putNumber("Motor Velocity", my_KrakenX60_Motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Motor Voltage", my_KrakenX60_Motor.getMotorVoltage().getValueAsDouble());

    SmartDashboard.putNumber("Arm Position in rotations", Units.radiansToRotations(m_armSim.getAngleRads()));  
    SmartDashboard.putNumber("Arm Velocity in RPMs", Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()));

    SmartDashboard.putNumber("Pumpkin X Velocity", m_mechanism.m_pumpkin.get_velocity()[0]);
    SmartDashboard.putNumber("Pumpkin Y Velocity", m_mechanism.m_pumpkin.get_velocity()[1]);
    SmartDashboard.putNumber("Pumpkin X Position", m_mechanism.m_pumpkin.get_position()[0]);
    SmartDashboard.putNumber("Pumpkin Y Position", m_mechanism.m_pumpkin.get_position()[1]);


    if (((m_mechanism.m_pumpkin.get_position()[1]) < 0.2) && ((m_mechanism.m_pumpkin.get_position()[1]) > 0)) {
      System.out.println("Pumpking landing distance is " + (3.5 - m_mechanism.m_pumpkin.get_position()[0]));
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {

    if (m_joystick.getAButton()) {
      /* Use duty cycle request */
      my_KrakenX60_Motor.setControl(m_dutyCycle.withOutput(1.0));
    }
    if (m_joystick.getBButton()) {
      /* Use velocity voltage */
      my_KrakenX60_Motor.setControl(m_positionVoltage.withPosition(0.0));
    }

/*     else if (m_joystick.getAButton()) {
      //desiredRotations = -4.0;
      motor_homing_enabled = true;
      within_time_window = true;
      m_timer = System.currentTimeMillis();
    }
    else if (m_joystick.getBButton()) {
      desiredRotations = 0.0;
      motor_homing_enabled = false;
    }


    if ((System.currentTimeMillis() - m_timer) > 0.1) {
      within_time_window = false;
    }

    if (motor_homing_enabled == true) {
      motor_homing(true, 0.03, 0.8);
    }
    else {
      //my_KrakenX60_Motor.setControl(m_positionVoltage.withOverrideBrakeDurNeutral(true));
    }
*/
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(my_KrakenX60_Motor, 0.001);
  }

  @Override
  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
    if (Units.radiansToRotations(m_armSim.getAngleRads()) < Constants.top_limit) {
      m_mechanism.update(Units.radiansToDegrees(m_armSim.getAngleRads()), true);
    }
    else {
      m_mechanism.update(Units.radiansToDegrees(m_armSim.getAngleRads()), false);
    }
   
    m_armSim.setInputVoltage(my_KrakenX60_Motor.getMotorVoltage().getValueAsDouble());
    m_armSim.update(0.02);
  }

  public void motor_homing(boolean direction, double output_limit, double velocity_trigger) {
    if ((direction == true)&&(within_time_window)){
      my_KrakenX60_Motor.setControl(m_dutyCycle.withOutput(-1*output_limit));
    }
    else if ((direction == false)&&(within_time_window)){
      my_KrakenX60_Motor.setControl(m_dutyCycle.withOutput(output_limit));
    }
    else if (Math.abs(my_KrakenX60_Motor.getVelocity().getValueAsDouble()) < velocity_trigger) {
      my_KrakenX60_Motor.setPosition(0.0);
      my_KrakenX60_Motor.setControl(m_dutyCycle.withOutput(0.0));
      motor_homing_enabled = false;
    }
  }
}