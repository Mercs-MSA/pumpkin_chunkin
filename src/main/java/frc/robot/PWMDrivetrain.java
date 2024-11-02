// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via PWM. If using SPARK MAX
 * controllers connected to CAN, go to RobotContainer and comment out the line declaring this subsystem and uncomment
 * the line for the CANDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class PWMDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public PWMDrivetrain() {
    /*Create MotorControllerGroups for each side of the drivetrain. These are declared here, and not at the class level
     * as we will not need to reference them directly anymore after we put them into a DifferentialDrive.
     */

    PWMTalonFX leftFrontMotor = new PWMTalonFX(21);
    PWMTalonFX leftRearMotor = new PWMTalonFX(23);
    PWMTalonFX rightFrontMotor = new PWMTalonFX(20);
    PWMTalonFX rightRearMotor = new PWMTalonFX(22);

    leftFrontMotor.addFollower(leftRearMotor);
    leftRearMotor.setInverted(true);
    leftFrontMotor.setInverted(false);
    rightFrontMotor.addFollower(rightRearMotor);
    rightRearMotor.setInverted(true);
    rightFrontMotor.setInverted(false);

    // Put our controller groups into a DifferentialDrive object. This object represents all 4 motor
    // controllers in the drivetrain
    m_drivetrain = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}
