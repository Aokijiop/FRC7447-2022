// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // Differential Drive
  MotorControllerGroup m_left;
  MotorControllerGroup m_right;

  WPI_VictorSPX m_leftFront;
  WPI_VictorSPX m_leftBack;
  WPI_VictorSPX m_rightFront;
  WPI_VictorSPX m_rightBack;

  DifferentialDrive m_drive;

  // Turn to Angle PID
  PIDController m_turnRightController;
  PIDController m_turnLeftController;
  AHRS m_gyro;
  double turnMeasurement;

  // Drive to Distance PID
  PIDController m_distanceController;
  Ultrasonic m_rangeFinder;
  float displacementY;  

  // Boost boolean
  public boolean RButtonHeld = false;

  // Encoders
  Encoder m_encoder;

  // Turn Controller Gains - TESTING GAINS - DO NOT DEPLOY. These will require tuning. Use the Ziegler-Nichols rule or the robot charatcerization tool.
  static final double kPtr = 0.0;
  static final double kItr = 0.0;
  static final double kDtr = 0.0;
  static final double kPtl = 0.0;
  static final double kItl = 0.0;
  static final double kDtl = 0.0;

  // Distance Controller Gains - TESTING GAINS - DO NOT DEPLOY. These might be the same or slightly different from the Turn Controller gains. Assume different for now.
  static final double kPd = 0.0;
  static final double kId = 0.0;
  static final double kDd = 0.0;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // PID Controllers
    m_turnRightController = new PIDController(kPtr, kItr, kDtr);
    m_turnLeftController = new PIDController(kPtl, kItl, kDtl);
    m_distanceController = new PIDController(kPd, kId, kDd);

    // Differential Drive
    m_leftFront = new WPI_VictorSPX(Constants.leftFront);
    m_leftBack = new WPI_VictorSPX(Constants.leftBack);
    m_rightFront = new WPI_VictorSPX(Constants.rightFront);
    m_rightBack = new WPI_VictorSPX(Constants.rightBack);

    m_left = new MotorControllerGroup(m_leftFront, m_leftBack);
    m_right = new MotorControllerGroup(m_rightFront, m_rightBack);
    m_right.setInverted(true);

    m_drive = new DifferentialDrive(m_left, m_right);

    m_gyro = new AHRS();

    // Ultrasonic
    m_rangeFinder = new Ultrasonic(Constants.pingChannel, Constants.echoChannel);
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Turn to Angle
  public void setTurnSetpoint(double setpoint) {
    m_turnRightController.setSetpoint(setpoint);
    m_turnLeftController.setSetpoint(setpoint);
  }

  public void updateTurnMeasurement() {
    turnMeasurement = m_gyro.getAngle();
    System.out.println("Updating angle measurement");
  }
  

  public boolean atTurnSetpoint() {
    return m_turnRightController.atSetpoint() && m_turnLeftController.atSetpoint();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void turnToAngle() {
    m_left.setVoltage(m_turnLeftController.calculate(turnMeasurement, m_turnLeftController.getSetpoint()));
    m_right.setVoltage(m_turnRightController.calculate(turnMeasurement, m_turnRightController.getSetpoint()));
  }

  // Drive to Distance
  public void setDriveSetpoint(double drivesetpoint) {
    m_distanceController.setSetpoint(drivesetpoint);
  }  
  // Global variable Displacement X for use elsewhere

  public void updateMovementMeasurement() {
    displacementY = m_gyro.getDisplacementY();
    System.out.println("Update Movement Measurement");
  }

  public boolean atDistanceSetpoint(){
    return m_distanceController.atSetpoint();
  }

  public void resetDistance(){
    m_gyro.resetDisplacement();
  }

  public void driveToDistance() {
    m_drive.arcadeDrive(m_distanceController.calculate(displacementY, m_distanceController.getSetpoint()), 0);
  }

  // Odometry


  // Other Commands
  // Might need to manually add a negative sign later if invert doesn't work
  public void driveManually(Joystick joystick, double speed, double turnSpeed) {
    m_drive.arcadeDrive(-joystick.getRawAxis(Constants.y_axis) * speed, joystick.getRawAxis(Constants.z_axis) * turnSpeed); 
  }

  public void activateBoostBoolean() {
    RButtonHeld = true;
  }

  public void deactivateBoostBoolean() {
    RButtonHeld = false;
  }

  public void driveForward(double speed) {
    m_drive.arcadeDrive(speed, 0);
  }

  public void stop() {
    m_drive.stopMotor();
  }
}

/*
░░░░░░░░░░░░░░░░██████████████████
░░░░░░░░░░░░████░░░░░░░░░░░░░░░░░░████
░░░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░██
░░░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░██
░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██
░░░░░░░░██░░░░░░░░░░░░░░░░░░░░██████░░░░██ 
░░░░░░░░██░░░░░░░░░░░░░░░░░░░░██████░░░░██ 
░░░░░░░░██░░░░██████░░░░██░░░░██████░░░░██ 
░░░░░░░░░░██░░░░░░░░░░██████░░░░░░░░░░██ 
░░░░░░░░████░░██░░░░░░░░░░░░░░░░░░██░░████
░░░░░░░░██░░░░██████████████████████░░░░██
░░░░░░░░██░░░░░░██░░██░░██░░██░░██░░░░░░██ 
░░░░░░░░░░████░░░░██████████████░░░░████
░░░░░░░░██████████░░░░░░░░░░░░░░██████████
░░░░░░██░░██████████████████████████████░░██
░░░░████░░██░░░░██░░░░░░██░░░░░░██░░░░██░░████ 
░░░░██░░░░░░██░░░░██████░░██████░░░░██░░░░░░██
░░██░░░░████░░██████░░░░██░░░░██████░░████░░░░██
░░██░░░░░░░░██░░░░██░░░░░░░░░░██░░░░██░░░░░░░░██ 
░░██░░░░░░░░░░██░░██░░░░░░░░░░██░░██░░░░░░░░░░██ 
░░░░██░░░░░░██░░░░████░░░░░░████░░░░██░░░░░░██ 
░░░░░░████░░██░░░░██░░░░░░░░░░██░░░░██░░████ 
░░░░░░░░██████░░░░██████████████░░░░██████ 
░░░░░░░░░░████░░░░██████████████░░░░████
░░░░░░░░██████████████████████████████████
░░░░░░░░████████████████░░████████████████
░░░░░░░░░░████████████░░░░░░████████████ 
░░░░░░██████░░░░░░░░██░░░░░░██░░░░░░░░██████
░░░░░░██░░░░░░░░░░████░░░░░░████░░░░░░░░░░██
░░░░░░░░██████████░░░░░░░░░░░░░░██████████
*/

// snas !!!