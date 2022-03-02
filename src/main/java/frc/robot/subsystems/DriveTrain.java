// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  PIDController m_turnController;
  AHRS m_gyro;
  double turnMeasurement;

  // Drive to Distance PID (All units in meters)
  PIDController m_leftDistanceController;
  PIDController m_rightDistanceController;
  double leftDisplacement;  
  double rightDisplacement;
  double wheelRadius = 0.0762f;
  double encoderResolution = 360f;

  // Boost Boolean
  public boolean RButtonHeld = false;

  // TTA Feedforward
  SimpleMotorFeedforward m_turnFeedforward; 

  // DTD Feedforward
  SimpleMotorFeedforward m_leftDTDFeedforward;
  SimpleMotorFeedforward m_rightDTDFeedforward;
  
  // Encoders
  Encoder m_leftEncoder;
  Encoder m_rightEncoder;

  // Turn Controller Gains - TESTING GAINS - DO NOT DEPLOY. These will require tuning. Use the Ziegler-Nichols rule or the robot charatcerization tool.
  static final double kPt = 0.0;
  static final double kIt = 0.0;
  static final double kDt = 0.0;
  
  // Turn Feedforward Gains
  static final double kSt = 0.0;
  static final double kVt = 0.0;

  // Left DTT Feedfoward Gains
  static final double kSld = 0.0;
  static final double kVld = 0.0;

  // Right DTT Feedfoward Gains
  static final double kSrd = 0.0;
  static final double kVrd = 0.0;

  // Left Distance Controller Gains - TESTING GAINS - DO NOT DEPLOY. These will require tuning. Use the Ziegler-Nichols rule or the robot charatcerization tool.
  static final double kPld = 0.0;
  static final double kIld = 0.0;
  static final double kDld = 0.0;

  // Right Distance Controller Gains - TESTING GAINS - DO NOT DEPLOY. These will require tuning. Use the Ziegler-Nichols rule or the robot charatcerization tool.
  static final double kPrd = 0.0;
  static final double kIrd = 0.0;
  static final double kDrd = 0.0;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // PID Controllers
    m_leftDistanceController = new PIDController(kPld, kIld, kDld);
    m_rightDistanceController = new PIDController(kPrd, kIrd, kDrd);
    m_turnController = new PIDController(kPt, kIt, kDt);

    // Feedforward Controllers
    m_turnFeedforward = new SimpleMotorFeedforward(kSt, kVt);
    m_rightDTDFeedforward = new SimpleMotorFeedforward(kSrd, kVrd);
    m_leftDTDFeedforward = new SimpleMotorFeedforward(kSld, kVld);

    // Differential Drive
    m_leftFront = new WPI_VictorSPX(Constants.leftFront);
    m_leftFront.configOpenloopRamp(0.5);
    m_leftFront.configVoltageCompSaturation(12.0);
    m_leftFront.enableVoltageCompensation(true);
    m_leftBack = new WPI_VictorSPX(Constants.leftBack); 
    m_leftBack.configOpenloopRamp(0.5);
    m_leftBack.configVoltageCompSaturation(12.0);
    m_leftBack.enableVoltageCompensation(true);
    m_rightFront = new WPI_VictorSPX(Constants.rightFront);
    m_rightFront.configOpenloopRamp(0.5);
    m_rightFront.configVoltageCompSaturation(12.0);
    m_rightFront.enableVoltageCompensation(true);
    m_rightBack = new WPI_VictorSPX(Constants.rightBack);
    m_rightBack.configOpenloopRamp(0.5);
    m_rightBack.configVoltageCompSaturation(12.0);
    m_rightBack.enableVoltageCompensation(true);

    m_left = new MotorControllerGroup(m_leftFront, m_leftBack);
    m_right = new MotorControllerGroup(m_rightFront, m_rightBack);
    m_right.setInverted(true);

    m_drive = new DifferentialDrive(m_left, m_right);

    m_gyro = new AHRS();

    // Encoders (All units in meters)
    m_leftEncoder = new Encoder(Constants.leftEncoderA, Constants.leftEncoderB, false, Encoder.EncodingType.k2X);
    m_leftEncoder.setDistancePerPulse((2 * Math.PI * wheelRadius)/encoderResolution);
    m_rightEncoder = new Encoder(Constants.rightEncoderA, Constants.rightEncoderB, true, Encoder.EncodingType.k2X);
    m_rightEncoder.setDistancePerPulse((2 * Math.PI * wheelRadius)/encoderResolution);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Turn to Angle
  public void setTurnSetpoint(double setpoint) {
    m_turnController.setSetpoint(setpoint);
  }

  public void updateTurnMeasurement() {
    turnMeasurement = m_gyro.getAngle();
    System.out.println("Updating angle measurement");
  }
  
  public boolean atTurnSetpoint() {
    return m_turnController.atSetpoint();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void turnToAngle() {
    m_drive.arcadeDrive(0, m_turnController.calculate(turnMeasurement, m_turnController.getSetpoint()));
  }

  // Drive to Distance
  public void setDriveSetpoint(double drivesetpoint) {
    m_leftDistanceController.setSetpoint(drivesetpoint);
    m_rightDistanceController.setSetpoint(drivesetpoint);
  }  

  public void updateMovementMeasurement() {
    leftDisplacement = m_leftEncoder.getDistance();
    rightDisplacement = m_rightEncoder.getDistance();
    System.out.println("Update Movement Measurement");
  }

  public void printMovementMeasurement() {
    System.out.println("Left: " + m_leftEncoder.getDistance());
    System.out.println("Right: " + m_rightEncoder.getDistance());
  }

  public boolean atLeftDistanceSetpoint() {
    return m_leftDistanceController.atSetpoint();
  }

  public boolean atRightDistanceSetpoint() {
    return m_rightDistanceController.atSetpoint();
  }

  public void resetDistance(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void driveToDistance() {
    m_left.setVoltage(m_leftDistanceController.calculate(leftDisplacement, m_leftDistanceController.getSetpoint()) + m_turnController.calculate(turnMeasurement, m_turnController.getSetpoint()));
    m_right.setVoltage(m_rightDistanceController.calculate(rightDisplacement, m_rightDistanceController.getSetpoint()) - m_turnController.calculate(turnMeasurement, m_turnController.getSetpoint()));
  }

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

  public void driveTimed(double speed) {
    m_drive.arcadeDrive(speed, 0);
  }

  public void turnTimed(double angle) {
    m_drive.arcadeDrive(0, angle);
  }

  public void stop() {
    m_drive.stopMotor();
  }
}

/*
░░░░░░░░░░░░░░░░██████████████████
░░░░░░░░░░░░████░░░░░░░░░░░░░░░░░░█░██
░░░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░██
░░░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░██
░░░░░░░░██░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░██
░░░░░░░░██░░░░░░░░░░░░░░░░░░░░██████░░░░██ 
░░░░░░░░██░░░░░░░░░░░░░░░░░░░░██████░░░░██ 
░░░░░░░░██░░░░██████░░░░██░░░░██████░░░░██ 
░░░░░░░░░░██░░░░░░░░░░██████░░░░░░░░░░██ 
░░░░░░░░████░░██░░░░░░░░░░░░░░░░░░██░░████
░░░░░░░░██░░░░██████████████████████░░░░██
░░░░░░░░██░░░░░░██░░██░░██░░██░░░░░░░█████
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