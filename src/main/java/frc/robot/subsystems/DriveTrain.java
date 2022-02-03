// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  MotorControllerGroup m_left;
  MotorControllerGroup m_right;

  WPI_VictorSPX m_leftFront;
  WPI_VictorSPX m_leftBack;
  WPI_VictorSPX m_rightFront;
  WPI_VictorSPX m_rightBack;

  DifferentialDrive m_drive;

  AHRS m_gyro;
  PIDController m_turnController;
  public boolean PIDActivated = false;

  SimpleMotorFeedforward m_feedForward;

  Ultrasonic m_rangeFinder;

  // TESTING GAINS - DO NOT DEPLOY. Constants for PID Gains. These will require tuning. Use the Ziegler-Nichols rule or the robot charatcerization tool.
  static final double kP = 0.5;
  static final double kI = 0.5;
  static final double kD = 0.5;

  // Constants for Feedforward Gains
  static final double kS = 0.3;
  static final double kV = 0.1;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_leftFront = new WPI_VictorSPX(Constants.leftFront);
    // m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack = new WPI_VictorSPX(Constants.leftBack);
    // m_leftBack.setNeutralMode(NeutralMode.Brake);
    m_rightFront = new WPI_VictorSPX(Constants.rightFront);
    // m_rightFront.setNeutralMode(NeutralMode.Coast);
    m_rightBack = new WPI_VictorSPX(Constants.rightBack);
    // m_rightBack.setNeutralMode(NeutralMode.Coast);

    m_left = new MotorControllerGroup(m_leftFront, m_leftBack);

    m_right = new MotorControllerGroup(m_rightFront, m_rightBack);

    m_right.setInverted(true);

    m_drive = new DifferentialDrive(m_left, m_right);

    m_gyro = new AHRS();

    m_turnController = new PIDController(kP, kI, kD);

    m_feedForward = new SimpleMotorFeedforward(kS, kV);

    m_rangeFinder = new Ultrasonic(0, 1);
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    while (PIDActivated) {
      turnToAngle(m_gyro.getAngle(), m_turnController.getSetpoint());

      if (m_gyro.getAngle() == m_turnController.getSetpoint()) {
        deactivatePID();
      }
    }
  }

  // Might need to manually add a negative sign later if invert doesn't work
  public void driveManually(Joystick joystick, double speed, double turnSpeed) {
    m_drive.arcadeDrive(-joystick.getRawAxis(Constants.y_axis) * speed, joystick.getRawAxis(Constants.z_axis) * turnSpeed);
  }

  public void driveForward(double speed) {
    m_drive.arcadeDrive(speed, 0);
  }

  public void turnToAngle(double currentAngle, double angleSetpoint) {
    m_drive.arcadeDrive(0, m_turnController.calculate(currentAngle, angleSetpoint));
    // m_left.setVoltage(m_feedForward.calculate(speed) + m_turnController.calculate(m_gyro.getAngle(), angleSetpoint));
    // m_right.setVoltage(m_feedForward.calculate(speed) + m_turnController.calculate(m_gyro.getAngle(), angleSetpoint));
  }

  public void setTheSetpoint(float angleSetpoint) {
    m_turnController.setSetpoint(angleSetpoint);
  }

  public void activatePID() {
    PIDActivated = true;
  }

  public void deactivatePID() {
    PIDActivated = false;
  }

  public void resetGyro() {
    m_gyro.reset();
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