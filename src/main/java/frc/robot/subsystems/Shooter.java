// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  WPI_VictorSPX m_shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotor = new WPI_VictorSPX(Constants.shooterM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shootBall(double percentVoltage) {
    m_shooterMotor.set(ControlMode.PercentOutput, percentVoltage);
  }

  public void stop() {
    m_shooterMotor.set(0);
  }
}
