// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dumper extends SubsystemBase {
  /** Creates a new Dumper. */
  WPI_VictorSPX m_dumperMotor;

  public Dumper() {
    m_dumperMotor = new WPI_VictorSPX(Constants.dumperPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveDumper(double dumperVoltage) {
    m_dumperMotor.set(ControlMode.PercentOutput, dumperVoltage);
  }

  public void stopDumper() {
    m_dumperMotor.stopMotor();
  }

}
