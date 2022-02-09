// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dumper extends SubsystemBase {
  /** Creates a new Dumper. */
  WPI_VictorSPX m_dumperMotor;
  CANSparkMax m_dumperMoveMotor;


  public Dumper() {
    m_dumperMotor = new WPI_VictorSPX(Constants.dumperPort);
    m_dumperMoveMotor = new CANSparkMax(Constants.dumperMovePort, null);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void dumperIntake(double dumperVoltage) {
    m_dumperMotor.set(ControlMode.PercentOutput, dumperVoltage);
  }

  public void dumperVomit(double dumperRetch){
    m_dumperMotor.set(ControlMode.PercentOutput, dumperRetch);
  }

  public void dumperMove(double dumperUpSpeed){
    m_dumperMoveMotor.set(dumperUpSpeed);
  }


  public void stopDumper() {
    m_dumperMotor.stopMotor();
  }
}