// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dumper extends SubsystemBase {
  /** Creates a new Dumper. */
  WPI_VictorSPX m_dumperIntakeOuttake;
  CANSparkMax m_dumperArm;


  public Dumper() {
    m_dumperIntakeOuttake = new WPI_VictorSPX(Constants.dumperPort);
    m_dumperArm = new CANSparkMax(Constants.dumperMovePort, MotorType.kBrushless);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeOuttake(double dumperVoltage) {
    m_dumperIntakeOuttake.set(ControlMode.PercentOutput, dumperVoltage);
  }

  public void moveArm(double dumperUpSpeed){
    m_dumperArm.setVoltage(dumperUpSpeed);
  }

  public void stopIntakeOuttake() {
    m_dumperIntakeOuttake.stopMotor();
  }

  public void stopArm() {
    m_dumperArm.stopMotor();
  }
}