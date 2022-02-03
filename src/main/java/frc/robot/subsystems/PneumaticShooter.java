// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticShooter extends SubsystemBase {
  DoubleSolenoid m_solenoidOne;
  DoubleSolenoid m_solenoidTwo;
  DoubleSolenoid m_solenoidThree;

  /** Creates a new PneumaticShooter. */
  public PneumaticShooter() {
    m_solenoidOne = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.solOneForward, Constants.solOneReverse);
    m_solenoidOne.set(Value.kReverse);
    m_solenoidTwo = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.solTwoForward, Constants.solTwoReverse);
    m_solenoidTwo.set(Value.kReverse);
    m_solenoidThree = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.solThreeForward, Constants.solThreeReverse);
    m_solenoidThree.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void pneumaticShoot() {
    m_solenoidOne.set(Value.kForward);
    m_solenoidTwo.set(Value.kForward);
    m_solenoidThree.set(Value.kForward);
  }

  public void reset() {
    m_solenoidOne.set(Value.kReverse);
    m_solenoidTwo.set(Value.kReverse);
    m_solenoidThree.set(Value.kReverse);
  }
}
