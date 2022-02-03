// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticShooter;

public class PneumaticShoot extends CommandBase {
  PneumaticShooter m_pneumaticShooter;

  /** Creates a new PneumaticShoot. */
  public PneumaticShoot(PneumaticShooter p) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pneumaticShooter = p;
    addRequirements(m_pneumaticShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pneumaticShooter.pneumaticShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pneumaticShooter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
