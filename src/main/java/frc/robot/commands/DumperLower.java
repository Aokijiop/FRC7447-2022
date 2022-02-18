// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;
import frc.robot.Constants;

public class DumperLower extends CommandBase {
  /** Creates a new DumperMoveDown. */
  Dumper m_dumper;
  private double angleTarget = -375.0f;
  private boolean finish;

  public DumperLower(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumper = d;
    addRequirements(m_dumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dumper.moveArm(Constants.dumperDownSpeed);

    if (m_dumper.getPosition() <= angleTarget) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dumper.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
