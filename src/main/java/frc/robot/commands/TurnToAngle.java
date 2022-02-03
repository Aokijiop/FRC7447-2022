// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain m_driveTrain;
  private boolean finish;
  private float angle;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain dt, float a) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;
    angle = a;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
    m_driveTrain.deactivatePID();
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.activatePID();
    while (m_driveTrain.PIDActivated) {
      if (angle == 180.0f) {
        m_driveTrain.setTheSetpoint(180.0f);
      }

      if (angle == 90.0f) {
        m_driveTrain.setTheSetpoint(90.0f);
      }
    }
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.deactivatePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
