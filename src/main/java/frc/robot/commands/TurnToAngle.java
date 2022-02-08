// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain m_driveTrain;
  private boolean finish;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetGyro();
    finish = false;
    m_driveTrain.setTurnSetpoint(180.0f);
    System.out.println("Command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.updateTurnMeasurement();
    m_driveTrain.turnToAngle();
    System.out.println("Command executing");

    if (m_driveTrain.atTurnSetpoint()) {
      finish = true;
      System.out.println("Setpoint reached");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ending");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}