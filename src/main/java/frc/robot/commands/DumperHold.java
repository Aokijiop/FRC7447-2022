// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;
import frc.robot.Constants;

public class DumperHold extends CommandBase {
  Dumper m_dumper;
  
  /** Creates a new DumperHold. */
  public DumperHold(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dumper = d;
    addRequirements(m_dumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_dumper.armIsUp()){
    //   m_dumper.moveArm(Constants.dumperHoldUpSpeed); 
    // }
    if(!m_dumper.armIsUp()){
      m_dumper.moveArm(Constants.dumperHoldDownSpeed);
    }

    m_dumper.intakeOuttake(0.15f);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
