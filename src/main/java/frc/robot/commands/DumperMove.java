// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;
import frc.robot.Constants;

public class DumperMove extends CommandBase {
  /** Creates a new DumperRaiseLower. */
  Dumper m_dumper;
  private boolean finish;
  private boolean armUp;
  private double angleSetpoint = 0.0f;


  public DumperMove(Dumper d) {
    // Use addRequirements() here to declare subsystem dependencies.
    armUp = true;
    m_dumper = d;
    addRequirements(m_dumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    System.out.println("initialized move");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // moves dumper up
    if(!armUp) {
        if (m_dumper.getPosition() >= angleSetpoint) {
            finish = true;
            armUp = true;
          }
          else {
            m_dumper.moveArm(Constants.dumperUpSpeed);
            System.out.println("Moving upppp");
          }
    }
    //moves dumper down
    if(armUp) {
        if (m_dumper.getPosition() <= -375.0f) {
            finish = true;
            armUp = false;
          }
          else {
            m_dumper.moveArm(Constants.dumperDownSpeed);
            System.out.println("moving downnnnnn!!");
          }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dumper.stopArm();
    System.out.println("Finished!!");
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

