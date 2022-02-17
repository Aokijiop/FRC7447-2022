// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Dumper;

public class AutonLowerDumper extends CommandBase {

  DigitalInput bottomlimitSwitch;
  
    /** Creates a new AutonLowerDumper. */

  Dumper m_autonlowerDumper;
  private boolean finish;

  public AutonLowerDumper(Dumper l) {
    m_autonlowerDumper = l;
    addRequirements(m_autonlowerDumper);
  }    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_autonlowerDumper.moveArm(-0.5);

    if (bottomlimitSwitch.get()) {
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_autonlowerDumper.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
