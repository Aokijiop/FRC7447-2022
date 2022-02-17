// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Dumper;

public class AutonRaiseDumper extends CommandBase {

  DigitalInput toplimitSwitch;

  /** Creates a new AutonDumper. */

  Dumper m_autonraiseDumper;
  private boolean finish;

  //I'm confused as to what dumperVoltage is supposed to measure...   
  
  public AutonRaiseDumper(Dumper a) {
    m_autonraiseDumper = a;
    addRequirements(m_autonraiseDumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_autonraiseDumper.moveArm(Constants.dumperUpSpeed);

    if (toplimitSwitch.get()) {
      finish = true;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_autonraiseDumper.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
