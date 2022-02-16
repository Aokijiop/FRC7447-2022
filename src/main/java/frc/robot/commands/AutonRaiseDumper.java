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
  DigitalInput bottomlimitSwitch;

  /** Creates a new AutonDumper. */

  Dumper m_autonDumper;

  //I'm confused as to what dumperVoltage is supposed to measure...   
  
  public AutonRaiseDumper(Dumper a) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_autonDumper = a;
    addRequirements(m_autonDumper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    


    // if (dumperVoltage > 0) {
    //     if (toplimitSwitch.get()) {
    //         // We are going up and top limit is tripped so stop
    //         m_autonDumper.stopElevationDumper();
    //     } else {
    //         // We are going up but top limit is not tripped so go at commanded speed
    //         m_autonDumper.dumperMove(Constants.dumperDownSpeed);
    //     }
    // } else {
    //     if (bottomlimitSwitch.get()) {
    //         // We are going down and bottom limit is tripped so stop
    //         m_autonDumper.stopElevationDumper();
    //     } else {
    //         // We are going down but bottom limit is not tripped so go at commanded speed
    //         m_autonDumper.dumperMove(Constants.dumperDownSpeed);
    //     }
    // }
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
