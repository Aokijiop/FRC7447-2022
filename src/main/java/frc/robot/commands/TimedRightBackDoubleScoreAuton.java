// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Dumper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedRightBackDoubleScoreAuton extends SequentialCommandGroup {
  Wait m_wait;

  /** Creates a new TerminalSideScoreAuton. */
  public TimedRightBackDoubleScoreAuton (DriveTrain dt, Dumper d) { 
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_wait = new Wait(3.0f);
    addCommands(new ParallelCommandGroup(new DriveTimed(dt, SOMESPEED, SOMETIME), new DumperMove(d), new DumperIntake(d).raceWith(new Wait(2.5f))), new TurnTimed(dt, SOMESPEED, SOMETIME), new DriveTimed(dt, SOMESPEED, SOMETIME), new ParallelCommandGroup(new TurnTimed(dt, SOMESPEED, SOMETIME), new DumperMove(d)), new DumperVomit(d).raceWith(new Wait(3.0f)), new TurnTimed(dt, SOMESPEED, SOMETIME), new DriveTimed(dt, SOMESPEED, SOMETIME));
  }
}