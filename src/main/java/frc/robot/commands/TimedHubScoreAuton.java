// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Dumper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimedHubScoreAuton extends SequentialCommandGroup {
  Wait m_wait;

  /** Creates a new TimerTerminalSideScoreAuton. */
  public TimedHubScoreAuton(Dumper d, DriveTrain dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_wait = new Wait(3.0f);
    addCommands(new DumperVomit(d).raceWith(m_wait), new DumperMove(d), new DriveTimed(dt, -0.3f, 5.0f));
  }
}
