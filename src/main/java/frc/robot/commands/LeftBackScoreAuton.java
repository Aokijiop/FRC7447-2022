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
public class LeftBackScoreAuton extends SequentialCommandGroup {
  Wait m_wait;

  public LeftBackScoreAuton(DriveTrain dt, Dumper d) {
    m_wait = new Wait(5.0f);         // creates a new wait object

    addCommands(new DriveToDistance(dt, 1.80f), new TurnToAngle(dt, -15.0f), new DumperVomit(d).raceWith(m_wait), new DriveToDistance(dt, -4.0f));
    // drives to center from back of tarmac, dumps ball into center, drives off of tarmac from center
    // static values are not definite; just random bullshit lol
    // this is legit a copy-paste of RightBackScoreAuton lmfaooo
  }
}
