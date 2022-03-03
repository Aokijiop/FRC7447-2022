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
public class TimedRightBackScoreAuton extends SequentialCommandGroup {
  Wait m_wait;

  public TimedRightBackScoreAuton(DriveTrain dt, Dumper d) {
    m_wait = new Wait(5.0f);         // creates a new wait object

    addCommands(new DriveTimed(dt, SOMESPEED, SOMETIME), new TurnTimed(dt, SOMESPEED, SOMETIME), new DumperVomit(d).raceWith(m_wait), new DriveTimed(dt, SOMESPEED, SOMETIME));
    // drives to center from back of tarmac, dumps ball into center, drives off of tarmac from center
    // static values are not definite; just random bullshit lol
  }
}