// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final PowerDistribution m_PowerDistr = new PowerDistribution(0, ModuleType.kCTRE);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Implements PowerDistribution Module
    SmartDashboard.putNumber("Amps for Channel 0: ", m_PowerDistr.getCurrent(0));
    SmartDashboard.putNumber("Amps for Channel 1: ", m_PowerDistr.getCurrent(1));
    SmartDashboard.putNumber("Amps for Channel 2: ", m_PowerDistr.getCurrent(2));
    SmartDashboard.putNumber("Amps for Channel 3: ", m_PowerDistr.getCurrent(3));
    SmartDashboard.putNumber("Amps for Channel 4: ", m_PowerDistr.getCurrent(4));
    SmartDashboard.putNumber("Amps for Channel 5: ", m_PowerDistr.getCurrent(5));
    SmartDashboard.putNumber("Amps for Channel 6: ", m_PowerDistr.getCurrent(6));
    SmartDashboard.putNumber("Amps for Channel 7: ", m_PowerDistr.getCurrent(7));
    SmartDashboard.putNumber("Amps for Channel 8: ", m_PowerDistr.getCurrent(8));
    SmartDashboard.putNumber("Amps for Channel 9: ", m_PowerDistr.getCurrent(9));
    SmartDashboard.putNumber("Amps for Channel 10: ", m_PowerDistr.getCurrent(10));
    SmartDashboard.putNumber("Amps for Channel 11: ", m_PowerDistr.getCurrent(11));
    SmartDashboard.putNumber("Amps for Channel 12: ", m_PowerDistr.getCurrent(12));
    SmartDashboard.putNumber("Amps for Channel 13: ", m_PowerDistr.getCurrent(13));
    SmartDashboard.putNumber("Amps for Channel 14: ", m_PowerDistr.getCurrent(14));
    SmartDashboard.putNumber("Amps for Channel 15: ", m_PowerDistr.getCurrent(15));
    SmartDashboard.putNumber("Total Power from Monitored Channels: ", m_PowerDistr.getTotalPower());
    SmartDashboard.putNumber("Input Voltage of PDP: ", m_PowerDistr.getVoltage());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
