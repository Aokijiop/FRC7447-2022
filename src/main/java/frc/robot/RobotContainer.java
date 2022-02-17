// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SeqComGroupEx;
import frc.robot.commands.BoostBoolean;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveManually;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.DumperIntake;
import frc.robot.commands.DumperLower;
import frc.robot.commands.DumperRaise;
import frc.robot.commands.DumperVomit;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Dumper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Subsystems
  private final DriveTrain m_driveTrain;
  private final Dumper m_dumper;

  // Commands
  private final DriveManually m_driveManually;
  private final DriveForwardTimed m_driveForwardTimed;
  private final DumperIntake m_dumperIntake;
  private final DumperVomit m_dumperVomit;
  private final DumperRaise m_dumperMoveMotorUp;
  private final DumperLower m_dumperMoveMotorDown;
  private final BoostBoolean m_boost;
  private final DriveToDistance m_driveTo10;

  // Turn to Angle Commands
  private final TurnToAngle m_cancelTurnTo;
  private final TurnToAngle m_turnTo45;
  private final TurnToAngle m_turnTo90;
  private final TurnToAngle m_turnTo135;
  private final TurnToAngle m_turnTo180;
  private final TurnToAngle m_turnTo45cc;
  private final TurnToAngle m_turnTo90cc;
  private final TurnToAngle m_turnTo135cc;

  // Autonomous Commands
  private final SeqComGroupEx m_autonOne;

  // Autonomous Command Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controller
  public static Joystick m_joystick;
  public static JoystickButton xButton;
  public static JoystickButton aButton;
  public static JoystickButton bButton;
  public static JoystickButton LTrigger;
  public static JoystickButton RTrigger;
  public static JoystickButton yButton;
  public static JoystickButton LButton;
  public static JoystickButton RButton;

  // POV Buttons
  public static POVButton pov0;
  public static POVButton pov45;
  public static POVButton pov90;
  public static POVButton pov135;
  public static POVButton pov180;
  public static POVButton pov225;
  public static POVButton pov270;
  public static POVButton pov315;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    m_driveTrain = new DriveTrain();
    m_dumper = new Dumper();
    

    // Commands
    m_driveManually = new DriveManually(m_driveTrain);
    m_driveManually.addRequirements(m_driveTrain);
    m_driveTrain.setDefaultCommand(m_driveManually);
    m_driveForwardTimed = new DriveForwardTimed(m_driveTrain);
    m_driveForwardTimed.addRequirements(m_driveTrain);
    m_dumperIntake = new DumperIntake(m_dumper);
    m_dumperVomit = new DumperVomit(m_dumper);
    m_dumperMoveMotorDown = new DumperLower(m_dumper);
    m_dumperMoveMotorUp = new DumperRaise(m_dumper);
    m_boost = new BoostBoolean(m_driveTrain);
    m_driveTo10 = new DriveToDistance(m_driveTrain, 10.0);

    // Turn to Angle Commands
    m_cancelTurnTo = new TurnToAngle(m_driveTrain, 0.0f);
    m_turnTo45 = new TurnToAngle(m_driveTrain, 45.0f);
    m_turnTo90 = new TurnToAngle(m_driveTrain, 90.0f);
    m_turnTo135 = new TurnToAngle(m_driveTrain, 135.0f);
    m_turnTo180 = new TurnToAngle(m_driveTrain, 180.0f);
    m_turnTo45cc = new TurnToAngle(m_driveTrain, -45.0f);
    m_turnTo90cc = new TurnToAngle(m_driveTrain, -90.0f);
    m_turnTo135cc = new TurnToAngle(m_driveTrain, -135.0f);

    // Autonomous Commands
    m_autonOne = new SeqComGroupEx(m_driveTrain, m_dumper);
    
    // Autonomous Command Chooser
    m_chooser.setDefaultOption("AutonOne", m_autonOne);
    // m_chooser.addOption("AutonTwo", m_autonTwo);
    SmartDashboard.putData("Autonomous", m_chooser);

    // Controller
    m_joystick = new Joystick(Constants.joystickPort);
    xButton = new JoystickButton(m_joystick, Constants.xButton);
    aButton = new JoystickButton(m_joystick, Constants.aButton);
    bButton = new JoystickButton(m_joystick, Constants.bButton);
    yButton = new JoystickButton(m_joystick, Constants.yButton);
    LTrigger = new JoystickButton(m_joystick, Constants.LTrigger);
    RTrigger = new JoystickButton(m_joystick, Constants.RTrigger);
    LButton = new JoystickButton(m_joystick, Constants.LButton);
    RButton = new JoystickButton(m_joystick, Constants.RButton);

    // POV Buttons
    pov0 = new POVButton(m_joystick, 0, 0);
    pov45 = new POVButton(m_joystick, 45, 0);
    pov90 = new POVButton(m_joystick, 90, 0);
    pov135 = new POVButton(m_joystick, 135, 0);
    pov180 = new POVButton(m_joystick, 180, 0);
    pov225 = new POVButton(m_joystick, 225, 0);
    pov270 = new POVButton(m_joystick, 270, 0);
    pov315 = new POVButton(m_joystick, 315, 0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    bButton.toggleWhenPressed(m_driveForwardTimed, false);
    LTrigger.whenHeld(m_dumperIntake);
    RTrigger.whenHeld(m_dumperVomit);
    RButton.whenHeld(m_boost);
    aButton.toggleWhenPressed(m_dumperMoveMotorDown);
    yButton.toggleWhenPressed(m_dumperMoveMotorUp);
    pov0.toggleWhenPressed(m_cancelTurnTo);
    pov45.toggleWhenPressed(m_turnTo45);
    pov90.toggleWhenPressed(m_turnTo90);
    pov135.toggleWhenPressed(m_turnTo135);
    pov180.toggleWhenPressed(m_turnTo180);
    pov225.toggleWhenPressed(m_turnTo45cc);
    pov270.toggleWhenPressed(m_turnTo90cc);
    pov315.toggleWhenPressed(m_turnTo135cc);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
