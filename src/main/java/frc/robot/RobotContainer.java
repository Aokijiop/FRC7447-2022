// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutonOne;
import frc.robot.commands.AutonTwo;
import frc.robot.commands.DriveForwardTimed;
import frc.robot.commands.DriveManually;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.DumperIntake;
import frc.robot.commands.DumperVomit;
import frc.robot.commands.PneumaticShoot;
import frc.robot.commands.ShootBall;
// import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Dumper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticShooter;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final PneumaticShooter m_pneumaticShooter;
  private final Dumper m_dumper;

  // Commands
  private final DriveManually m_driveManually;
  private final DriveForwardTimed m_driveForwardTimed;
  private final ShootBall m_shootBall;
  private final IntakeBall m_intakeBall;
  private final AutoShoot m_autoShoot;


  private final DumperIntake m_dumperIntake;
  private final DumperVomit m_dumperVomit;


  private final PneumaticShoot m_pneumaticShoot;
  // private final TurnToAngle m_turnTo180;
  // private final TurnToAngle m_turnTo90;

  // Autonomous Commands
  private final AutonOne m_autonOne;
  private final AutonTwo m_autonTwo;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems
    m_driveTrain = new DriveTrain();
    m_shooter = new Shooter();
    m_intake = new Intake();
    m_pneumaticShooter = new PneumaticShooter();
    m_dumper = new Dumper();
    

    // Commands
    m_driveManually = new DriveManually(m_driveTrain);
    m_driveManually.addRequirements(m_driveTrain);
    m_driveTrain.setDefaultCommand(m_driveManually);
    m_driveForwardTimed = new DriveForwardTimed(m_driveTrain);
    m_driveForwardTimed.addRequirements(m_driveTrain);
    m_shootBall = new ShootBall(m_shooter);
    m_shootBall.addRequirements(m_shooter);
    m_intakeBall = new IntakeBall(m_intake);
    m_intakeBall.addRequirements(m_intake);
    m_autoShoot = new AutoShoot(m_shooter);
    m_autoShoot.addRequirements(m_shooter);
    m_pneumaticShoot = new PneumaticShoot(m_pneumaticShooter);
    m_pneumaticShoot.addRequirements(m_pneumaticShooter);
    m_dumperIntake = new DumperIntake(m_dumper);
    m_dumperVomit = new DumperVomit(m_dumper);

    // Autonomous Commands
    m_autonOne = new AutonOne(m_driveTrain, m_shooter);
    m_autonTwo = new AutonTwo(m_driveTrain, m_shooter);
    
    // Autonomous Command Chooser
    m_chooser.setDefaultOption("AutonOne", m_autonOne);
    m_chooser.addOption("AutonTwo", m_autonTwo);
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
    RTrigger.whenHeld(m_shootBall);
    LTrigger.whenHeld(m_intakeBall);
    bButton.toggleWhenPressed(m_driveForwardTimed, false);
    aButton.whenHeld(m_pneumaticShoot);
    LButton.whenHeld(m_dumperIntake);
    RButton.whenHeld(m_dumperVomit);


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
