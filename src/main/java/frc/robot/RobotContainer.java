// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Launcher;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_xbox1 = new XboxController(OIConstants.xbox1_port);
  private final XboxController m_xbox2 = new XboxController(OIConstants.xbox2_port);

  // The robot's subsystems and commands are defined here...
  private final IntakeWheels m_intakeWheels = new IntakeWheels();
  private final IntakeArm m_intakeArm = new IntakeArm(
      () -> m_xbox2.getLeftTriggerAxis(), () -> m_xbox2.getRightTriggerAxis());
  private final Indexer m_indexer = new Indexer();
  private final Launcher m_launcher = new Launcher(() -> m_xbox2.getPOV());
  private final ClimberElevator m_elevator = new ClimberElevator();
  private final DriveTrain m_drive = new DriveTrain();

  private final AutoCommands m_autoCommands = 
    new AutoCommands(m_drive, m_intakeWheels, m_intakeArm, m_indexer, m_launcher);
  
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Start the camera server
    // Note: CameraServer does not work in Simulation mode.
    if (Robot.isReal())
      CameraServer.startAutomaticCapture();

    // Configure the default commands
    m_drive.setDefaultCommand( new RunCommand(
      () -> m_drive.arcadeDrive(-m_xbox1.getLeftY()/2, m_xbox1.getRightX()/3), m_drive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_xbox2, Button.kLeftBumper.value)
      .whenPressed(() -> m_intakeWheels.pullInCargo())
      .whenReleased(() -> m_intakeWheels.stopIntakeWheels());
    
    new JoystickButton(m_xbox2, Button.kRightBumper.value)
      .whenPressed(() -> m_intakeWheels.pushOutCargo())
      .whenReleased(() -> m_intakeWheels.stopIntakeWheels());
  
    
    new JoystickButton(m_xbox2, Button.kX.value)
      .whenPressed(() -> m_indexer.feedToLauncher())
      .whenReleased(() -> m_indexer.stopIndexer());
    new JoystickButton(m_xbox2, Button.kY.value)
      .whenPressed(() -> m_indexer.throwAwayToIntake())
      .whenReleased(() -> m_indexer.stopIndexer());
    
    new JoystickButton(m_xbox2, Button.kA.value)
      .whenPressed(() -> m_launcher.launch())
      .whenReleased(() -> m_launcher.stopLauncher());
    new JoystickButton(m_xbox2, Button.kB.value)
      .whenPressed(() -> m_launcher.unclogLauncher())
      .whenReleased(() -> m_launcher.stopLauncher());
    
    new JoystickButton(m_xbox1, Button.kA.value)
      .whenPressed(() -> m_drive.toggleMaxOutput());
    
    new JoystickButton(m_xbox1, Button.kX.value)
      .whenPressed(() -> m_elevator.raiseClimberElevator())
      .whenReleased(() -> m_elevator.stopClimberElevator());
    
    new JoystickButton(m_xbox1, Button.kY.value)
      .whenPressed(() -> m_elevator.lowerClimberElevator())
      .whenReleased(() -> m_elevator.stopClimberElevator());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return m_autoCommands.getAutoCommand_1();
  }
}
