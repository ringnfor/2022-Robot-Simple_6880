// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBackupAndShoot extends SequentialCommandGroup {
  /** Creates a new AutoBackupAndShoot. */
  public AutoBackupAndShoot(DriveTrain drive, Indexer index, Launcher launch, IntakeArm arm) {
    addCommands(
      new DriveDistance(Units.inchesToMeters(92), -0.4, drive),
      new InstantCommand(
        () -> launch.launch(), launch),
      new OpenArm(arm),
      new WaitCommand(2),
      new InstantCommand(
        () -> index.feedToLauncher(), index),
      new WaitCommand(2),
      new InstantCommand(
        () -> launch.stopLauncher(), launch),
      new InstantCommand(
        () -> index.stopIndexer(), index)
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
