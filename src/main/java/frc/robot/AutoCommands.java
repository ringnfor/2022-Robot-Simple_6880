// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Launcher;

/** Add your docs here. */
public class AutoCommands {
    DriveTrain m_drvSys;
    IntakeWheels m_intakeWheels;
    IntakeArm m_intakeArm;
    Indexer m_indexer;
    Launcher m_launcher;
    public AutoCommands(DriveTrain drvSys, IntakeWheels intakeWheels, 
        IntakeArm intakeArm, Indexer indexer, Launcher launcher) {
        m_drvSys = drvSys;
        m_intakeWheels = intakeWheels;
        m_intakeArm = intakeArm;
        m_indexer = indexer;
        m_launcher = launcher;
    }

    public Command getAutoCommand_1() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveTrainConstants.ksVolts,
                DriveTrainConstants.kvVoltSecondsPerMeter,
                DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
            DriveTrainConstants.kDriveKinematics,
            10);

      // Create config for trajectory
      TrajectoryConfig config =
        new TrajectoryConfig(
          DriveTrainConstants.kMaxSpeed_m_s,
          DriveTrainConstants.kMaxAccel_m_ss)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveTrainConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

          // An example trajectory to follow.  All units in meters.
      Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

      RamseteCommand ramseteCommand =
      new RamseteCommand(
          exampleTrajectory,
          m_drvSys::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
              DriveTrainConstants.ksVolts,
              DriveTrainConstants.kvVoltSecondsPerMeter,
              DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
          DriveTrainConstants.kDriveKinematics,
          m_drvSys::getWheelSpeeds,
          new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
          new PIDController(DriveTrainConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_drvSys::tankDriveVolts,
          m_drvSys);

      // Reset odometry to the starting pose of the trajectory.
      m_drvSys.resetOdometry(exampleTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_drvSys.tankDriveVolts(0, 0));

    }
}
