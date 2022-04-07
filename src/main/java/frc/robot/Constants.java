// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class CAN_IDs {
        public final static int left1_ID = 11;
        public final static int left2_ID = 12;
        public final static int right1_ID = 13;
        public final static int right2_ID = 14;
        public final static int intakeWheels_ID = 21;
        public final static int intakeArm_ID = 22;
        public final static int indexerFront_ID = 23;
        public final static int indexerBack_ID = 24;
        public final static int launcher1_ID = 25;
        public final static int launcher2_ID = 26;
        public final static int climber1_ID = 31;
        public final static int climber2_ID = 32;
    }
    public final static class EncoderConstants {
        public final static int[] leftEnc_ports = new int[]{0, 1};
        public final static boolean leftEnc_reversed = false;
        public final static int[] rightEnc_ports = new int[]{2, 3};
        public final static boolean rightEnc_reversed = true;
        public final static int[] intakeArmEnc_ports = new int[]{4,5};
        public final static boolean intakeArmEnc_reversed = false;
        public final static int[] climberEnc_ports = new int[]{8,9};
        public final static boolean climberEnc_reversed = false;
        public final static double revThroughboreEnc_PPR = 2048;
    }
    public final static class DriveTrainConstants {
        private final static double _gearRatio = 10.71;
        private final static double _wheelDiameter_m = Units.inchesToMeters(6);
        public final static double kWheelCircumference_m = Math.PI * _wheelDiameter_m;
        // ToDo: trackWidth in meters; to be calculated with sysID tool or measured with tape
        private final static double _trackWidth = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(_trackWidth);
        // ToDo: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // ToDo: Determine the Feedback constants from sysID tool
        //  These are the values for the SparkMax motor controller
        public static final double kPDriveVel = 8.5;
        public final static double kIDriveVel = 0.0;
        public final static double kDDriveVel = 0.0;

        public final static double kMaxSpeed_m_s = Units.feetToMeters(10); // Max velocity 10 ft per second
        public final static double kMaxAccel_m_ss = Units.feetToMeters(20); // Max acceleration 20 ft/s^2
    }
    public static final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    public final class IntakeWheelConstants {
        // The following fixed speed is applicable only if not using the FeedForward or Feedback controllers
        public final static double wheelMotorSpeed = 0.6;
    }
    public final static class IntakeArmConstants {
        // Arm Positions 
        private final static double _closedPosArmDegrees = 0; // Position of the Arm 1x1 tubing on the robot
        private final static double _openedPosArmDegrees = -105; // Position of the Arm 1x1 tubing on the robot
        public final static double kClosedPosArmRad = Units.degreesToRadians(_closedPosArmDegrees);
        public final static double kOpenedPosArmRad = Units.degreesToRadians(_openedPosArmDegrees);
        private final static double _gearRatio = (5*4*60/24); // 5:1 cartridge + 4:1 cartrdige + 60T:24T sprockets
        // The following two values in terms of Neo motor shaft rotations
        public final static double kClosedPosNeoRotations = Units.degreesToRotations(_closedPosArmDegrees * _gearRatio);
        public final static double kOpenedPosNeoRotations = Units.degreesToRotations(_openedPosArmDegrees * _gearRatio);

        // Arm position limits in units of Neo Motor shaft rotations
        // Add 10% leeway for setting the softlimit
        public final static float kForwardSoftlimit = (float)kClosedPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        public final static float kReverseSoftLimit = (float)kOpenedPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations

        // The following fixed speeds are applicable only if not using the FeedForward or Feedback controllers
        public final static double armMotorOpeningSpeed = -0.2;
        public final static double armMotorClosingSpeed = 0.3;
    }
    public final class IndexerConstants {
        public final static double indexerMotorSpeed = 0.5;
    }

    public final class LauncherConstants {
        public final static double launcherMotorSpeed = 0.9;
        public final static double launcherMotorHighSpeed = 0.95;
        public final static double launcherMotorLowSpeed = 0.7;  
    }

    public final static class ClimberElevatorConstants {
        // Elevator Position
        // Convention: _in=inches, _m=meters
        private final static double _downPos_in = 0;
        private final static double _upPos_in = 20; // ToDo: Verify
        public final static double kDownPos_m = Units.inchesToMeters(_downPos_in);
        public final static double kUpPos_m = Units.inchesToMeters(_upPos_in);
        private final static double _gearRatio = 20; // 5:1 cartridge + 4:1 cartrdige 
        private final static double _winchDia_in = 0.787; // For AndyMark Climber-in-a-box Winch kit 1-stage
        private final static double _winchDia_m = Units.inchesToMeters(_winchDia_in);
        private final static double _winchCircumference_m = _winchDia_m * Math.PI;
        public final static double KElevatorMetersToNeoRotationsFactor = _gearRatio / _winchCircumference_m;
        // The following two values in terms of Neo motor shaft rotations
        public final static double kDownPosNeoRotations = KElevatorMetersToNeoRotationsFactor * kDownPos_m;
        public final static double kUpPosNeoRotations = KElevatorMetersToNeoRotationsFactor * kUpPos_m;
        // Elevator position limits in units of Neo Motor shaft rotations
        // Add 10% leeway for setting the softlimit
        public final static float kForwardSoftlimit = (float)kDownPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        public final static float kReverseSoftLimit = (float)kUpPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations

        // The following fixed speeds are applicable only if not using the FeedForward or Feedback controllers
        // Note: Elevator uses constant force spring and there is no load so not much power is needed when moving up
        public final static double elevatorMotorUpSpeed = 0.4; 
        public final static double elevatorMotorDownSpeed = -0.75;
    }

    public final class OIConstants {
        public final static int xbox1_port = 0;
        public final static int xbox2_port = 1;
    }
}
