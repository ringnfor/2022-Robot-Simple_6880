// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.EncoderConstants;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax m_left1 = new CANSparkMax(CAN_IDs.left1_ID, MotorType.kBrushless);
  private CANSparkMax m_left2 = new CANSparkMax(CAN_IDs.left2_ID, MotorType.kBrushless);
  private CANSparkMax m_right1 = new CANSparkMax(CAN_IDs.right1_ID, MotorType.kBrushless);
  private CANSparkMax m_right2 = new CANSparkMax(CAN_IDs.right2_ID, MotorType.kBrushless);
  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;
  private DifferentialDrive m_diffdrv;
  private Encoder m_leftEnc, m_rightEnc;
  private AHRS m_gyro;
  private DifferentialDriveOdometry m_odometry;
  public boolean speed_mode = true;

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
    m_left1.restoreFactoryDefaults();
    m_left2.restoreFactoryDefaults();
    m_right1.restoreFactoryDefaults();
    m_right2.restoreFactoryDefaults();
    m_leftMotors = new MotorControllerGroup(m_left1, m_left2);
    m_rightMotors = new MotorControllerGroup(m_right1, m_right2);
    m_rightMotors.setInverted(true);

    m_diffdrv = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // Initialize Encoders
    m_leftEnc = new Encoder(EncoderConstants.leftEnc_ports[0], EncoderConstants.leftEnc_ports[1], 
                      EncoderConstants.leftEnc_reversed, EncodingType.k4X);
    m_rightEnc = new Encoder(EncoderConstants.rightEnc_ports[0], EncoderConstants.rightEnc_ports[1],
                      EncoderConstants.rightEnc_reversed, EncodingType.k4X);
    
    m_leftEnc.setDistancePerPulse(DriveTrainConstants.kWheelCircumference_m / (EncoderConstants.revThroughboreEnc_PPR * 4));
    m_rightEnc.setDistancePerPulse(DriveTrainConstants.kWheelCircumference_m / (EncoderConstants.revThroughboreEnc_PPR * 4));

    resetEncoders();

    // Initialize Gyro
    m_gyro = new AHRS(Port.kMXP);
    m_gyro.reset();

    // Initialize the Odometry object
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEnc.getDistance(), m_rightEnc.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEnc.getRate(), m_rightEnc.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_diffdrv.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_diffdrv.feed();
  }


  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEnc.reset();
    m_rightEnc.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEnc.getDistance() + m_rightEnc.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEnc;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEnc;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffdrv.setMaxOutput(maxOutput);
  }

  /**
   * Toggles the max output of the drive. Useful for scaling the drive to drive more slowly.
   */
  public void toggleMaxOutput(){
    if(speed_mode) {
      m_diffdrv.setMaxOutput(0.5);
      speed_mode = false;
    } else {
      m_diffdrv.setMaxOutput(1);
      speed_mode = true;
    }
}

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
