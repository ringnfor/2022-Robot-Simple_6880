// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {
  public enum ArmState {OPENED, CLOSED};
  private static double kDt = 0.02;

  private CANSparkMax m_intakeArm = new CANSparkMax(CAN_IDs.intakeArm_ID, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_intakeArm.getEncoder();
  private ArmState m_armState = ArmState.CLOSED;
  private final Supplier<Double> m_leftTriggerAxisSupplier;
  private final Supplier<Double> m_rightTriggerAxisSupplier;
  
  /** Creates a new IntakeArm. */
  public IntakeArm(Supplier<Double> leftTriggerSupplier,
      Supplier<Double> rightTriggerSupplier) {
    System.out.print("constructing intake arm\n");
    m_leftTriggerAxisSupplier = leftTriggerSupplier;
    m_rightTriggerAxisSupplier = rightTriggerSupplier;
    m_intakeArm.restoreFactoryDefaults();
    m_intakeArm.setIdleMode(IdleMode.kBrake);
    m_intakeArm.setSmartCurrentLimit(80);

    // Verify that the soft limit is correct after experimentation 
    m_intakeArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_intakeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_intakeArm.setSoftLimit(SoftLimitDirection.kForward, IntakeArmConstants.kForwardSoftlimit);
    m_intakeArm.setSoftLimit(SoftLimitDirection.kReverse, IntakeArmConstants.kReverseSoftLimit);
  }

  // Unused
  public void openArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.OPENED) {
      return;
    } else {
      m_armState = ArmState.OPENED;
    }
  }

  // Unused
  public void closeArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.CLOSED) {
      return;
    } else {
      m_armState = ArmState.CLOSED;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_rightTriggerAxisSupplier.get() > 0) {
      if (m_encoder.getPosition() <= IntakeArmConstants.kClosedPosNeoRotations) {
        m_intakeArm.set(IntakeArmConstants.armMotorClosingSpeed);
      } else {
        m_intakeArm.set(0.0);
      }
      // closeArm();
    } else if (m_leftTriggerAxisSupplier.get() > 0) {
      if (m_encoder.getPosition() >= IntakeArmConstants.kOpenedPosNeoRotations) {
        m_intakeArm.set(IntakeArmConstants.armMotorOpeningSpeed);
      } else {
        m_intakeArm.set(0.0);
      }
      // openArm();
    } else {
      return;
    }
  }

  public double getArmPosition(){
    return m_encoder.getPosition();
  }

  public void setPower(double power){
    m_intakeArm.set(power);
  }
}
