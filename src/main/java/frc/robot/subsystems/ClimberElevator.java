// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.ClimberElevatorConstants;

public class ClimberElevator extends SubsystemBase {
  private CANSparkMax m_elevator1 = new CANSparkMax(CAN_IDs.climber1_ID, MotorType.kBrushless);
  private CANSparkMax m_elevator2 = new CANSparkMax(CAN_IDs.climber2_ID, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_elevator1.getEncoder();
  /** Creates a new ClimberElevator. */
  public ClimberElevator() {
    m_elevator1.restoreFactoryDefaults();
    m_elevator1.setIdleMode(IdleMode.kBrake);
    m_elevator1.setSmartCurrentLimit(80);

    m_elevator2.restoreFactoryDefaults();
    m_elevator2.setIdleMode(IdleMode.kBrake);
    m_elevator2.setSmartCurrentLimit(80);

    m_elevator2.follow(m_elevator1, true);

    m_elevator1.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_elevator1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_elevator1.setSoftLimit(SoftLimitDirection.kForward, ClimberElevatorConstants.kForwardSoftlimit);
    m_elevator1.setSoftLimit(SoftLimitDirection.kReverse, ClimberElevatorConstants.kReverseSoftLimit);

  }

  public void raiseClimberElevator() {
    if (m_encoder.getPosition() <= ClimberElevatorConstants.kUpPosNeoRotations) {
      m_elevator1.set(ClimberElevatorConstants.elevatorMotorUpSpeed);
    } else {
      m_elevator1.set(0.0);
    }
  }

  public void lowerClimberElevator() {
    if (m_encoder.getPosition() >= ClimberElevatorConstants.kDownPosNeoRotations) {
      m_elevator1.set(ClimberElevatorConstants.elevatorMotorDownSpeed);
    } else {
      m_elevator1.set(0.0);
    }
  }

  public void stopClimberElevator() {
    m_elevator1.set(0.0);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
