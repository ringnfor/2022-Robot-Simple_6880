// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private CANSparkMax m_indexerFront = new CANSparkMax(CAN_IDs.indexerFront_ID, MotorType.kBrushless);
  private CANSparkMax m_indexerBack = new CANSparkMax(CAN_IDs.indexerBack_ID, MotorType.kBrushless);
  /** Creates a new Indexer. */
  public Indexer() {
    m_indexerFront.restoreFactoryDefaults();
    m_indexerBack.restoreFactoryDefaults();
    m_indexerFront.setIdleMode(IdleMode.kBrake);
    m_indexerBack.setIdleMode(IdleMode.kBrake);
    // ToDo:  Check if the smart current limit is too low
    m_indexerFront.setSmartCurrentLimit(30);
    m_indexerBack.setSmartCurrentLimit(30);
    m_indexerBack.follow(m_indexerFront, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void feedToLauncher() {
    m_indexerFront.set(IndexerConstants.indexerMotorSpeed);
  }

  public void throwAwayToIntake() {
    m_indexerFront.set(-IndexerConstants.indexerMotorSpeed);
  }

  public void stopIndexer() {
    m_indexerFront.set(0.0);
  }
}
