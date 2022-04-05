// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeWheelConstants;

public class IntakeWheels extends SubsystemBase {
  private CANSparkMax m_intakeWheels = new CANSparkMax(CAN_IDs.intakeWheels_ID, MotorType.kBrushless);
  
  /** Creates a new Intake. */
  public IntakeWheels() {
    m_intakeWheels.restoreFactoryDefaults();
    m_intakeWheels.setIdleMode(IdleMode.kCoast);
    m_intakeWheels.setOpenLoopRampRate(0.5); // time in seconds to go from 0 to full throttle

    m_intakeWheels.setSmartCurrentLimit(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void pullInCargo() {
    m_intakeWheels.set(IntakeWheelConstants.wheelMotorSpeed);
  }

  public void pushOutCargo() {
    m_intakeWheels.set(-IntakeWheelConstants.wheelMotorSpeed);
  }

  public void stopIntakeWheels() {
    m_intakeWheels.set(0.0);
  }
}
