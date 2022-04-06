// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;

public class OpenArm extends CommandBase {
  IntakeArm m_arm;
  
  /** Creates a new OpenArm. */
  public OpenArm(IntakeArm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_arm.setPower(IntakeArmConstants.armMotorOpeningSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_arm.getArmPosition() <= IntakeArmConstants.kOpenedPosNeoRotations){ return true;}
    else{ return false;}
  }
}
