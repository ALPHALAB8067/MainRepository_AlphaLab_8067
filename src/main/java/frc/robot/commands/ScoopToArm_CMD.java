// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Actuator_SS;
import frc.robot.subsystems.Scoop_SS;

public class ScoopToArm_CMD extends Command {
  /** Creates a new ScoopToPosition_CMD. */
  Scoop_SS mScoop_SS;
  Actuator_SS mActuator_SS;
  double mScoopPosition;
  double mEncoderdifference = 32;
  double mDontBreakValue = -10;
  /**
  * @param Position position en degrées
  */
  public ScoopToArm_CMD(Scoop_SS pScoop_SS,Actuator_SS pActuator_SS ) {
    // Use addRequirements() here to declare subsystem dependencies.
    mScoop_SS = pScoop_SS;
    mActuator_SS = pActuator_SS;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mScoopPosition = mActuator_SS.getPosition()+(mEncoderdifference + mDontBreakValue);
    SmartDashboard.putNumber("scoop to arm",mScoopPosition);
    mScoop_SS.gotoPosition(mScoopPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
