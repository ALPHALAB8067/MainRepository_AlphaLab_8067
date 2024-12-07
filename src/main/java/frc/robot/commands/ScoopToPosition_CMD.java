// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scoop_SS;

public class ScoopToPosition_CMD extends Command {
  /** Creates a new ScoopToPosition_CMD. */
  Scoop_SS mScoop_SS;
  double mPosition;
  Command mScoopDOWN_CMD = new ScoopDOWN_CMD(mScoop_SS);

  /**
  * @param Position position en degrées
  */
  public ScoopToPosition_CMD(Scoop_SS pScoop_SS, Double Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    mScoop_SS = pScoop_SS;
    mPosition = Position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mScoop_SS.gotoPosition(mPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
