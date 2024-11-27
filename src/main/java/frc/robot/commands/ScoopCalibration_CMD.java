// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scoop_SS;

public class ScoopCalibration_CMD extends Command {
  /** Creates a new ScoopTest_CMD. */
  Scoop_SS mScoop_SS;
  public ScoopCalibration_CMD(Scoop_SS pScoop_SS) {
    // Use addRequirements() here to declare subsystem dependencies.
    mScoop_SS = pScoop_SS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mScoop_SS.ScoopCalibration();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mScoop_SS.ScoopCalibrationDone;
  }
}
