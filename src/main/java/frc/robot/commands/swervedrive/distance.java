// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.AimAndAlign_SS;
import frc.robot.subsystems.swervedrive.DistanceFromAprilTag;

public class distance extends Command {

  private final DistanceFromAprilTag mDistanceFromAprilTag;

  /** Creates a new AimAtTarget_CMD. */
  public distance(DistanceFromAprilTag pDistanceFromAprilTag) {
    mDistanceFromAprilTag = pDistanceFromAprilTag;
    addRequirements(mDistanceFromAprilTag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance", mDistanceFromAprilTag.metersfromapriltagx());
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
