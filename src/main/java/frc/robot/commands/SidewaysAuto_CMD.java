// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.AimAndAlign_SS;
import frc.robot.subsystems.swervedrive.AimAndCome;
import frc.robot.subsystems.swervedrive.AlignAprilTag;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SidewaysAuto_CMD extends Command {

private AimAndAlign_SS mAlignAprilTag = new AimAndAlign_SS();
private SwerveSubsystem drivebase;
  /** Creates a new ForwardAuto_CMD. */
  public SidewaysAuto_CMD(SwerveSubsystem pSwerveSubsystem, AimAndAlign_SS pAimAndAlign_SS) {
    drivebase = pSwerveSubsystem;
    mAlignAprilTag = pAimAndAlign_SS;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.driveCommand(
        () -> MathUtil.applyDeadband(mAlignAprilTag.ForwardAim(1, 0.8), OperatorConstants.LEFT_Y_DEADBAND),
        () -> (0),
        () -> 0);    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());   // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
         
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
