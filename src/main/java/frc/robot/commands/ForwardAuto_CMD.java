// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.AimAndCome;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ForwardAuto_CMD extends Command {

private  SwerveSubsystem mdrivebase;
private  AimAndCome mAimAndCome = new AimAndCome();  
  
  /** Creates a new ForwardAuto_CMD. */
  public ForwardAuto_CMD(SwerveSubsystem pSwerveSubsystem, AimAndCome pAimAndCome) {
    mdrivebase = pSwerveSubsystem;
    mAimAndCome = pAimAndCome;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mdrivebase.driveCommand(
        () -> (0),
        () ->  MathUtil.applyDeadband((mAimAndCome.ForwardAim()), OperatorConstants.LEFT_X_DEADBAND),
        () -> 0);    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
         
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
