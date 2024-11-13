// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.AimAndAlign_SS;

public class AimAtTarget_CMD extends Command {

  private final AimAndAlign_SS mAimAndAlign_SS;
  private final double turnKP;
  private final double maxturnspeed;
  /** Creates a new AimAtTarget_CMD. */
  public AimAtTarget_CMD(AimAndAlign_SS pAimAndAlign_SS, double t_turnkp, double m_maxturnspeed) {
    mAimAndAlign_SS = pAimAndAlign_SS;
    turnKP = t_turnkp;
    maxturnspeed = m_maxturnspeed;
    addRequirements(mAimAndAlign_SS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   MathUtil.applyDeadband(mAimAndAlign_SS.AimAtApril(turnKP, maxturnspeed), 0.1); 
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
