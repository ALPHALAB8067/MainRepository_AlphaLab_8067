// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter_SS;

public class Shoot_CMDautomatic extends Command {
  /** Creates a new Shoot_CMD. */

  private final Shooter_SS mShooter_SS;
  double speed;

  public Shoot_CMDautomatic(Shooter_SS pShooter_SS, double mspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    speed = mspeed;
    mShooter_SS = pShooter_SS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter_SS.shootauto(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter_SS.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
