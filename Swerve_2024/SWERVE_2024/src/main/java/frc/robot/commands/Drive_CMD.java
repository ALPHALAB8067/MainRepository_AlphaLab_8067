// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve_SS;
import edu.wpi.first.wpilibj.XboxController;

public class Drive_CMD extends Command {
  /** Creates a new DrinkAndDrive_CMD. */
  private final Swerve_SS mswerveDrive;
  private final XboxController controller;
  public Drive_CMD(Swerve_SS mswerveDrive, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mswerveDrive = mswerveDrive;
    this.controller = controller;

    addRequirements(mswerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double fwd = controller.getLeftY();
    double strf = controller.getLeftX();
    double drift = controller.getRightX();

    mswerveDrive.drive(fwd, strf, drift);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //test
  }
}
