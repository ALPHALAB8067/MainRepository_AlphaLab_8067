// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Actuator_SS;

public class Actuator_down_CMD extends Command {
  /** Creates a new Actuator_up_CMD. */
  private final Actuator_SS mActuator_SS;

  public Actuator_down_CMD(Actuator_SS pActuator_SS) {
    mActuator_SS = pActuator_SS;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mActuator_SS.Actuator_down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mActuator_SS.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
