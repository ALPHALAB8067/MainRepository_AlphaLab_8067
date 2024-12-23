// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Actuator_SS;

public class ActuatorCalibration_CMD extends Command {
  /** Creates a new ActuatorCalibration_CMD. */
  Actuator_SS mActuator_SS;
  public ActuatorCalibration_CMD(Actuator_SS pActuator_SS) {
    // Use addRequirements() here to declare subsystem dependencies.
    mActuator_SS = pActuator_SS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mActuator_SS.encoderReset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
