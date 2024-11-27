// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Actuator_SS;
import frc.robot.subsystems.Scoop_SS;

public class DontBreakStuff_CMD extends Command {
  /** Creates a new DontBreakStuff_CMD. */

  private final Actuator_SS mActuator_SS;
  private final Scoop_SS mScoop_SS;

  public DontBreakStuff_CMD() {
    // Use addRequirements() here to declare subsystem dependencies.
    mActuator_SS = new Actuator_SS();
    mScoop_SS = new Scoop_SS();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //need to set so zero is the same both
    double actuatorPos = mActuator_SS.getActuatorPosition();
    double scoopPos = mScoop_SS.GetencoderInDegrees();
    double goodScoopPos = scoopPos + 32.9;

    if (actuatorPos == goodScoopPos) {
      mScoop_SS.stop();
    } 


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
