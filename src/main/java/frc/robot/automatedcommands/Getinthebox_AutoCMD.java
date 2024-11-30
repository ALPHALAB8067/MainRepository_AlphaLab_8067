// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.automatedcommands;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ForwardAuto_CMD;
import frc.robot.commands.SidewaysAuto_CMD;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.AimAndAlign_SS;
import frc.robot.subsystems.swervedrive.AimAndCome;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Getinthebox_AutoCMD extends SequentialCommandGroup {
  /** Creates a new Getinthebox_AutoCMD. */

  
  public Getinthebox_AutoCMD(SwerveSubsystem pSwerveSubsystem, AimAndCome pAimAndCome, AimAndAlign_SS pAimAndAlign_SS) {
      
    ForwardAuto_CMD mForwardAuto_CMD = new ForwardAuto_CMD(pSwerveSubsystem, pAimAndCome);
    SidewaysAuto_CMD mSidewaysAuto_CMD = new SidewaysAuto_CMD(pSwerveSubsystem, pAimAndAlign_SS);

    addCommands(
     mForwardAuto_CMD.withTimeout(3),
    mSidewaysAuto_CMD.withTimeout(3)

      );
  }
}
