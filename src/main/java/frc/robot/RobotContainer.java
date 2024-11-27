// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ScoopCalibration_CMD;
import frc.robot.commands.ScoopDOWN_CMD;
import frc.robot.commands.ScoopToPosition_CMD;
import frc.robot.commands.ScoopUP_CMD;
import frc.robot.commands.ActuatorCalibration_CMD;
import frc.robot.commands.ActuatorToPosition_CMD;
import frc.robot.commands.Actuator_down_CMD;
import frc.robot.commands.Actuator_up_CMD;
import frc.robot.commands.swervedrive.AimAtTarget_CMD;
import frc.robot.commands.Shoot_CMD;
import frc.robot.commands.Suck_CMD;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Actuator_SS;
import frc.robot.subsystems.Scoop_SS;
import frc.robot.subsystems.Shooter_SS;
import frc.robot.subsystems.swervedrive.AimAndAlign_SS;
import frc.robot.subsystems.swervedrive.AimAndCome;
import frc.robot.subsystems.swervedrive.AlignAprilTag;
import frc.robot.subsystems.swervedrive.DistanceFromAprilTag;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.swervedrive.distance;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final AimAndAlign_SS AimAndAlign_SS = new AimAndAlign_SS();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  private final Actuator_SS mActuator_SS = new Actuator_SS();
  private final Actuator_up_CMD mActuator_up_CMD = new Actuator_up_CMD(mActuator_SS);
  private final Actuator_down_CMD mActuator_down_CMD = new Actuator_down_CMD(mActuator_SS);
  private final ActuatorCalibration_CMD mActuatorCalibration_CMD = new ActuatorCalibration_CMD(mActuator_SS);
  private final ActuatorToPosition_CMD mActuatorToPosition_CMD = new ActuatorToPosition_CMD(mActuator_SS, 10);
  //position en degré

  private final Scoop_SS mScoop_SS = new Scoop_SS();
  private final ScoopCalibration_CMD mScoopCalibration_CMD = new ScoopCalibration_CMD(mScoop_SS);
  private final ScoopDOWN_CMD mScoopDOWN_CMD = new ScoopDOWN_CMD(mScoop_SS);
  private final ScoopUP_CMD mScoopUP_CMD = new ScoopUP_CMD(mScoop_SS);
  private final ScoopToPosition_CMD mScoopToPosition_CMD = new ScoopToPosition_CMD(mScoop_SS ,0 );
  //position a mettre en degré

  private final Shooter_SS mShooter_SS = new Shooter_SS();
  private final Suck_CMD mSuck_CMD = new Suck_CMD(mShooter_SS);
  private final Shoot_CMD mShoot_CMD = new Shoot_CMD(mShooter_SS);




  private final DistanceFromAprilTag distanceFromAprilTag = new DistanceFromAprilTag();
  private final distance distance = new distance(distanceFromAprilTag);
  private final AimAndCome aimAndCome = new AimAndCome();   
  private final AlignAprilTag alignAprilTag = new AlignAprilTag();                            

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                 OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                 OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                 OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.8);
        //() -> MathUtil.applyDeadband(driverXbox.getRightX(), 0.1) * 0.3);


    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

 
  private void configureBindings()
  {
    driverXbox.y().whileTrue(mScoopUP_CMD);
    driverXbox.b().whileTrue(mScoopDOWN_CMD);

    driverXbox.leftBumper().whileTrue(mShoot_CMD);
    driverXbox.rightBumper().whileTrue(mSuck_CMD);

    driverXbox.povLeft().onTrue(mActuatorCalibration_CMD);
    driverXbox.povRight().onTrue(mScoopCalibration_CMD);
    driverXbox.povUp().whileTrue(mActuator_up_CMD);
    driverXbox.povDown().whileTrue(mActuator_down_CMD);

    
    //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //driverXbox.b().whileTrue(
       // Commands.deferredProxy(() -> drivebase.driveToPose(
         //                          new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
           //                   ));
    driverXbox.x().whileTrue(
      drivebase.driveCommand(
        () -> MathUtil.applyDeadband(alignAprilTag.ForwardAim(1, 0.85), OperatorConstants.LEFT_Y_DEADBAND),
        () ->  MathUtil.applyDeadband((aimAndCome.ForwardAim(1,0.85)), OperatorConstants.LEFT_X_DEADBAND),
        () -> AimAndAlign_SS.AimAtApril(0.1 ,0.3)));


    driverXbox.a().whileTrue(
      drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
        () -> AimAndAlign_SS.AimAtApril(0.1 ,0.3)));


      driverXbox.b().whileTrue(
       drivebase.driveCommand(
        () -> MathUtil.applyDeadband(alignAprilTag.ForwardAim(1, 0.8), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.8));    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      
      driverXbox.y().whileTrue(
      drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () ->  MathUtil.applyDeadband((aimAndCome.ForwardAim(1,0.8)), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.8));    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
