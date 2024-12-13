package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.automatedcommands.Getinthebox_AutoCMD;
import frc.robot.commands.ActuatorCalibration_CMD;
import frc.robot.commands.ActuatorToPositionSmart_CMD;
import frc.robot.commands.Actuator_down_CMD;
import frc.robot.commands.Actuator_up_CMD;

import frc.robot.commands.ScoopCalibration_CMD;
import frc.robot.commands.ScoopDOWN_CMD;
import frc.robot.commands.ScoopToArm_CMD;
import frc.robot.commands.ScoopToPosition_CMD;
import frc.robot.commands.ScoopUP_CMD;
 
import frc.robot.commands.ShootSmart_CMD;
import frc.robot.commands.ShootStop_CMD;
import frc.robot.commands.Shoot_CMD;
import frc.robot.commands.Shoot_CMDautomatic;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Actuator_SS;
import frc.robot.subsystems.Scoop_SS;
import frc.robot.subsystems.Shooter_SS;
import frc.robot.subsystems.swervedrive.AimAndAlign_SS;
import frc.robot.subsystems.swervedrive.AimAndCome;
import frc.robot.subsystems.swervedrive.AlignAprilTag;
import frc.robot.subsystems.swervedrive.DistanceFromAprilTag;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer
{

  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Swerve Subsystem
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  // April Tag Subsystems

  private final AimAndAlign_SS AimAndAlign_SS = new AimAndAlign_SS(); 
  private final DistanceFromAprilTag mDistanceFromAprilTag = new DistanceFromAprilTag();       private final DistanceFromAprilTag distanceFromAprilTag = new DistanceFromAprilTag();
  private final AimAndCome aimAndCome = new AimAndCome();   
  private final AlignAprilTag alignAprilTag = new AlignAprilTag();     
       
  // April Tag commands 
  private final Getinthebox_AutoCMD mGetinthebox_AutoCMD = new Getinthebox_AutoCMD(drivebase, aimAndCome, AimAndAlign_SS);

  // Actuator Subsytem
  private final Actuator_SS mActuator_SS = new Actuator_SS();

  // Actuator Commands 
  private final ActuatorToPositionSmart_CMD mActuatorToPosition_CMDprecise = new ActuatorToPositionSmart_CMD(mActuator_SS, mDistanceFromAprilTag.calculateangle());
  private final Actuator_up_CMD mActuator_up_CMD = new Actuator_up_CMD(mActuator_SS);
  private final Actuator_down_CMD mActuator_down_CMD = new Actuator_down_CMD(mActuator_SS);
  private final ActuatorCalibration_CMD mActuatorCalibration_CMD = new ActuatorCalibration_CMD(mActuator_SS);
  private final ActuatorToPositionSmart_CMD mActuatorToPosition_CMD = new ActuatorToPositionSmart_CMD(mActuator_SS, 25.0);

  // Scoop Subsytem 
  
  private final Scoop_SS mScoop_SS = new Scoop_SS();

  //Scoop Commands
  private final ScoopCalibration_CMD mScoopCalibration_CMD = new ScoopCalibration_CMD(mScoop_SS);
  private final ScoopDOWN_CMD mScoopDOWN_CMD = new ScoopDOWN_CMD(mScoop_SS);
  private final ScoopUP_CMD mScoopUP_CMD = new ScoopUP_CMD(mScoop_SS);
  private final ScoopToPosition_CMD mScoopToPosition_CMD = new ScoopToPosition_CMD(mScoop_SS, 30.0 );
  private final ScoopToArm_CMD mScoopToPosition2_CMD = new ScoopToArm_CMD(mScoop_SS,mActuator_SS);

  // Shooter Subsytem
  private final Shooter_SS mShooter_SS = new Shooter_SS();

  // Shooter Commands
    private final Shoot_CMD mShoot_CMD = new Shoot_CMD(mShooter_SS);
    private final Shoot_CMDautomatic mShoot_CMDautomatic = new Shoot_CMDautomatic(mShooter_SS, mDistanceFromAprilTag.motor());
    private final ShootStop_CMD mShootStop_CMD = new ShootStop_CMD(mShooter_SS);
    private final ShootSmart_CMD mShootSmart_CMD = new ShootSmart_CMD(mShooter_SS);

  public RobotContainer()
  {
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
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() *0.4 , OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() *-0.4, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.6);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
  }

  private void configureBindings()
  {

    // April Tag Commands

     driverXbox.povUp().whileTrue(
           drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () ->  MathUtil.applyDeadband((aimAndCome.ForwardAim()), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.8));    


    driverXbox.povDown().whileTrue(  drivebase.driveCommand(
      () -> MathUtil.applyDeadband(alignAprilTag.ForwardAim(1, 0.85), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * 0.8));

    // Orientation Commands

    driverXbox.povLeft().whileTrue(
      drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
        () -> -0.27));

     driverXbox.povRight().whileTrue(
      drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() , OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() , OperatorConstants.LEFT_X_DEADBAND),
        () -> 0.27));
  
    // Start Shooter

    driverXbox.y().whileTrue(mShoot_CMD);

    // Stop Shooter

    driverXbox.b().whileTrue(mActuator_down_CMD);

    // Send Scoop

    driverXbox.a().whileTrue(mScoopToPosition2_CMD);
    driverXbox.a().whileFalse(mScoopToPosition2_CMD);
    driverXbox.start().whileTrue(mScoopDOWN_CMD);

    //driverXbox.a().whileTrue(mScoopUP_CMD);
    //driverXbox.a().whileFalse(mScoopDOWN_CMD);


    // Actuator up and downsP

    //driverXbox.leftBumper().whileTrue(mActuator_up_CMD);
    //driverXbox.rightBumper().whileTrue(mActuator_down_CMD);

    // Scoop up

    driverXbox.x().whileTrue(mActuator_up_CMD);

    driverXbox.leftBumper().whileTrue(drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY() *0.125, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX() *0.125, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.3));
    }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
