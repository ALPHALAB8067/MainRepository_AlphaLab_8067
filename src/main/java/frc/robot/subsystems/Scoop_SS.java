// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoop_SS extends SubsystemBase {

  private final TalonSRX mScoopMotor;
  private final DigitalInput mLimitSwitch;

  //Need to check, either 2048 or 8192
  final double ticksPerRev = 4096;
  final double gearRatio = 10;
  final double degreesPerTick = 360 / (ticksPerRev * gearRatio);

  final double kP = 0;
  final double kI = 0;
  final double kD = 0;

  public Scoop_SS() {
    mLimitSwitch = new DigitalInput(9);

    mScoopMotor = new TalonSRX(21);
    //set factory default and set basic parameter (not nescessary)
    mScoopMotor.configFactoryDefault();
    mScoopMotor.setNeutralMode(NeutralMode.Brake);
    //set what kind of feedback device we use
    mScoopMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,30);
    //to use if forward on the motor is not equal to forward on sensor
    mScoopMotor.setSensorPhase(false);

    //mScoopMotor.configNeutralDeadband(0.04);
    //mScoopMotor.configAllowableClosedloopError(0,0.1 );
    //mScoopMotor.getSensorCollection();
		mScoopMotor.selectProfileSlot(0, 0);
		mScoopMotor.config_kP(0, 0.05, 30);
		mScoopMotor.config_kI(0, 0, 30);
		mScoopMotor.config_kD(0, 0, 30);
    mScoopMotor.config_kF(0, 0, 30);
  }

  public void testPosition(){
    mScoopMotor.set(ControlMode.MotionMagic, 0);
  }

  public void stop() {
    mScoopMotor.set(ControlMode.PercentOutput, 0);
  }
  /* 
  public void limitSwitch() {
    if (mLimitSwitch.get()){
      mScoopMotor.set(ControlMode.PercentOutput, 0); 
    } 
  }
  */

  /* 
  public double getScoopAngle() {
    return mScoopMotor.getSelectedSensorPosition() * degreesPerTick;
  }
  */

  /* 
  public void setScoopPosition() {
    double actualAngle = getScoopAngle();
    double targetAngle = 45;
    //TO FIX OUTPUT VALUE LATER!!!!!//
    mScoopMotor.set(ControlMode.PercentOutput, 0);
  }
  */

  /* 
  //im going to change this bs
  public double getOutputForTelemetry() {
    double actualAngle = getScoopAngle();
    double targetAngle = 45;
    //double outputforTelemetry = mPidController.calculate(actualAngle, targetAngle);
    //return outputforTelemetry;

    //TO CHANGE LATER
    return 0;
  }
  */
  
  public double getEncodervalue(){
    return mScoopMotor.getSelectedSensorPosition();
  }
  @Override
  public void periodic() {
   /*  SmartDashboard.putNumber("kP", kP);    
    SmartDashboard.putNumber("kI", kI);    
    SmartDashboard.putNumber("kD", kD);  
    SmartDashboard.putNumber("Actual Angle", getScoopAngle());    
    SmartDashboard.putNumber("Output", getOutputForTelemetry());
    */ 
    SmartDashboard.putNumber("encoder Value", getEncodervalue()); 
  }
}
