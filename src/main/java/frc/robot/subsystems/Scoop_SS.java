// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoop_SS extends SubsystemBase {

  private final TalonSRX mScoopMotor;
  //private final Encoder mEncoder;
  private final DigitalInput mLimitSwitch;
  private final PIDController mPidController;

  //Need to check, either 2048 or 8192
  final double ticksPerRev = 4096;
  final double gearRatio = 10;
  final double degreesPerTick = 360 / (ticksPerRev * gearRatio);

  final double kP = 0;
  final double kI = 0;
  final double kD = 0;

  public Scoop_SS() {
    mScoopMotor = new TalonSRX(21);
    
    //mEncoder = new Encoder(0, 0);
    mLimitSwitch = new DigitalInput(9);

    mPidController = new PIDController(kP, kI, kD);
  }

  public void stop() {
    mScoopMotor.set(ControlMode.PercentOutput, 0);
  }

  public void limitSwitch() {
    if (mLimitSwitch.get()){
      mScoopMotor.set(ControlMode.PercentOutput, 0); 
      //mEncoder.reset();
    } 
  }

  public double getScoopAngle() {
    //return mEncoder.get() * degreesPerTick;
    return mScoopMotor.getSelectedSensorPosition() * degreesPerTick;
  }

  public void setScoopPosition() {
    double actualAngle = getScoopAngle();
    double targetAngle = 45;
    double output = mPidController.calculate(actualAngle, targetAngle);
    mScoopMotor.set(ControlMode.PercentOutput, output);
  }

  //im going to change this bs
  public double getOutputForTelemetry() {
    double actualAngle = getScoopAngle();
    double targetAngle = 45;
    double outputforTelemetry = mPidController.calculate(actualAngle, targetAngle);
    return outputforTelemetry;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("kP", kP);    
    SmartDashboard.putNumber("kI", kI);    
    SmartDashboard.putNumber("kD", kD);  
    SmartDashboard.putNumber("Actual Angle", getScoopAngle());    
    SmartDashboard.putNumber("Output", getOutputForTelemetry());  
    
    //not optimized at all
    limitSwitch();
  }
}
