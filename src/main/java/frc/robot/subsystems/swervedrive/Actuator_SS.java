// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator_SS extends SubsystemBase {

  private final CANSparkMax mActuator;
  private final RelativeEncoder mEncoder;
  private final DigitalInput mDigitalInput;
  private final SparkPIDController mPIDcontroller;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public boolean mLimitSwitch;

  /** Creates a new Actuator_SS. */
  public Actuator_SS() {
    kP = 0.1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.85; 
    kMinOutput = -0.85;
  mDigitalInput = new DigitalInput(9);
  mActuator = new CANSparkMax(23,MotorType.kBrushed); 
  mEncoder = mActuator.getEncoder();
  mPIDcontroller = mActuator.getPIDController();

  }
  public void Actuator_up (){
    mActuator.set(1);
  }
  
  public void Actuator_down (){
    mActuator.set( -1);
  }
  public void stop(){
    mActuator.set(0);
  }
  @Override

  public void periodic() {
    // This method will be called once per scheduler run
    mLimitSwitch = mDigitalInput.get();
  }
}
