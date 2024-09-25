package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve_SS;
import frc.robot.commands.Drive_CMD;
import edu.wpi.first.wpilibj.XboxController;


public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final XboxController mController = new XboxController(0);
  private final Swerve_SS mswervedrive = new Swerve_SS();
  private final Drive_CMD mdriveCMD = new Drive_CMD(mswervedrive, mController);

 

  public RobotContainer() {
    mswervedrive.setDefaultCommand(mdriveCMD);
    configureBindings();
  }

 
  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
