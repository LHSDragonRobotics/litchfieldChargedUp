// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RobotDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  public double divrate = 1;
  public double rotrate = 0.1;

  private XboxController xbox = RobotContainer.m_driverController;
  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
  







  private int printCount = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RobotDrive() {
    m_subsystem = RobotContainer.m_robotDrive ;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rightTrigger = xbox.getLeftTriggerAxis();
    if (xbox.getLeftStickButton() || xbox.getRightStickButton()) {
      divrate = 0.1;
      rotrate = 0.1;
    } else {
      divrate = Math.max(0,(.1/(rightTrigger+.1)));
      rotrate = Math.max(0,(.1/(rightTrigger+.1)));
    }
    // System.out.println(String.valueOf(divrate));

    double JS_BIAS_X = 0; //  double JS_BIAS_X = .3 

    // compute a deadband-safe speed for each direction of control.
    double xrate = xbox.getLeftX() + JS_BIAS_X ;  //double xrate = xbox.getLeftX() + JS_BIAS_X ;
    xrate *= Math.abs(xrate * divrate); // competition rate is .8 - The lower the decimal the slower it drives
    double yrate = xbox.getLeftY();
    yrate *= Math.abs(yrate * divrate); // competition rate is .8 - The lower the decimal the slower it drives

    double zRate = xbox.getRawAxis(4)*rotrate;

    if ( printCount++ % 55 == 0 )
        //System.out.println("  DRIVE " + xrate + ", " + yrate + " (" + zRate + ")");

     m_subsystem.drive(-yrate, -xrate, -zRate, false, true);
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
