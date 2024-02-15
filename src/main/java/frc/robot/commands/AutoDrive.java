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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class AutoDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private static NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();

  private double rate = 0.3;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoDrive(double rate_param) {


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
    lockOnTag(.8f);
  }
  public void lockOnTag(float rotrate) {
    double tx = tableInstance.getTable("limelight").getEntry("tx").getDouble(0);
    double ta = tableInstance.getTable("limelight").getEntry("ta").getDouble(0);
    double tagID = tableInstance.getTable("limelight").getEntry("tid").getDouble(0);
    double[] trans = new double[3];
    trans = tableInstance.getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    if (tx == 0.0) {
      m_subsystem.drive(0, 0, 0, false, true);
      return;
    }
    double xrate = ((trans[4])/-80)*rotrate;
    double zRate = tx/80;
    double yrate = ((ta-1.5)/2)*rotrate;
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
