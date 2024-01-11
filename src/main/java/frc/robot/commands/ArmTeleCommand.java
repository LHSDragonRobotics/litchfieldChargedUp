// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.BasicController;

/** An example command that uses an example subsystem. */
public class ArmTeleCommand extends BasicCommand  {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmTeleCommand() {
    super(RobotContainer.arm, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    /////   this...
    //addRequirements(m_subsystem);
    /////    ... was done by super().
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double armPower = RobotContainer.m_driverController.getLeftTriggerAxis() * .25;   // max-power = 20%.
    double altArmPower = - RobotContainer.m_driverController.getRightTriggerAxis() * .25;   // max-power = 10%
    if (Math.abs(armPower)< .05) armPower = altArmPower;
    else if ( Math.abs(armPower)> .15)       System.out.println("  ARM " + armPower);
     m_subsystem.go(armPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.go(0);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
