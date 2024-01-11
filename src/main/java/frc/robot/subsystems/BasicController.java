// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasicController extends SubsystemBase {

  // Single-motor mechanism....
  private  MotorController motor = null;


  /** Creates a new DriveSubsystem. */
  public BasicController(int CANID) {
    motor = new CANSparkMax(CANID,  MotorType.kBrushless);
  }

  @Override
  public void periodic() {
  }

  /**
   */
  public void go(double power) {
     motor.set(power);
  }

}