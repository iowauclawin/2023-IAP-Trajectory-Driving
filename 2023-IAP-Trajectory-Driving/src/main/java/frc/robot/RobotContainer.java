// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.TankDrive;
import frc.robot.Subsystems.DriveTrain;

public class RobotContainer {

  private DriveTrain dt = new DriveTrain();
  private Joystick j = new Joystick(0);
  public RobotContainer() {
    dt.setDefaultCommand(new TankDrive(dt, j));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
