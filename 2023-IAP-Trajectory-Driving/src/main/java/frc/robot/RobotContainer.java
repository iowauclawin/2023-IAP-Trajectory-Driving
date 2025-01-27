// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.Delayed;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.EncoderDrive;
import frc.robot.Commands.PIDTurnCCW;
import frc.robot.Commands.Ramsete;
import frc.robot.Commands.TankDrive;
import frc.robot.Subsystems.DriveTrain;


public class RobotContainer {

  public static DriveTrain dt = new DriveTrain();
  public PIDTurnCCW PID = new PIDTurnCCW(dt, 90, false);
  public static Joystick j = new Joystick(0);
  public Ramsete ramsete = new Ramsete();
  //public EncoderDrive EncoderDrive = new EncoderDrive(dt);
  public RobotContainer() {
    dt.setDefaultCommand(new TankDrive(dt, j));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return ramsete;
  }     
}
