/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.RobotIO;
import frc.robot.Settings;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drives the robot drivetrain using joystick inputs
 */
public class DriveJoystick extends CommandBase {
  private final DriveSubsystem m_subsystem;

  public DriveJoystick(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_subsystem.getInReverse()) {
      m_subsystem.setSpeed(-RobotIO.driveStick.getY(), -RobotIO.driveStick.getX() * Settings.JOYSTICK_ROTATION_MULTIPLIER);
    } else {
      m_subsystem.setSpeed(RobotIO.driveStick.getY(), -RobotIO.driveStick.getX() * Settings.JOYSTICK_ROTATION_MULTIPLIER);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
