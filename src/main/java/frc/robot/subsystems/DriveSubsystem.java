/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.commands.DriveJoystick;

public class DriveSubsystem extends SubsystemBase {
  DifferentialDrive diffDrive = new DifferentialDrive(RobotIO.leftMotor1, RobotIO.rightMotor1);
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    setDefaultCommand(new DriveJoystick(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets drivetrain output speed and rotation. Also squares inputs
   * @param speed - +1 for forward -1 for reverse
   * @param rotation - +1 for counter clockwise -1 for clockwise
   */
  public void setSpeed(double speed, double rotation) {
    setSpeed(speed, rotation, true);
  }

  /**
   * Sets drivetrain output speed and rotation.
   * @param speed - +1 for forward -1 for reverse
   * @param rotation - +1 for counter clockwise -1 for clockwise
   * @param squareInputs - set false to disable input squaring
   */
  public void setSpeed(double speed, double rotation, boolean squareInputs) {
    diffDrive.arcadeDrive(speed, rotation, squareInputs);
  }
}
