/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeArmSubsystem.
   */
  public IntakeArmSubsystem() {
    raiseArm(); //Ensures the arm doeesn't come down when the robot starts up
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Raises the intake arm into the robot
   */
  public void raiseArm() {
    //TODO: Method Stub
  }

  /**
   * Lowers the intake arm out of the robot
   */
  public void lowerArm() {
    //TODO: Method stub
  }
}
