/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeWheelsSpin;

public class IntakeWheelsSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeWheelsSubsystem.
   */
  public IntakeWheelsSubsystem() {
    setDefaultCommand(new IntakeWheelsSpin(this, 0)); //Wheels don't spin by default
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Rotates the intake wheels to suck in or dispose of a ball
   * @param speed - the speed of the wheels (1 - full intake, -1 - full reverse)
   */
  public void setWHeelSpeed(double speed) {
    //TODO: Method stub
  }
}
