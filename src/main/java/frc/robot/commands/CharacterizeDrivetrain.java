/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command should be run in conjunction with the drive characteristics
 * tools provided by FIRST. Set this to the auto command while running the tool.
 */
public class CharacterizeDrivetrain extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private NetworkTableEntry autoSpeedEntry, telemetryEntry, rotateEntry;
  double priorAutospeed;
  Number[] numberArray;

  /**
   * Creates a new CharacterizeDrivetrain.
   */
  public CharacterizeDrivetrain(DriveSubsystem subsystem) {
    addRequirements(subsystem);
    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    rotateEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    priorAutospeed = 0;
    numberArray = new Number[10];
    m_subsystem.resetLeftDistance();
    m_subsystem.resetRightDistance();
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double leftPosition = m_subsystem.getLeftDistance();
    double leftRate = m_subsystem.getLeftSpeed();
    double rightPosition = m_subsystem.getRightDistance();
    double rightRate = m_subsystem.getRightSpeed();
    double battery = RobotController.getBatteryVoltage();
    double leftMotorVolts = m_subsystem.getLeftMotorVoltage();
    double rightMotorVolts = m_subsystem.getRightMotorVoltage();
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;
    m_subsystem.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = Math.toRadians(m_subsystem.getGyroAngle());

    telemetryEntry.setNumberArray(numberArray);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
