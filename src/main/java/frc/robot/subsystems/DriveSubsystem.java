/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotIO;
import frc.robot.Settings;
import frc.robot.commands.DriveJoystick;

/**
 * This class is the subsystem for the drivetrain. It controls drive motors, drive encoders, and the imu.
 */
public class DriveSubsystem extends SubsystemBase {
  DifferentialDrive diffDrive = new DifferentialDrive(RobotIO.leftMotor1, RobotIO.rightMotor1);
  public static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Settings.DRIVEBASE_WIDTH);
  DifferentialDriveOdometry driveOdometry;
  private boolean inReverse = false; //Used by joystick drive commands to know when to drive in reverse
  /**
   * Creates a new DriveSubsystem, runs any necessary setup, and applies and important parameters
   */
  public DriveSubsystem() {
    setDefaultCommand(new DriveJoystick(this));

    RobotIO.leftMotor1.setInverted(Settings.INVERT_LEFT_MOTORS);
    RobotIO.rightMotor1.setInverted(Settings.INVERT_RIGHT_MOTORS);
    RobotIO.leftMotor2.follow(RobotIO.leftMotor1);
    RobotIO.leftMotor3.follow(RobotIO.leftMotor1);
    RobotIO.rightMotor2.follow(RobotIO.rightMotor1);
    RobotIO.rightMotor3.follow(RobotIO.rightMotor1);
    RobotIO.leftMotor1.configMotorCommutation(MotorCommutation.Trapeziodal, Settings.CAN_TIMEOUT_INTERVAL);
    RobotIO.leftMotor2.configMotorCommutation(MotorCommutation.Trapeziodal, Settings.CAN_TIMEOUT_INTERVAL);
    RobotIO.leftMotor3.configMotorCommutation(MotorCommutation.Trapeziodal, Settings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor1.configMotorCommutation(MotorCommutation.Trapeziodal, Settings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor2.configMotorCommutation(MotorCommutation.Trapeziodal, Settings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor3.configMotorCommutation(MotorCommutation.Trapeziodal, Settings.CAN_TIMEOUT_INTERVAL);

    setVoltageCompensation(true, Settings.DRIVE_VOLTAGE_COMPENSATION_TARGET);
    setBrakeMode(true);
    RobotIO.leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Settings.CAN_TIMEOUT_INTERVAL);
    RobotIO.rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Settings.CAN_TIMEOUT_INTERVAL);

    diffDrive.setDeadband(0);

    driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()));
  }

  @Override
  public void periodic() {
    driveOdometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftDistance(), getRightDistance());
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

  /**
   * Controls the drive motors with defined outputs for each side of the robot.
   * @param leftSpeed - output for left wheels [-1 to 1]
   * @param rightSpeed - output for right wheels [-1 to 1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * Controls the drive motors with defined voltages for each side of the robot.
   * @param leftSpeed - output for left wheels
   * @param rightSpeed - output for right wheels
   */
  public void tankDriveWithVolts(double leftVolts, double rightVolts) {
    RobotIO.leftMotor1.setVoltage(leftVolts);
    RobotIO.rightMotor1.setVoltage(-rightVolts);
  }

  /**
   * Returns the currently-estimated pose of the robot
   * @return The pose.
   */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetLeftDistance();
    resetRightDistance();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
  }

  /**
   * Returns the distance the left side of the robot has traveled since last reset. In inches
   * @return distance in inches
   */
  public double getLeftDistance() {
    return RobotIO.leftMotor1.getSelectedSensorPosition() * Settings.LEFT_ENC_DIST_PER_PULSE;
  }

  /**
   * Returns the speed of the left sidee of the robot in inches/second
   * @return speed in inches/second
   */
  public double getLeftSpeed() {
    return RobotIO.leftMotor1.getSelectedSensorVelocity() * Settings.LEFT_ENC_DIST_PER_PULSE * 10.0;
  }

  /**
   * Resets the encoder on the left side of the robot
   */
  public void resetLeftDistance() {
    RobotIO.leftMotor1.setSelectedSensorPosition(0);
  }

  /**
   * Returns the distance the left side of the robot has traveled since last reset. In inches
   * @return distance in inches
   */
  public double getRightDistance() {
    return RobotIO.rightMotor1.getSelectedSensorPosition() * Settings.LEFT_ENC_DIST_PER_PULSE;

  }

  /**
   * Returns the speed of the right sidee of the robot in inches/second
   * @return speed in inches/second
   */
  public double getRightSpeed() {
    return RobotIO.rightMotor1.getSelectedSensorVelocity() * Settings.LEFT_ENC_DIST_PER_PULSE * 10.0;

  }

  /**
   * Resets the encoder on the right side of the robot
   */
  public void resetRightDistance() {
    RobotIO.rightMotor1.setSelectedSensorPosition(0);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }
  
  /**
   * Returns the current angle of the robot in degrees
   * @return yaw in degrees
   */
  public double getGyroAngle() {
    return -RobotIO.imu.getGyroAngleZ();
  }

  /**
   * Returns the rotation rate of the robot in degrees per second
   * @return - rotation rate, degrees/s
   */
  public double getRotRate() {
    return -RobotIO.imu.getRate();
  }

  /**
   * Resets the imu angle to 0 degrees.
   */
  public void resetAngle() {
    RobotIO.imu.reset();
  }

  /**
   * Returns true if the drivesubsystem "in reverese" flag is true. Used by joystick drive
   * @return true if drive is in reverse
   */
  public boolean getInReverse() {
    return inReverse;
  }

  /**
   * Useed to set the drive subsystem into reverse. A flag by joystick drive
   * @param reverse - true for reverse
   */
  public void setInReverse(boolean reverse) {
    inReverse = reverse;
  }
  
  /**
   * Returns the voltage currently being applied to the left drivetrain motors
   * @return - electrical potential applied in volts
   */
  public double getLeftMotorVoltage() {
    return RobotIO.leftMotor1.getMotorOutputVoltage();
  }

  /**
   * Returns the voltage currently being applied to the right drivetrain motors
   * @return - electrical potential applied in volts
   */
  public double getRightMotorVoltage() {
    return RobotIO.rightMotor1.getMotorOutputVoltage();
  }

  /**
   * Used to toggle voltage compensation in the drivetrain. This is a useful feature when running
   * PID loops or motion profiling.
   * @param enabled - Pass true to enable, false to disable
   * @param voltage - The voltage that equates to full power.
   */
  public void setVoltageCompensation(boolean enabled, double voltage) {
    RobotIO.leftMotor1.configVoltageCompSaturation(voltage);
    RobotIO.leftMotor1.enableVoltageCompensation(enabled);
    RobotIO.leftMotor2.configVoltageCompSaturation(voltage);
    RobotIO.leftMotor2.enableVoltageCompensation(enabled);
    RobotIO.leftMotor3.configVoltageCompSaturation(voltage);
    RobotIO.leftMotor3.enableVoltageCompensation(enabled);

    RobotIO.rightMotor1.configVoltageCompSaturation(voltage);
    RobotIO.rightMotor1.enableVoltageCompensation(enabled);
    RobotIO.rightMotor2.configVoltageCompSaturation(voltage);
    RobotIO.rightMotor2.enableVoltageCompensation(enabled);
    RobotIO.rightMotor3.configVoltageCompSaturation(voltage);
    RobotIO.rightMotor3.enableVoltageCompensation(enabled);
  }

  /**
   * Used to toggle brake mode on thee drivetrain motors. Pass true to enable, false to disable.
   * @param brake - True if motors should be in brake mode
   */
  public void setBrakeMode(boolean brake) {
    RobotIO.leftMotor1.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.leftMotor2.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.leftMotor3.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.rightMotor1.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.rightMotor2.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    RobotIO.rightMotor3.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }
}
