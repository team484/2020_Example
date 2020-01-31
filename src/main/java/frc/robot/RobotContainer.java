/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveAlignWithTarget;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.IntakeArmDown;
import frc.robot.commands.IntakeArmUp;
import frc.robot.commands.IntakeWheelsSpin;
import frc.robot.commands.auto.AutoDoNothing;
import frc.robot.commands.auto.AutoDriveExamplePath;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeWheelsSubsystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeArmSubsystem intakeArmSubsystem = new IntakeArmSubsystem();
  private final IntakeWheelsSubsystem intakeWheelsSubsystem = new IntakeWheelsSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    autoChooser.setDefaultOption("Do Nothing", new AutoDoNothing());
    autoChooser.addOption("Example Auto", new AutoDriveExamplePath(driveSubsystem));
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //-----Driver commands-----
    //Reverse Button
    new JoystickButton(RobotIO.driveStick, Settings.DRIVE_REVERSE_BUTTON)
    .whenPressed(() -> driveSubsystem.setInReverse(!driveSubsystem.getInReverse()));

    //Align with target button
    new JoystickButton(RobotIO.driveStick, Settings.DRIVE_ALIGN_TARGET_BUTTON)
    .whileHeld(new DriveAlignWithTarget(driveSubsystem))
    .whenReleased(new DriveJoystick(driveSubsystem));


    //-----Operator commands-----
    //Shoot Button
    new JoystickButton(RobotIO.operatorStick, Settings.SHOOT_BUTTON)
    .whileHeld(new WaitCommand(1)); //WaitCommand is a placeholder

    //Pickup Button
    new JoystickButton(RobotIO.operatorStick, Settings.SHOOT_BUTTON)
    .whenPressed(new IntakeArmDown(intakeArmSubsystem))
    .whileHeld(new IntakeWheelsSpin(intakeWheelsSubsystem, Settings.INTAKE_WHEEL_SPEED))
    .whenReleased(new IntakeArmUp(intakeArmSubsystem))
    .whenReleased(new IntakeWheelsSpin(intakeWheelsSubsystem, 0.0));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser.getSelected() != null) {
      return autoChooser.getSelected();
    } else {
      return new AutoDoNothing();
    }
  }

  /**
   * Generates the command which will drive the desired trajectory.
   * Maybe one of these days I'll add comments to this
   * @param trajectoryName - the name of the trajectory to run
   * @param driveSub - the drivetrain's subsystem
   * @return - the command to execute for the trajectory
   */
  public static Command generateTrajectoryCommand(String trajectoryName, DriveSubsystem driveSub) {
    String trajectoryJSON = "output/"+trajectoryName+".wpilib.json";
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, e.getStackTrace());
      return new WaitCommand(0); // Command to do nothing
    }
    
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      driveSub::getPose, 
      new RamseteController(Settings.DRIVE_RAMSETE_B, Settings.DRIVE_RAMSETE_Z),
      new SimpleMotorFeedforward(
        Settings.DRIVE_KS, 
        Settings.DRIVE_KV,
        Settings.DRIVE_KA),
      DriveSubsystem.driveKinematics,
      driveSub::getWheelSpeeds,
      new PIDController(Settings.DRIVE_KP, 0, 0), 
      new PIDController(Settings.DRIVE_KP, 0, 0),
      driveSub::tankDriveWithVolts,
      driveSub
      );

      return ramseteCommand.andThen(() -> driveSub.tankDrive(0, 0));
  }
}
