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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeWheelsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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

  private final Command autoCommand = new DriveJoystick(driveSubsystem);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
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
    // The command that will run in autonomous
    return autoCommand;
  }
}
