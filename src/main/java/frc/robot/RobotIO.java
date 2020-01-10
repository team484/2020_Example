/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class RobotIO {
    public static final Joystick driveStick = new Joystick(Settings.DRIVE_STICK_PORT);
    public static final Joystick operatorStick = new Joystick(Settings.OPERATOR_STICK_PORT);

    public static final WPI_TalonFX leftMotor1 = new WPI_TalonFX(Settings.LEFT_MOTOR_1_ID);
    public static final WPI_TalonFX leftMotor2 = new WPI_TalonFX(Settings.LEFT_MOTOR_2_ID);
    public static final WPI_TalonFX leftMotor3 = new WPI_TalonFX(Settings.LEFT_MOTOR_3_ID);
    public static final WPI_TalonFX rightMotor1 = new WPI_TalonFX(Settings.RIGHT_MOTOR_1_ID);
    public static final WPI_TalonFX rightMotor2 = new WPI_TalonFX(Settings.RIGHT_MOTOR_2_ID);
    public static final WPI_TalonFX rightMotor3 = new WPI_TalonFX(Settings.RIGHT_MOTOR_3_ID);


    public static void setup() {
        leftMotor1.setInverted(Settings.INVERT_LEFT_MOTORS);
        leftMotor2.setInverted(Settings.INVERT_LEFT_MOTORS);
        leftMotor3.setInverted(Settings.INVERT_LEFT_MOTORS);
        rightMotor1.setInverted(Settings.INVERT_RIGHT_MOTORS);
        rightMotor2.setInverted(Settings.INVERT_RIGHT_MOTORS);
        rightMotor3.setInverted(Settings.INVERT_RIGHT_MOTORS);

        leftMotor2.follow(leftMotor1);
        leftMotor3.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
        rightMotor3.follow(rightMotor1);
    }
}
