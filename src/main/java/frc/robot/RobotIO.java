/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.analog.adis16448.frc.ADIS16448_IMU;
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

    public static final ADIS16448_IMU imu = new ADIS16448_IMU();


    public static void setup() {
        leftMotor1.configFactoryDefault();
        leftMotor2.configFactoryDefault();
        leftMotor3.configFactoryDefault();
        rightMotor1.configFactoryDefault();
        rightMotor2.configFactoryDefault();
        rightMotor3.configFactoryDefault();
    }
}
