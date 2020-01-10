package frc.robot;

public class Settings {
//HID

    //Joystick Settings
    public static final int DRIVE_STICK_PORT = 0;
    public static final int OPERATOR_STICK_PORT = 1;

    public static final int SHOOT_BUTTON = 1; //operator stick
    public static final int PICKUP_BUTTON = 2; //operator stick


//SUBSYSTEMS

    //DriveTrain Settings
    public static final int LEFT_MOTOR_1_ID = 1;
    public static final int LEFT_MOTOR_2_ID = 2;
    public static final int LEFT_MOTOR_3_ID = 3;
    public static final int RIGHT_MOTOR_1_ID = 4;
    public static final int RIGHT_MOTOR_2_ID = 5;
    public static final int RIGHT_MOTOR_3_ID = 6;

    public static final boolean INVERT_LEFT_MOTORS = false;
    public static final boolean INVERT_RIGHT_MOTORS = false;
    public static final double JOYSTICK_ROTATION_MULTIPLIER = 0.7; //Set to <1 to make robot turn slower
    public static final double ENCODER_DIST_PER_PULSE = 0.02; //Inches traveled per pulse

    //Pickup Settings


    //Indexer Settings


    //Shooter Settings


    //Climber Settings

}