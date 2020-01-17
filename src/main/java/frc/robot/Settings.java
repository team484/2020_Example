package frc.robot;

public class Settings {
//HID

    //Joystick Settings
    public static final int DRIVE_STICK_PORT = 0;
    public static final int OPERATOR_STICK_PORT = 1;

    //Driver Stick Buttons
    public static final int DRIVE_REVERSE_BUTTON = 1;
    public static final int DRIVE_ALIGN_TARGET_BUTTON = 2;

    //Operator Stick Buttons
    public static final int SHOOT_BUTTON = 1;
    public static final int PICKUP_BUTTON = 2;


//SUBSYSTEMS

    //-----DriveTrain Settings-----
    public static final int LEFT_MOTOR_1_ID = 1;
    public static final int LEFT_MOTOR_2_ID = 2;
    public static final int LEFT_MOTOR_3_ID = 3;
    public static final int RIGHT_MOTOR_1_ID = 4;
    public static final int RIGHT_MOTOR_2_ID = 5;
    public static final int RIGHT_MOTOR_3_ID = 6;

    public static final boolean INVERT_LEFT_MOTORS = false;
    public static final boolean INVERT_RIGHT_MOTORS = false;
    public static final double JOYSTICK_ROTATION_MULTIPLIER = 0.7; //Set to <1 to make robot turn slower
    //Drivetrain reduction is 7.44047619. Encoder is 2048 CPR. Wheels are 6.25-inch
    public static final double LEFT_ENC_DIST_PER_PULSE = 0.001288543861824; //Inches traveled per pulse
    public static final double RIGHT_ENC_DIST_PER_PULSE = -LEFT_ENC_DIST_PER_PULSE; //Inches traveled per pulse

    public static final double DRIVE_VOLTAGE_COMPENSATION_TARGET = 11.0;


    //-----Intake Settings-----
    public static final double INTAKE_WHEEL_SPEED = 0.9;

    //-----Indexer Settings-----


    //-----Shooter Settings-----


    //-----Climber Settings-----


//MISC
    public static final int CAN_TIMEOUT_INTERVAL = 100;

}