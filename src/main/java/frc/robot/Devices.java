package frc.robot;

public final class Devices {
    //====================Controller====================
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER = 1;
    public static final double JOYSTICK_JOG_SPEED_MULTIPLIER = 0.125; //0.125
    public static final double JOYSTICK_DEADZONE_TOLERANCE = 0.05;
    public static final double CONTROLLER_RUMBLE = 1.0;

    //====================Intake/Indexer====================
    public static final int INTAKE_WRIST_MOTOR = 14;
    public static final int INTAKE_ROLLER_MOTOR = 15;
    public static final int INDEXER_MOTOR = 16;
    public static final int INTAKE_WRIST_THROUGH_BORE_PORT = 7; //THIS IS CORRECT
    //public static final int INDEXER_PHOTOELECTRIC_PORT = 7; 

    //====================End Effector====================
    public static final int END_EFFECTOR_WRIST_MOTOR = 17;
    public static final int END_EFFECTOR_ROLLER_MOTOR = 18;
    public static final int END_EFFECTOR_PHOTOELECTRIC_FRONT_PORT = 0; //0, 1, 8
    public static final int END_EFFECTOR_PHOTOELECTRIC_BACK_PORT = 1;
    
    //====================Elevator====================
    public static final int ELEVATOR_MASTER_MOTOR = 19;
    public static final int ELEVATOR_SLAVE_MOTOR = 20;

    //====================Climb====================
    public static final int CLIMB_WRIST_MOTOR = 21;
    public static final int CLIMB_ROLLER_MOTOR = 22;
}