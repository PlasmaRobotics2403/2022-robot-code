package frc.robot;

public class Constants {
    /* front of robot has electronics */
	/* right & left sides from robot's perspective */

	/* CONTROLLER CONSTANTS */
	public static final int JOYSTICK1_PORT = 0;
	public static final int JOYSTICK2_PORT = 1;

	/* CAN ID CONSTANTS */
	public static final int L_DRIVE_ID = 0; // left side motor farthest from talons
	public static final int L_DRIVE_SLAVE_ID = 1;
	public static final int R_DRIVE_ID = 2; // right side motor farthest from talons
	public static final int R_DRIVE_SLAVE_ID = 3;
	

	public static final int INTAKE_ID = 4;
	public static final int KICKER_ID = 5;
	public static final int INDEX_ID = 6;

	public static final int TURRET_ID = 7;

	public static final int CLIMB_ID = 8;

	public static final int SHOOTER_MAIN_MOTOR_ID = 30;
	public static final int SHOOTER_ACCELERATOR_MOTOR_ID = 31;

	public static final int POWER_DISTRIBUTION_HUB = 20;
	public static final int PNUEMATIC_HUB_ID = 21;

	/* PNUEMATIC CONSTANTS */
	public static final int INTAKE_SOLENOID_CHANNEL = 0;
	public static final int SHOOTER_SOLENOID_CHANNEL = 2;
	public static final int CLIMB_SOLENOID_CHANNEL = 1;
	

	/* DIO ID CONSTANTS */
	public static final int FRONT_INDEX_SENSOR_ID = 0;
	public static final int MID_INDEX_SENSOR_ID = 1;
	public static final int BACK_INDEX_SENSOR_ID = 2;
	

	/* DRIVETRAIN CONSTANTS */
	public static final double MAX_AUTO_DRIVE_SPEED = 0.9;
	public static final double MAX_DRIVE_SPEED = 1;
	public static final double MAX_DRIVE_TURN = 0.8;
	public static final double DRIVE_ENCODER_CONVERSION = 0.00003083; // ticks to meters //0.001581037;
	public static final double WHEEL_BASE = 0.65; //distance between left and right wheel in meters
	public static final int UNITS_PER_METER = 32848;

	/* SHOOTER CONSTANTS */
	public static final double BASE_FLYWHEEL_SPEED = 0.77;
	public static final double ACCELERATOR_SPEED = 0.5;

	/* INTAKE CONSTANTS */
	public static final double INTAKE_SPEED = 0.5;
	public static final double KICKER_SPEED = 0.5;
	public static final double INDEX_SPEED = 0.5;


	/* TURRET CONSTANTS */
	public static final double MAX_LIMIT_DISTANCE = 25000; //33381
	public static final double MIN_LIMIT_DISTANCE = -5000; //-11661

	public static final int ENCODER_TICKS_PER_ROTATION = 40800;


	/* CLIMB CONSTANTS */
	public static final double MAX_SPOOL_SPEED = .9;



	/* VISION CONSTANTS */
	public static final double CAMERA_HEIGHT = 25.5; //inches
	public static final double CAMERA_ANGLE = 24; //degrees
	public static final double OUTERPORT_HEIGHT = 98; // inches
	public static final double x2_ZOOM_Y_CONVERION = 1.077;
	public static final double LIMELIGHT_PAN = 6.468;
	public static final double VISION_X_OFFSET = -1.5;

	/* TALON CONFIG CONSTANTS */
	public static final int TALON_TIMEOUT = 30;
	

}