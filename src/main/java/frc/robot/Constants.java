package frc.robot;

import javax.swing.undo.StateEdit;

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
	public static final int CLIMB_PIVOT_ID = 9;

	public static final int SHOOTER_MAIN_MOTOR_ID = 30;

	public static final int POWER_DISTRIBUTION_HUB = 20;
	public static final int PNUEMATIC_HUB_ID = 21;

	/* PNUEMATIC CONSTANTS */
	public static final int INTAKE_SOLENOID_CHANNEL = 0;
	public static final int CLIMB_SOLENOID_CHANNEL = 1;
	public static final int SHOOTER_SOLENOID_CHANNEL = 2;
	

	/* DIO ID CONSTANTS */
	public static final int FRONT_INDEX_SENSOR_ID = 0;
	

	/* DRIVETRAIN CONSTANTS */
	public static final double MAX_AUTO_DRIVE_SPEED = 0.9;
	public static final double MAX_DRIVE_SPEED = 1;
	public static final double MAX_DRIVE_TURN = 0.8;
	public static final double DRIVE_ENCODER_CONVERSION = 0.00003083; // ticks to meters //0.001581037;
	public static final double WHEEL_BASE = 0.65; //distance between left and right wheel in meters
	public static final int UNITS_PER_METER = 32848;

	/* SHOOTER CONSTANTS */
	public static final double LOW_SHOT_SPEED = 4000;
	public static final double LAUNCH_PAD_SPEED = 9000;

	/* INTAKE CONSTANTS */
	public static final double INTAKE_SPEED = 0.75;
	public static final double KICKER_SPEED = 0.5;
	public static final double INDEX_SPEED = 0.75;
	public static final double EJECT_MULTIPLIER = 0.6;


	/* TURRET CONSTANTS */
	public static final int MAX_LIMIT_DISTANCE = 30700; ///
	public static final int MIN_LIMIT_DISTANCE = -10300; /// -10476
	public static final double TURRET_SPEED = 0.5;
	public static final int ENCODER_TICKS_PER_ROTATION = 40800;

	public static final double FORWARD_FACING = 0;
	public static final double LEFT_FACING = 90;
	public static final double RIGHT_FACING = -90;
	public static final double BACK_FACING = 180;
	public static final double LAUNCH_PAD_ANGLE = 20.2403;


	/* CLIMB CONSTANTS */
	public static final double MAX_CLIMB_SPEED = -0.9;
	public static final double MAX_CLIMB_DISTANCE = 467000; // 522895 //492474 //467634 //-7000?
	public static final double MIN_CLIMB_DISTANCE = 5000; /// 7000

	public static final double MID_BAR_CLIMB_HIGHT = 403348;
	public static final double TRAVERSING_CLIMB_HIGHT = 467000;
	public static final double RELEASE_CLIMB_HOOK_HEIGHT = 86750; // 1/4 MAX_CLIMB
	public static final double OPEN_CLAW_HEIGHT = 430000;  // trial number

	public static final double MAX_PIVOT_SPEED = 0.6;
	public static final double PIVOT_RETRACT_SPEED = -0.3;
	public static final double MAX_PIVOT_DISTANCE = 136790;
	public static final double MIN_PIVOT_DISTANCE = 0;

	public static final double TRAVERSING_PIVOT_DISTANCE = 115183;

	public static final double MAX_TRAVERSE_ANGLE = -2; //gyro pitch
	public static final double MIN_TRAVERSE_ANGLE = 0;



	/* VISION CONSTANTS */
	public static final double CAMERA_HEIGHT = 41.5; //inches
	public static final double CAMERA_ANGLE = 48.6; //degrees  //limelight angle = 90 - 48.6 = 41.4
	public static final double UPPER_HUB_HEIGHT = 104; // inches (8ft 8 in)

	/* TALON CONFIG CONSTANTS */
	public static final int TALON_TIMEOUT = 30;
	

}