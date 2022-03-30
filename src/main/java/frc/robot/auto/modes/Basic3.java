package frc.robot.auto.modes;

import frc.robot.auto.actions.FollowTrajectory;
import frc.robot.auto.actions.RunIntake;
import frc.robot.auto.actions.SetTurretPosition;
import frc.robot.auto.actions.Shoot;
import frc.robot.auto.actions.setTracking;
import frc.robot.auto.util.AutoMode;
import frc.robot.auto.util.AutoModeEndedException;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

//import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Drive;
import frc.robot.Intake;
import frc.robot.Shooter;
import frc.robot.Turret;



/**
 *
 */
public class Basic3 extends AutoMode {
	Drive drive;
	Turret turret;
	Shooter shooter;
	Intake intake;
	NetworkTable table;

    public Basic3(Drive drive, Turret turret, Shooter shooter, Intake intake, NetworkTable table) {
		this.drive = drive;
		this.turret = turret;
		this.shooter = shooter;
		this.intake = intake;
		this.table = table;
    }
	/*
	 * (non-Javadoc)
	 * 
	 * @see org.usfirst.frc.team2403.robot.auto.util.AutoMode#routine()
	 */
	@Override
	protected void routine() throws AutoModeEndedException {
		DriverStation.reportWarning("started Action", false);
		// runAction(new setTracking(turret, false));
		// runAction(new SetTurretPosition(turret, Constants.BACK_FACING));
		// runAction(new setTracking(turret, true));
		// runAction(new Shoot(shooter, intake, table, 2.0, 3));
		//runAction(new RunIntake(intake, true));
		// DriverStation.reportWarning("extended intake", false);
		// runAction(new FollowTrajectory(0, drive));
		// runAction(new RunIntake(intake, false));
		runAction(new FollowTrajectory(1, drive));
		// runAction(new setTracking(turret, true));
		// runAction(new Shoot(shooter, intake, table, 3.0, 3));
		// runAction(new setTracking(turret, false));

		DriverStation.reportWarning("Finished Action", false);
	}

}