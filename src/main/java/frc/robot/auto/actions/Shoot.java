package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import frc.robot.Shooter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Shoot implements Action{
    Shooter shooter;
    Intake intake;
    NetworkTable table;

    NetworkTableEntry ty;
    NetworkTableEntry ta;

    double vision_Y;
    double vision_Area;

    double distanceFromTarget;

    double startTime;
    double finishTime;


    public Shoot(Shooter shooter, Intake intake, NetworkTable table, double finishTime ){
        this.shooter = shooter;
        this.intake = intake;
        this.table = table;

        this.finishTime = finishTime;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= finishTime + startTime;
    }

    @Override
    public void start() {
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        table.getEntry("ledMode").setNumber(3);

        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update(){
        vision_Y = ty.getDouble(0.0);
        vision_Area = ta.getDouble(0.0);

        distanceFromTarget = (((Constants.UPPER_HUB_HEIGHT - Constants.CAMERA_HEIGHT) / Math.tan(Constants.CAMERA_ANGLE*Math.PI/180 + (vision_Y)*Math.PI/180))/12 - 2);

        if(vision_Area != 0){
            shooter.autoShoot(distanceFromTarget);
            double errorValue = Math.abs(shooter.getTargetShootSpeed(distanceFromTarget) - shooter.getShooterSpeed());
            //SmartDashboard.putNumber("shooter error", errorValue);
            if(errorValue < 400){
              intake.runIndex(Constants.INDEX_SPEED);
            }
            else {
              intake.stopIndex();
            }
        }

    }

    public void end() {
        shooter.stopShooter();
        shooter.retractHood();
        intake.stopIndex();
        DriverStation.reportWarning("done", false);
    }
}