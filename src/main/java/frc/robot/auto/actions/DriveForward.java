package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Drive;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveForward implements Action{
    double speed;
    double distance;

    Drive drive;

    public DriveForward(double speed, double distance, Drive drive){
        this.drive = drive;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getDistance()) > distance;
    }

    @Override
    public void start() {
        drive.resetEncoders();
        while(Math.abs(drive.getDistance())> 1){
            drive.resetEncoders();
            DriverStation.reportWarning("broke", false);
        }
        drive.zeroGyro();
        
    }

    @Override
    public void update(){
        drive.autonTankDrive(speed, speed);
    }

    public void end() {
        drive.autonTankDrive(0, 0);
        DriverStation.reportWarning("done", false);
    }
}