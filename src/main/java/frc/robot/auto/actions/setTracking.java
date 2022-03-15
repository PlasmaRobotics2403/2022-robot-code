package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Turret;
import edu.wpi.first.wpilibj.DriverStation;

public class setTracking implements Action{
    Turret turret;
    boolean tracking;

    public setTracking(Turret turret, boolean tracking){
        this.turret = turret;
        this.tracking = tracking;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        turret.setTurretTracking(tracking);
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}