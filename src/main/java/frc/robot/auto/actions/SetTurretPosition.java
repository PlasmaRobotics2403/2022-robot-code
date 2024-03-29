package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Turret;
import edu.wpi.first.wpilibj.DriverStation;

public class SetTurretPosition implements Action{
    Turret turret;
    double angle;

    public SetTurretPosition(Turret turret, double angle){
        this.turret = turret;
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {
        
        return (Math.abs(turret.getTurretAngle() - angle) < 2) && (Math.abs(turret.getTurretSpeed()) < 0.025);
    }

    @Override
    public void start() {
        turret.setTurretPosition(angle);
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}