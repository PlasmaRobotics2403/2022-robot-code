package frc.robot.auto.actions;

import frc.robot.auto.util.Action;
import frc.robot.Constants;
import frc.robot.Intake;
import edu.wpi.first.wpilibj.DriverStation;

public class RunIntake implements Action{
    Intake intake;
    boolean engaged;

    public RunIntake(Intake intake, boolean engaged){
        this.intake = intake;
        this.engaged = engaged;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        if(engaged == true){
            intake.extendIntake();
            intake.runIntake(Constants.INTAKE_SPEED);
            intake.runKicker(Constants.KICKER_SPEED);
        }
        else {
            intake.stopIntake();
            intake.stopKicker();
            intake.retractIntake();
        }
    }

    @Override
    public void update(){
    }

    public void end() {
        DriverStation.reportWarning("done", false);
    }
}