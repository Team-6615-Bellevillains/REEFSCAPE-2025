package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotArmSubsystem;

public class ThrowBallAutonCommand extends Command{
    
    private double runTimeSeconds = 1.5;
    private final Timer runTimer = new Timer();
    private PivotArmSubsystem pivotArmSubsystem;
    
    public ThrowBallAutonCommand(PivotArmSubsystem subsystem){
        this.pivotArmSubsystem = subsystem;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        runTimer.restart();
        pivotArmSubsystem.throwBall();

    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return true;
        }
        
        return runTimer.hasElapsed(runTimeSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        SharedState.get().setLoaded(false);
        pivotArmSubsystem.setGrabberMotor(0);
    }


}
