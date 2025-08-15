package frc.robot.commands;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotSubsytem;

public class ThrowBallAutonCommand extends Command{
    private static final Time runTime = Seconds.of(1.5);
    private final Timer runTimer = new Timer();
    private final PivotSubsytem pivotSubsystem;
    
    public ThrowBallAutonCommand(PivotSubsytem subsystem){
        this.pivotSubsystem = subsystem;
        this.addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        runTimer.restart();
        pivotSubsystem.throwBall();
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return true;
        }
        
        return runTimer.hasElapsed(runTime.in(Seconds));
    }

    @Override
    public void end(boolean interrupted) {
        SharedState.get().setLoaded(false);
        pivotSubsystem.setGrabberPower(Percent.zero());
    }


}
