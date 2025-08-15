package frc.robot.commands;


import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotSubsytem;

public class LoadCoralLaserCANCommand extends Command{
    private boolean finished;
    private boolean seenCoral;

    private final PivotSubsytem pivotSubsystem;

    private final Timer simulationTimer = new Timer();
    private Time simulationTimeUntilLoad;

    public LoadCoralLaserCANCommand(PivotSubsytem pivotSubsystem){
        this.addRequirements(pivotSubsystem);

        this.pivotSubsystem = pivotSubsystem;
    }
    
    @Override
    public void initialize() {
       finished = false;
       seenCoral = false; 
       pivotSubsystem.loadCoral();
       
       if (Robot.isSimulation()) {
        simulationTimer.restart();
        simulationTimeUntilLoad = Seconds.of(ThreadLocalRandom.current().nextDouble(1.5, 5));
       }
    }

    @Override
    public void execute() {
        if(!seenCoral){
            seenCoral = pivotSubsystem.measureCoralInWay();
        } else finished = !pivotSubsystem.measureCoralInWay();

        if (Robot.isSimulation()) {
            SmartDashboard.putNumber("Sim. Load Finished Countdown", simulationTimeUntilLoad.in(Seconds) - simulationTimer.get());
        }
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return simulationTimer.hasElapsed(simulationTimeUntilLoad.in(Seconds));
        }
        
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stopMotors();
        if (!interrupted) {
            SharedState.get().setLoaded(true);
        }
    }
}
