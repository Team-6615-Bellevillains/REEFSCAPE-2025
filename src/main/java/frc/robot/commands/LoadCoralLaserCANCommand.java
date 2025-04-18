package frc.robot.commands;


import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotArmSubsystem;

public class LoadCoralLaserCANCommand extends Command{

    private boolean finished;
    private boolean seenCoral;

    private PivotArmSubsystem pivot;

    private final Timer simulationTimer = new Timer();
    private double simulationTimeUntilLoad;

    public LoadCoralLaserCANCommand(PivotArmSubsystem subsystem){
        this.addRequirements(subsystem);
        pivot = subsystem;
    }
    
    @Override
    public void initialize() {
       finished = false;
       seenCoral = false; 
       pivot.loadCoral();
       
       if (Robot.isSimulation()) {
        simulationTimer.restart();
        simulationTimeUntilLoad = ThreadLocalRandom.current().nextDouble(1.5, 5);
       }
    }

    @Override
    public void execute() {
        if(!seenCoral){
            seenCoral = pivot.measureCoralInWay();
        } else finished = !pivot.measureCoralInWay();

        if (Robot.isSimulation()) {
            SmartDashboard.putNumber("Sim. Load Finished Countdown", simulationTimeUntilLoad - simulationTimer.get());
        }
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return simulationTimer.hasElapsed(simulationTimeUntilLoad);
        }
        
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stopMotors();
        if (!interrupted) {
            SharedState.get().setLoaded(true);
        }
    }
}
