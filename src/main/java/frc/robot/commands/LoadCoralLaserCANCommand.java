package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SharedState;
import frc.robot.subsystems.PivotArmSubsystem;

public class LoadCoralLaserCANCommand extends Command{

    private boolean finished;
    private boolean seenCoral;

    private PivotArmSubsystem pivot;

    public LoadCoralLaserCANCommand(PivotArmSubsystem subsystem){
        this.addRequirements(subsystem);
        pivot = subsystem;
    }
    
    @Override
    public void initialize() {
       finished = false;
       seenCoral = false; 
       pivot.loadCoral();
    }

    @Override
    public void execute() {
        if(!seenCoral){
            seenCoral = pivot.measureCoralInWay();
        } else finished = !pivot.measureCoralInWay();
    }

    @Override
    public boolean isFinished() {
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
