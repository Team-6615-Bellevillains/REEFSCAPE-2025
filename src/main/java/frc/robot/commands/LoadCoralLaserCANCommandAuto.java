package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArmSubsystem;

public class LoadCoralLaserCANCommandAuto extends Command{

    private boolean finished;
    private boolean seenCoral;
    private int runCycles;
    private int reverseCycles;

    private PivotArmSubsystem pivot;

    public LoadCoralLaserCANCommandAuto(PivotArmSubsystem subsystem){
        this.addRequirements(subsystem);
        pivot = subsystem;
    }
    
    @Override
    public void initialize() {
       finished = false;
       seenCoral = false; 
       pivot.loadCoral();
       runCycles = 0;
       reverseCycles = 0;
    }

    @Override
    public void execute() {
        if(!seenCoral){
            if (reverseCycles > 0){
                pivot.reverse();
                reverseCycles--;
            } else if (runCycles >= 150){
                reverseCycles = 25;
                runCycles = 0;
            }
            seenCoral = pivot.measureCoralInWay();
            runCycles++;
        } else finished = !pivot.measureCoralInWay();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.stopMotors();
    }
}
