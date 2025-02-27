package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArmSubsystem;

public class WaitForPivotPositionCommand extends Command{
 
    PivotArmSubsystem pivot;

    public WaitForPivotPositionCommand(PivotArmSubsystem pivot){
        this.addRequirements(pivot);
        this.pivot = pivot;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
