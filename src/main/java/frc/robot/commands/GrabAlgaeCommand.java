package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

public class GrabAlgaeCommand extends Command{
    private final AlgaeGrabberSubsystem subsystem;
    private boolean stopped;
    private int delayTicks;

    public GrabAlgaeCommand(AlgaeGrabberSubsystem subsystem ){
        this.subsystem = subsystem;
        addRequirements(subsystem);
        stopped = false;
        delayTicks = 30;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing GrabAlgaeCommand");
        subsystem.setPositionDegrees(65);
        subsystem.setGrabberSpeed(-0.2);
        stopped = false;
        delayTicks = 30;
    }

    @Override
    public void execute() {
        //System.out.println(delayTicks);
        if (delayTicks != 0){
            if (subsystem.checkGrabberCurrent() >72){ 
                delayTicks--;
                //System.out.println("current limit exceeded");
            }
            //System.out.println("waiting");
        } else {
            subsystem.setPositionDegrees(0);
            //System.out.println("final grabbing phase");
            if (subsystem.getPositionDegrees() < 0.5 && subsystem.getPositionDegrees() > -0.5) stopped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return stopped;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("GrabAlgaeCommand finished");
        subsystem.setGrabberSpeed(0);
    }
}
