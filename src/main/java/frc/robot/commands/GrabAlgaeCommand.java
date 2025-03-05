package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

public class GrabAlgaeCommand extends Command{
    private final AlgaeGrabberSubsystem subsystem;
    private int delayTicks;
    private AlgaeGrabProgress algaeGrabProgress;

    public GrabAlgaeCommand(AlgaeGrabberSubsystem subsystem ){
        this.subsystem = subsystem;
        addRequirements(subsystem);
        delayTicks = 30;
        algaeGrabProgress = AlgaeGrabProgress.WAITING;
    }

    @Override
    public void initialize() {
        System.out.println("Initializing GrabAlgaeCommand");
        subsystem.setPositionDegrees(65);
        subsystem.setGrabberSpeed(-0.5);
        delayTicks = 30;
    }

    @Override
    public void execute() {
        switch (algaeGrabProgress){
            case WAITING:
                if (subsystem.checkGrabberRPM() < 1) algaeGrabProgress = AlgaeGrabProgress.SUCKING;
                break;
            
            case SUCKING:
                subsystem.setGrabberSpeed(-0.5);
                delayTicks--;
                if (delayTicks == 0) algaeGrabProgress = AlgaeGrabProgress.FINALISING;
                break;
            
            case FINALISING:
                subsystem.setPositionDegrees(0);
                if (subsystem.getPositionDegrees() > -1 && subsystem.getPositionDegrees() < 1) algaeGrabProgress = AlgaeGrabProgress.END;
                break;

            case END:
                break;

        }
    }

    @Override
    public boolean isFinished() {
        return algaeGrabProgress == AlgaeGrabProgress.END;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("GrabAlgaeCommand finished");
        if (interrupted){
            subsystem.setPositionDegrees(5);
            subsystem.setGrabberSpeed(0);
        } 
    }

    enum AlgaeGrabProgress{
        WAITING,
        SUCKING,
        FINALISING,
        END
    }
}
