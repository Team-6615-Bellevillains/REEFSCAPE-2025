package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

public class GrabAlgaeCommand extends Command{
    private final AlgaeGrabberSubsystem algae;
    private AlgaeGrabProgress algaeGrabProgress;
    private final Timer spinupTimer = new Timer();
    private final Timer suckingTimer = new Timer();

    private static final Time SUCK_TIME = Seconds.of(1);

    public GrabAlgaeCommand(AlgaeGrabberSubsystem algae){
        addRequirements(algae);
        this.algae = algae;

        algaeGrabProgress = AlgaeGrabProgress.WAITING;
    }

    @Override
    public void initialize() {
        algae.setReference(Degrees.of(65));
        algae.setGrabberPower(Percent.of(-100));
        algae.setGrabberCurrentLimit(Amps.of(30));

        suckingTimer.restart();
        spinupTimer.restart();
        algaeGrabProgress = AlgaeGrabProgress.WAITING;
    }

    @Override
    public void execute() {
        switch (algaeGrabProgress){
            case WAITING:
                if (spinupTimer.hasElapsed(0.4) && algae.getGrabberVelocity().gt(Rotations.per(Minute).of(-30))) {
                    algaeGrabProgress = AlgaeGrabProgress.SUCKING;
                }

                break;
            case SUCKING:
                if (suckingTimer.hasElapsed(SUCK_TIME.in(Seconds))) {
                    algaeGrabProgress = AlgaeGrabProgress.FINALISING;
                }

                break;
            case FINALISING:
                algae.setReference(Degrees.of(15));

                if (algae.getPosition().isNear(Degrees.of(15), Degrees.of(2))) {
                    algaeGrabProgress = AlgaeGrabProgress.END;
                }

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
        algae.setGrabberCurrentLimit(Amps.of(30));

        if (interrupted){
            algae.setReference(Degrees.of(5));
            algae.setGrabberPower(Percent.zero());
        } 
    }

    enum AlgaeGrabProgress{
        WAITING,
        SUCKING,
        FINALISING,
        END
    }
}
