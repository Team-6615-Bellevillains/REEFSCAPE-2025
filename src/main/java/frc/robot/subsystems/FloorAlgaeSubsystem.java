package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SharedUtils;

@Logged
public class FloorAlgaeSubsystem extends SubsystemBase {
    private final SparkFlex angleMotor = new SparkFlex(30, MotorType.kBrushless);
    private final SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private final SparkFlex intakeMotor = new SparkFlex(31, MotorType.kBrushless);

    // 20 motor rotations per algae grabber rotation
    private static final Dimensionless ANGLE_CONVERSION_FACTOR = Rotations.of(20.0).div(Rotations.one());
    private static final Dimensionless OUTWARDS_OUTPUT_LIMIT = Percent.of(100);
    private static final Dimensionless INWARDS_OUTPUT_LIMIT = Percent.of(-10); 
    
    public FloorAlgaeSubsystem(){
        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();

        angleMotorConfig.closedLoop
            .p(1)
            .i(0)
            .d(0)
            .outputRange(INWARDS_OUTPUT_LIMIT.magnitude(), OUTWARDS_OUTPUT_LIMIT.magnitude());
        angleMotorConfig.idleMode(IdleMode.kBrake);

        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angleMotor.getEncoder().setPosition(0);
        angleController.setReference(0, ControlType.kPosition);
        
        SparkFlexConfig grabberMotorConfig = new SparkFlexConfig();
        grabberMotorConfig.smartCurrentLimit(10);
        intakeMotor.configure(grabberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setReference(Angle reference){
        angleController.setReference(reference.times(ANGLE_CONVERSION_FACTOR).in(Rotations), ControlType.kPosition);
    }

    public void setIntakeCurrentLimit(Current currentLimit){
        SharedUtils.setCurrentLimit(intakeMotor, currentLimit);
    }

    public Angle getPosition(){
        return Rotations.of(angleMotor.getEncoder().getPosition()).div(ANGLE_CONVERSION_FACTOR);
    }

    public void setIntakePower(Dimensionless power){
        intakeMotor.set(power.magnitude());
    }

    public AngularVelocity getIntakeVelocity(){
        return Rotations.per(Minute).of(intakeMotor.getEncoder().getVelocity());
    }

    public Command spitAlgae(){
        return this.runEnd(()->{
            intakeMotor.set(1);
            setIntakeCurrentLimit(Amps.of(40));
        },
        ()->{
            intakeMotor.stopMotor();
            setIntakeCurrentLimit(Amps.of(30));
        });
    }

    public Command resetAlgaeState(){
        return this.run(()->{
            angleController.setReference(0, ControlType.kPosition);
            intakeMotor.set(0);
        });
    }
}
