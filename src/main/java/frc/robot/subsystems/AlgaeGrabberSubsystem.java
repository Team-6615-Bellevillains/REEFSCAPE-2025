package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SharedUtils;

@Logged
public class AlgaeGrabberSubsystem extends SubsystemBase {
    private final SparkFlex angleMotor = new SparkFlex(30, MotorType.kBrushless);
    private final SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
    private final SparkFlex grabberMotor = new SparkFlex(31, MotorType.kBrushless);

    private static final Dimensionless ANGLE_CONVERSION_FACTOR = Rotations.of(20.0).div(Degrees.of(360.0));
    private static final Dimensionless OUTWARDS_OUTPUT_LIMIT = Percent.of(100);
    private static final Dimensionless INWARDS_OUTPUT_LIMIT = Percent.of(-10); 
    
    public AlgaeGrabberSubsystem(){
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
        grabberMotor.configure(grabberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setReference(Angle reference){
        angleController.setReference(reference.times(ANGLE_CONVERSION_FACTOR).in(Rotations), ControlType.kPosition);
    }

    public void setGrabberCurrentLimit(int currentLimit){
        SharedUtils.setCurrentLimit(grabberMotor, currentLimit);
    }

    public Angle getPosition(){
        return Rotations.of(angleMotor.getEncoder().getPosition()).div(ANGLE_CONVERSION_FACTOR);
    }

    public void setGrabberPower(Dimensionless power){
        grabberMotor.set(power.magnitude());
    }

    public AngularVelocity getGrabberVelocity(){
        return Rotations.per(Minute).of(grabberMotor.getEncoder().getVelocity());
    }

    public Command spitAlgae(){
        return this.runEnd(()->{
            grabberMotor.set(1);
            setGrabberCurrentLimit(40);
        },
        ()->{
            grabberMotor.stopMotor();
            setGrabberCurrentLimit(30);
        });
    }

    public Command resetAlgaeState(){
        return this.run(()->{
            angleController.setReference(0, ControlType.kPosition);
            grabberMotor.set(0);
        });
    }
}
