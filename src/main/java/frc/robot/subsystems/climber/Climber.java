package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {


    private final TalonFX  climberKraken = new TalonFX(0);
    
    public Climber() {

        var motorOutputConfig =
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        var currentLimitConfig =
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(50)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        var talonFXConfig = 
            new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfig)
            .withMotorOutput(motorOutputConfig);
            
        
        climberKraken.getConfigurator().apply(talonFXConfig);
    }
   

    public void stopIntake() {
        climberKraken.set(0);
    }

    public Command runIntake(double speed){
        return run(() -> {
            climberKraken.set(speed);
        });
    }
}
