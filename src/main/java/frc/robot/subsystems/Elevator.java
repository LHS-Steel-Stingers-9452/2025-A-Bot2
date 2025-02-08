package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {

  private final TalonFX elevatorKraken = new TalonFX(0);

  public final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0)
        .withSlot(0);

private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.25).per(Second),        // Use default ramp rate (1 V/s) & change volts per sec
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout 
         Seconds.of(20),        // Use default timeout (10 s) & lower nubmer ples
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> elevatorKraken.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

  public Elevator() {


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
    var feedbackConfig =
        new FeedbackConfigs()
            .withSensorToMechanismRatio(56.0114554209); // 9(gear ratio) * 1.981(sprocket diamater) * pi
    var slot0Config =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKA(0)
            .withKG(0)
            .withKP(0)
            .withKS(0)
            .withKV(0);

    var talonFXConfig = 
        new TalonFXConfiguration()
        .withCurrentLimits(currentLimitConfig)
        .withMotorOutput(motorOutputConfig)
        .withFeedback(feedbackConfig)
        .withSlot0(slot0Config);
    elevatorKraken.getConfigurator().apply(talonFXConfig);
  }

  public Command setPosition(double pos) {
    motionMagicRequest.withPosition(pos);
    return run(() -> {
        elevatorKraken.setControl(motionMagicRequest);
    });
  }
  
  
}
