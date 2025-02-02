package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Arm extends SubsystemBase {
// change ID
  private final TalonFX armKraken = new TalonFX(4);

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
         (volts) -> armKraken.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

  public Arm() {

    var motionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(1.5)
            .withMotionMagicCruiseVelocity(1.5);

    var motorOutputConfig =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
            

    var currentLimitConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100)
            .withSupplyCurrentLimit(50)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true);
    var feedbackConfig =
        new FeedbackConfigs()
            .withSensorToMechanismRatio(21.428571428571427);
    var slot0Config =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKA(0.031146)
            .withKG(0)
            .withKP(0.5)
            .withKS(0.031686)
            .withKV(2.4891);

    var talonFXConfig = 
        new TalonFXConfiguration()
        .withCurrentLimits(currentLimitConfig)
        .withMotorOutput(motorOutputConfig)
        .withFeedback(feedbackConfig)
        .withSlot0(slot0Config)
        .withMotionMagic(motionMagicConfig);

    armKraken.getConfigurator().apply(new TalonFXConfiguration());
    armKraken.getConfigurator().apply(talonFXConfig);
  }

  public Command setPosition(double pos) {
    motionMagicRequest.withPosition(pos);
    return run(() -> {
        armKraken.setControl(motionMagicRequest);
    });
  }

  public Command runArm(double speed){
    return run(() -> {
        armKraken.set(speed);
    }); 
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
 }
 
 public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
 }
  @Logged
public double armVoltage() {
    return armKraken.getMotorVoltage().getValueAsDouble();
}

@Logged
public double armPosition() {
    return armKraken.getPosition().getValueAsDouble();
}

@Logged
public double armSupplyCurrent() {
    return armKraken.getSupplyCurrent().getValueAsDouble();
}

@Logged
public double armStatorCurrent() {
    return armKraken.getStatorCurrent().getValueAsDouble();
}

@Logged
public double armVelocity() {
    return armKraken.getVelocity().getValueAsDouble();
}

}
