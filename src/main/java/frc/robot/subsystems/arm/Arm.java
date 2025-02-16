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
  private final TalonFX armKraken = new TalonFX(11);

  public final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0)
        .withSlot(0);

private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(.5).per(Second),        // Use default ramp rate (1 V/s) & change volts per sec
         Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout 
         Seconds.of(2.5),        // Use default timeout (10 s) & lower nubmer ples
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
            .withMotionMagicAcceleration(2)
            .withMotionMagicCruiseVelocity(1);

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
            .withSensorToMechanismRatio(18); // 18(gear ratio) * 360(degrees)
    var slot0Config =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKA(0.53584)
            .withKG(0.397)
            .withKP(1.0)
            .withKS(0.16558)
            .withKV(1.6063);

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
    return runOnce(() -> {
        motionMagicRequest.withPosition(pos);
        armKraken.setControl(motionMagicRequest);
    });
  }

  public Command runArm(double speed){
    return run(() -> {
        armKraken.set(speed);
    }); 
  }

  public Command zeroArm() {
    return runOnce(() -> {
        armKraken.setPosition(0);
    });
  }

  public Command setArmEncoderStow() {
    return runOnce(() -> {
        armKraken.setPosition(0.37);
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
