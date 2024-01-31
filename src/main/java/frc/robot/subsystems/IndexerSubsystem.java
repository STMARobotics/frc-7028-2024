package frc.robot.subsystems;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.ColorSensorReader;

public class IndexerSubsystem extends SubsystemBase {
  public boolean hasRing;

  private final CANSparkMax indexer = new CANSparkMax(DEVICE_ID_INDEXER, kBrushless);
  private SparkPIDController pidController;
  private RelativeEncoder encoder;

  private final ColorSensorReader colorSensorReader = new ColorSensorReader();
  private final Notifier colorSensorNotifier = new Notifier(colorSensorReader);
  private final Debouncer fullSensorDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  public IndexerSubsystem() {
    indexer.restoreFactoryDefaults();
    indexer.enableVoltageCompensation(12);
    indexer.setOpenLoopRampRate(0.1);
    indexer.setClosedLoopRampRate(0.1);
    indexer.setInverted(true);
    pidController = indexer.getPIDController();
    pidController.setP(IndexerConstants.BELT_kP);
    pidController.setFF(0.00009);
    encoder = indexer.getEncoder();
    indexer.burnFlash();
    indexer.setIdleMode(IdleMode.kCoast);
  }
}
