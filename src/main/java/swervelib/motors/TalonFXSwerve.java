package swervelib.motors;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.simulation.ctre.PhysicsSim;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * {@link com.ctre.phoenix.motorcontrol.can.TalonFX} Swerve Motor. Made by Team 1466 WebbRobotics.
 */
public class TalonFXSwerve extends SwerveMotor {  
  private final TalonFXConfiguration configuration          = new TalonFXConfiguration();  
  private final boolean              absoluteEncoder        = false;  
  WPI_TalonFX motor;  
  private double  positionConversionFactor = 1;  
  private boolean configChanged            = true;  
  private double  nominalVoltage           = 12.0;

  public TalonFXSwerve(WPI_TalonFX motor, boolean isDriveMotor) {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;

    setFactoryDefaults();
    clearStickyFaults();

    if (SwerveDriveTelemetry.isSimulation) {
      PhysicsSim.getInstance().addTalonFX(motor, .25, 6800);
    }
  }

  public TalonFXSwerve(int id, String canbus, boolean isDriveMotor) {
    this(new WPI_TalonFX(id, canbus), isDriveMotor);
  }
  
  public TalonFXSwerve(int id, boolean isDriveMotor) {
    this(new WPI_TalonFX(id), isDriveMotor);
  }

  @Override
  public void setFactoryDefaults() {
    if (!factoryDefaultOccurred) {
      motor.configFactoryDefault();
      motor.setSensorPhase(true);
      motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      motor.configNeutralDeadband(0.001);
    }
  }
  
  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }

  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder) {
    // Do not support.
    return this;
  }

  @Override
  public void configureIntegratedEncoder(double positionConversionFactor) {
    this.positionConversionFactor = positionConversionFactor;
    // Taken from democat's library.
    // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L16
    configureCANStatusFrames(250);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
   */
  public void configureCANStatusFrames(int CANStatus1) {
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current Measurement
   */
  public void configureCANStatusFrames(int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus8,
                                       int CANStatus10, int CANStatus12, int CANStatus13, int CANStatus14,
                                       int CANStatus21, int CANStatusCurrent) {
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CANStatus2);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, CANStatus3);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, CANStatus4);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, CANStatus8);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, CANStatus10);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, CANStatus12);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, CANStatus13);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, CANStatus14);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, CANStatus21);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, CANStatusCurrent);

    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config) {
    configuration.slot0.kP = config.p;
    configuration.slot0.kI = config.i;
    configuration.slot0.kD = config.d;
    configuration.slot0.kF = config.f;
    configuration.slot0.integralZone = config.iz;
    configuration.slot0.closedLoopPeakOutput = config.output.max;
    configChanged = true;
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    // Do nothing
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    motor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }
  
  @Override
  public void setInverted(boolean inverted) {
    Timer.delay(1);
    motor.setInverted(inverted);
  }

  @Override
  public void burnFlash() {
    if (configChanged) {
      motor.configAllSettings(configuration, 0);
      configChanged = false;
    }
  }

  @Override
  public void set(double percentOutput) {
    motor.set(percentOutput);
  }

  public double convertToNativeSensorUnits(double setpoint, double position) {
    setpoint = isDriveMotor ? setpoint * .1 : SwerveMath.placeInAppropriate0To360Scope(position, setpoint);
    return setpoint / positionConversionFactor;
  }

  @Override
  public void setReference(double setpoint, double feedforward) {
    setReference(setpoint, feedforward, getPosition());
  }

  @Override
  public void setReference(double setpoint, double feedforward, double position) {
    if (SwerveDriveTelemetry.isSimulation) {
      PhysicsSim.getInstance().run();
    }

    burnFlash();

    if (isDriveMotor) {
      motor.set(TalonFXControlMode.Velocity, convertToNativeSensorUnits(setpoint, position), DemandType.ArbitraryFeedForward, feedforward / nominalVoltage);
    } else {
      motor.set(TalonFXControlMode.Position, convertToNativeSensorUnits(setpoint, position), DemandType.ArbitraryFeedForward, feedforward);
    }
  }

  @Override
  public double getVelocity() {
    return (motor.getSelectedSensorVelocity() * 10) * positionConversionFactor;
  }

  @Override
  public double getPosition() {
    return motor.getSelectedSensorPosition() * positionConversionFactor;
  }

  @Override
  public void setPosition(double position) {
    if (!absoluteEncoder && !SwerveDriveTelemetry.isSimulation) {
      position = position < 0 ? (position % 360) + 360 : position;
      motor.setSelectedSensorPosition(position / positionConversionFactor, 0, 0);
    }
  }
  
  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    configuration.voltageCompSaturation = nominalVoltage;
    configChanged = true;
    this.nominalVoltage = nominalVoltage;
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    configuration.supplyCurrLimit.currentLimit = currentLimit;
    configuration.supplyCurrLimit.enable = true;
    configChanged = true;
  }
  
  @Override
  public void setLoopRampRate(double rampRate) {
    configuration.closedloopRamp = rampRate;
    configuration.openloopRamp = rampRate;
    configChanged = true;
  }
  
  @Override
  public Object getMotor() {
    return motor;
  }

  @Override
  public boolean isAttachedAbsoluteEncoder() {
    return absoluteEncoder;
  }
}
