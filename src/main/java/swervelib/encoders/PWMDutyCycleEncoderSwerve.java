package swervelib.encoders;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * DutyCycle encoders such as "US Digital MA3 with PWM Output, the CTRE Mag Encoder, the Rev Hex Encoder, and the AM Mag
 * Encoder." attached via a PWM lane.
 * <p>
 * Credits to
 * <a href="https://github.com/p2reneker25/2035-YAGSL/blob/main/swervelib/encoders/PWMDutyCycleEncoderSwerve.java">
 * p2reneker25</a> for building this.
 */
public class PWMDutyCycleEncoderSwerve extends SwerveAbsoluteEncoder {
  
  private final DutyCycleEncoder encoder;
  private       boolean          isInverted;

  public PWMDutyCycleEncoderSwerve(int pin) {
    encoder = new DutyCycleEncoder(pin);
  }
  
  @Override
  public void configure(boolean inverted) {
    isInverted = inverted;
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition() {
    return (isInverted ? -1.0 : 1.0) * encoder.getAbsolutePosition() * 360;
  }

  /**
   * Get the encoder object.
   *
   * @return {@link DutyCycleEncoder} from the class.
   */
  @Override
  public Object getAbsoluteEncoder() {
    return encoder;
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault() {
    // Do nothing
  }

  /**
   * Clear sticky faults on the encoder.
   */
  @Override
  public void clearStickyFaults() {
    // Do nothing
  }

}