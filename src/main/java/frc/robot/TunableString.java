package frc.robot;

import java.util.function.Consumer;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TunableString {
  private String defaultValue;
  public ComplexWidget shuffleboardWidget;
  private final SendableChooser<String> autoChooser;

  /**
   * Creates a TunableDouble. It can be enabled and disabled (Use defaultValue)
   *
   * @param name
   * @param d
   * @param tunable
   */

  public TunableString(String name, String defaultValue, String[] vals, String tab) {
    this(name, defaultValue, vals, true, tab);
  }

  public TunableString(String name, String defaultValue, String[] vals, String tab, Consumer<String> onChange) {
    this(name, defaultValue, vals, tab);
    addChangeListener(onChange);
  }

  public TunableString(String name, String defaultValue, String[] vals, boolean tunable, String tab) {
    this.defaultValue = defaultValue;

    if (tunable) {
        autoChooser = new SendableChooser<>();
        shuffleboardWidget = Shuffleboard.getTab(tab).add(name, autoChooser);
        autoChooser.setDefaultOption(defaultValue, defaultValue);        
        for (String value : vals) {
            autoChooser.addOption(value, value);
        }        
    } else {
        shuffleboardWidget = null;
        autoChooser = null;
    }
  }

  public TunableString setSpot(int x, int y) {
    if (shuffleboardWidget != null) {
      shuffleboardWidget.withPosition(x, y);      

    }

    return this;
  }

  /**
   * @return Value as a double
   */
  public String getValue() {
    if (autoChooser!= null) {
        return autoChooser.getSelected();
    }
    return defaultValue;
  }

  public void addChangeListener(Consumer<String> onChange) {
    onChange.accept(getValue());
    CommandScheduler.getInstance().getDefaultButtonLoop().bind(
        new Runnable() {
          private String m_oldValue = getValue();

          @Override
          public void run() {
            String newValue = getValue();

            if (m_oldValue != newValue) {
              onChange.accept(newValue);
              m_oldValue = newValue;
            }
          }
        });
  }

}
