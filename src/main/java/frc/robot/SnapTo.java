package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class SnapTo {    
    public double snapAngle = 0;
    public boolean isAtAngle = true;
    public PIDController pid = new PIDController(0.2, 0, 0);
    public Supplier<Double> gyroMeasurement;
    public Consumer<Double> turnMethod;
    public double startTimeOfSnap;
    public double timeToSnap;

    public SnapTo() {
        this.pid.enableContinuousInput(0, 360);
        this.pid.setTolerance(1);         
    }

    public void setPidConfig(double p, double i, double d) {
        this.pid.setP(p);
        this.pid.setI(i);
        this.pid.setD(d);                
    }

    public void setTolerance(double tolerance) {
        this.pid.setTolerance(tolerance);     
    }

    public void setSnapTo(double angle) {
        this.snapAngle = angle;
        pid.setSetpoint(snapAngle);                  
        this.startTimeOfSnap = Timer.getFPGATimestamp();
        this.timeToSnap = 0;
    }

    private double pinRange(double in, double min, double max) {
        if (Math.abs(in) > max) {
          return in < 0 ? -max : max;
        }
        if (Math.abs(in) < min) {
          return in < 0 ? -min : min;
        }  
        return in;
    }

    public double calcPowerForSnapping(double min, double max) {
        if (this.gyroMeasurement == null) {
            throw new RuntimeException("Need to set a measurement function!");
        }
        double output = pid.calculate(this.gyroMeasurement.get() % 360);
        output = pinRange(output, min, max);
        if (pid.atSetpoint()) {
          output = 0;
          if (this.timeToSnap == 0) {
            this.timeToSnap = Timer.getFPGATimestamp() - this.startTimeOfSnap;
          }
        }
        return output;
    }

    public double doSnapToCycle(double min, double max) {
        double output = calcPowerForSnapping(min, max);
        if (this.turnMethod == null) {
            throw new RuntimeException("Need to set a turn method!");
        }
        this.turnMethod.accept(output);        
        return output;
    }

}
