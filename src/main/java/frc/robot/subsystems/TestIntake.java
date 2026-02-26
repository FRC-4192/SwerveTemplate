package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestIntake extends SubsystemBase {
    private TalonFX intake;
    private double power = 1;

    public TestIntake() {
        intake = new TalonFX(33);

        TalonFXConfiguration take = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withStatorCurrentLimit(40))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        intake.getConfigurator().apply(take);
        intake.setControl(new Follower(33, MotorAlignmentValue.Opposed));

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("intake Current", intake.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("shooter Current w/ Low Pass Filter",
        // filteredCurrent);
        SmartDashboard.putNumber("intake velo", intake.getVelocity().getValueAsDouble());
    }

    public Command runIntake() {
        return run(() -> runIntakeRaw(power));
    }

    public Command runOuttake() {
        return run(() -> runIntakeRaw(-power));
    }

    public void runIntakeRaw(double power) {
        intake.set(power);
    }

    public Command runIntakeOnce() {
        return runOnce(() -> runIntakeRaw(power));
    }

}
