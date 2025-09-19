package OpModes.subsystems;

import androidx.annotation.NonNull;

import java.util.Set;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


public class Claw implements Subsystem {
    public static final Claw INSTANCE = new Claw();

    private Claw() {
    }

    private ServoEx servo = new ServoEx("claw_servo");

    public Command open = new SetPosition(servo, 0.1).requires(this);
    public Command close = new SetPosition(servo, 0.2).requires(this);

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return Subsystem.super.getDefaultCommand();
    }

    @Override
    public void initialize() {
        Subsystem.super.initialize();
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();
    }

    @NonNull
    @Override
    public Set<Subsystem> getSubsystems() {
        return Subsystem.super.getSubsystems();
    }
}
