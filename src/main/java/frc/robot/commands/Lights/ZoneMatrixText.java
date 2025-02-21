package frc.robot.commands.Lights;

import com.lumynlabs.domain.led.MatrixTextScrollDirection;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Lights.LEDSubsystem;

public class ZoneMatrixText extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String zoneID;
    private final String text;
    private final Color color;
    private final MatrixTextScrollDirection direction;
    private final int delay;
    private final boolean oneShot;

    public ZoneMatrixText(LEDSubsystem subsystem, String zoneID, String text, Color color, MatrixTextScrollDirection direction, int delay, boolean oneShot) {
        this.ledSubsystem = subsystem;
        this.zoneID = zoneID;
        this.text = text;
        this.color = color;
        this.direction = direction;
        this.delay = delay;
        this.oneShot = oneShot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_ZoneMatrixText(zoneID, text, color, direction, delay, oneShot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}