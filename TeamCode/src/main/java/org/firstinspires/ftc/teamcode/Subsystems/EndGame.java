package org.firstinspires.ftc.teamcode.Subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class EndGame implements Subsystem {

    public ServoEx endgame_left;
    public ServoEx endgame_right;

    @Override
    public void initialize() {
        endgame_left = ActiveOpMode.hardwareMap().get(ServoEx.class, "endgame_left");
        endgame_left.reversed();
        endgame_right = ActiveOpMode.hardwareMap().get(ServoEx.class, "endgame_right");
    }

    @Override
    public void periodic() {

    }
}
