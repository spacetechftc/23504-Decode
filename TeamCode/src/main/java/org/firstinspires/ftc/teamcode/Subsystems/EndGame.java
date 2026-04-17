package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class EndGame implements Subsystem {

    public static final EndGame INSTANCE = new EndGame();
    private EndGame() {}

    private Servo endgame_left;
    private Servo endgame_right;

    public static double pos_left = 0.47;
    public static double pos_right = 0.37;

    public void endGameOn() {
        endgame_left.setPosition(pos_left);
        endgame_right.setPosition(pos_right);
    }

    public void endGameOff() {
        endgame_left.setPosition(0.07);
        endgame_right.setPosition(0);
    }

    @Override
    public void initialize() {
        endgame_left = ActiveOpMode.hardwareMap().get(Servo.class, "endgame_left");
        endgame_left.setDirection(Servo.Direction.REVERSE);
        endgame_right = ActiveOpMode.hardwareMap().get(Servo.class, "endgame_right");
    }

    @Override
    public void periodic() {

    }
}
