package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.Direction;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Led implements Subsystem {

    public Servo led_shooter;
    public Servo led_back;
    public GoBildaPrismDriver led_prism;

    public static final Led INSTANCE = new Led();
    private Led() {}

    public static double OFF = 0;
    public static double RED = 0.279;
    public static double GREEN = 0.500;
    public static double YELLOW = 0.36;

    private double lastBackPos = -1;
    private double lastShooterPos = -1;
    public enum PrismState {
        RED_FILL,
        RED_SNAKE,
        GREEN_SNAKE,
        GREEN_SOLID,
        GREEN_BLINK,
        RED_SOLID,
        RED_BLINK,
        BLUE_BLINK,
        OFF
    }

    private PrismState currentPrismState = null;
    @Override
    public void initialize() {
        led_back = ActiveOpMode.hardwareMap().get(Servo.class, "led_back");
        led_shooter = ActiveOpMode.hardwareMap().get(Servo.class, "led_shooter");
        led_prism = ActiveOpMode.hardwareMap().get(GoBildaPrismDriver.class, "prism");
    }

    public void setLedBack(double pos) {
        if (Math.abs(pos - lastBackPos) > 0.0001) {
            led_back.setPosition(pos);
            lastBackPos = pos;
        }
    }

    public void setLedShooter(double pos) {
        if (Math.abs(pos - lastShooterPos) > 0.0001) {
            led_shooter.setPosition(pos);
            lastShooterPos = pos;
        }
    }

    // Leds Intake

    public void setBackOff() {
        setLedBack(OFF);
    }
    public void setBackRed() {
        setLedBack(RED);
    }
    public void setBackGreen() {
        setLedBack(GREEN);
    }
    public void setBackYellow() {
        setLedBack(YELLOW);
    }

    // Led Shooter
    public void setShooterOn() {
        setLedShooter(0.85);
    }
    public void setShooterOff() {
        setLedShooter(0);
    }

    public void setPrismState(PrismState state) {
        if (state == currentPrismState) return;

        currentPrismState = state;
        led_prism.clearAllAnimations();

        switch (state) {
            case RED_SNAKE:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createSnakes(Color.RED)
                );
                break;

            case RED_FILL:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createFill()
                );
                break;

            case GREEN_SNAKE:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createSnakes(Color.GREEN)
                );
                break;

            case GREEN_SOLID:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createSolid(Color.GREEN)
                );
                break;

            case GREEN_BLINK:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createBlink(Color.GREEN)
                );
                break;

            case RED_SOLID:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createSolid(Color.RED)
                );
                break;

            case RED_BLINK:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createBlink(Color.RED)
                );
                break;

            case BLUE_BLINK:
                led_prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        createBlink(Color.BLUE)
                );
                break;

            case OFF:
                break;
        }
    }

    private PrismAnimations.Solid createSolid(Color color) {
        PrismAnimations.Solid solid = new PrismAnimations.Solid(color);
        solid.setBrightness(100);
        solid.setStartIndex(0);
        solid.setStopIndex(24);
        return solid;
    }

    private PrismAnimations.Blink createBlink(Color color) {
        PrismAnimations.Blink blink = new PrismAnimations.Blink();
        blink.setPrimaryColor(color);
        blink.setSecondaryColor(Color.TRANSPARENT);
        blink.setBrightness(100);
        blink.setStartIndex(0);
        blink.setStopIndex(24);
        blink.setPeriod(600);
        blink.setPrimaryColorPeriod(300);
        return blink;
    }

    private PrismAnimations.Snakes createSnakes(Color color) {
        PrismAnimations.Snakes snake = new PrismAnimations.Snakes();
        snake.setSnakeLength(5);
        snake.setSpacingBetween(2);
        snake.setColors(color);
        snake.setBackgroundColor(Color.TRANSPARENT);
        snake.setSpeed(0.52f);
        snake.setDirection(Direction.Forward);
        snake.setRepeatAfter(12);
        snake.setBrightness(100);
        return snake;
    }
    private PrismAnimations.SingleFill createFill() {
        PrismAnimations.SingleFill fill = new PrismAnimations.SingleFill();
        fill.setColors(Color.WHITE, Color.RED, Color.GREEN);
        fill.setSpeed(0.65f);
        fill.setPeriod(1000);
        fill.setPixelLength(3);
        fill.setDirection(Direction.Forward);
        fill.setStyle(PrismAnimations.SingleFill.SingleFillStyle.FillIn);
        return fill;
    }
}