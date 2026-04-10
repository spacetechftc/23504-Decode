package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "Prism Status Config", group = "OpModes Tests")
public class testPrism extends LinearOpMode {

    GoBildaPrismDriver prism;

    @Override
    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // Verde sólido
        PrismAnimations.Solid greenSolid = new PrismAnimations.Solid(Color.GREEN);
        greenSolid.setBrightness(100);
        greenSolid.setStartIndex(0);
        greenSolid.setStopIndex(12);

        // Verde piscando
        PrismAnimations.Blink greenBlink = new PrismAnimations.Blink();
        greenBlink.setPrimaryColor(Color.GREEN);
        greenBlink.setSecondaryColor(Color.TRANSPARENT); // apagado entre piscadas
        greenBlink.setBrightness(100);
        greenBlink.setStartIndex(0);
        greenBlink.setStopIndex(12);
        greenBlink.setPeriod(600);
        greenBlink.setPrimaryColorPeriod(300);

        // Vermelho sólido
        PrismAnimations.Solid redSolid = new PrismAnimations.Solid(Color.RED);
        redSolid.setBrightness(100);
        redSolid.setStartIndex(0);
        redSolid.setStopIndex(12);

        // Vermelho piscando
        PrismAnimations.Blink redBlink = new PrismAnimations.Blink();
        redBlink.setPrimaryColor(Color.RED);
        redBlink.setSecondaryColor(Color.TRANSPARENT);
        redBlink.setBrightness(100);
        redBlink.setStartIndex(0);
        redBlink.setStopIndex(12);
        redBlink.setPeriod(600);
        redBlink.setPrimaryColorPeriod(300);

        // Azul piscando
        PrismAnimations.Blink blueBlink = new PrismAnimations.Blink();
        blueBlink.setPrimaryColor(Color.BLUE);
        blueBlink.setSecondaryColor(Color.TRANSPARENT);
        blueBlink.setBrightness(100);
        blueBlink.setStartIndex(0);
        blueBlink.setStopIndex(12);
        blueBlink.setPeriod(600);
        blueBlink.setPrimaryColorPeriod(300);

        waitForStart();

        // Salva cada estado em um artboard
        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, greenSolid);
        prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_0);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, greenBlink);
        prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_1);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, redSolid);
        prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_2);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, redBlink);
        prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_3);

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, blueBlink);
        prism.saveCurrentAnimationsToArtboard(Artboard.ARTBOARD_4);

        while (opModeIsActive()) {
            // teste rápido
            if (gamepad1.a) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);
            if (gamepad1.b) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_1);
            if (gamepad1.x) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2);
            if (gamepad1.y) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3);
            if (gamepad1.dpad_up) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4);

            telemetry.addLine("A = verde sólido");
            telemetry.addLine("B = verde piscando");
            telemetry.addLine("X = vermelho sólido");
            telemetry.addLine("Y = vermelho piscando");
            telemetry.addLine("Dpad Up = azul piscando");
            telemetry.update();
        }
    }
}