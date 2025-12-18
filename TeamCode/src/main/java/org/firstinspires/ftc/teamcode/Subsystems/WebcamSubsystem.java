package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;
import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class WebcamSubsystem implements Subsystem {

    public static WebcamSubsystem INSTANCE = new WebcamSubsystem();
    private WebcamSubsystem() {}
    private ColorBlobLocatorProcessor Purple;
    private ColorBlobLocatorProcessor Green;
    private VisionPortal portal;

    public double x;
    public double y;

    @Override
    public void initialize() {
        // Construção dos Artefatos Roxos
                Purple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        // Construção dos Artefatos Verdes
                 Green = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        // Construção do Portal
                portal = new VisionPortal.Builder()
                .addProcessor(Purple)
                .addProcessor(Green)
                .setCameraResolution(new Size(320, 240))
                .setCamera(ActiveOpMode.hardwareMap().get(WebcamName.class, "Webcam 1"))
                .build();
    }

    @Override
    public void periodic() {
        List<ColorBlobLocatorProcessor.Blob> Purpleblobs = Purple.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> Greenblobs = Green.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, Purpleblobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, Purpleblobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, Greenblobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, Greenblobs);

        for (ColorBlobLocatorProcessor.Blob b : Purpleblobs) {
            if(Purpleblobs.isEmpty()){
                x = 0;
                y = 0;
            }
            Circle circleFit = b.getCircle();
            x = b.getCircle().getX() - 160;
            y = b.getCircle().getY() - 120;
        }

        for (ColorBlobLocatorProcessor.Blob b : Greenblobs) {
            if(Greenblobs.isEmpty()){
                x = 0;
                y = 0;
            }

            Circle circleFit = b.getCircle();
            x = b.getCircle().getX() - 160;
            y = b.getCircle().getY() - 120;
        }

    }
}
