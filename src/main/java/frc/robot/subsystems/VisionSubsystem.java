package frc.robot.subsystems;

import java.awt.Color;

import org.opencv.core.Mat;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        new Thread(() -> {
            run();
        }, "Vision Thread").start();
    }

    private void run() {
        int width = 120;
        int height = 120;

        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setResolution(width, height);
        camera.setFPS(30);

        CvSink cvSink = CameraServer.getVideo(camera);
        CvSource outputStream = CameraServer.putVideo("Test 123", width, height);

        Mat source = new Mat();
        while (!Thread.interrupted()) {
            if (cvSink.grabFrame(source) == 0) {
                continue;
            }
            Mat output = source.clone();

            // double[] rgbOut = new double[4];
            float[] hsvOut = new float[3];

            for (int row = 0; row < output.rows(); row++) {
                for (int col = 0; col < output.cols(); col++) {
                    double[] rgbOut = source.get(row, col);
                    int blue = (int) rgbOut[0];
                    int green = (int) rgbOut[1];
                    int red = (int) rgbOut[2];

                    Color.RGBtoHSB(red, green, blue, hsvOut);
                    float hue = hsvOut[0] * 360;
                    float saturation = hsvOut[1];
                    float brightness = hsvOut[2];

                    if (
                        hue > 190 &&
                        hue < 240 &&
                        saturation < 1 &&
                        brightness < 0.8 &&
                        brightness > 0.2
                    ) {
                        output.put(row, col, blue, green, red);
                    } else {
                        output.put(row, col, 0, 0, 0);
                    }
                    // output.put(row, col, 255 - blue, 255 - green, 255 - red);
                }
            }
            // Imgproc.invert
            outputStream.putFrame(output);
        }
    }
}
