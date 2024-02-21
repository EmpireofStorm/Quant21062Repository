package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp(name = "HuskyLens Example", group = "Examples")
public class Camera extends LinearOpMode {

    private I2cDeviceSynch huskyLens;

    @Override
    public void runOpMode() {
        // Initialize the HuskyLens device
        huskyLens = hardwareMap.get(I2cDeviceSynch.class, "huskylens");
        huskyLens.setI2cAddress(I2cAddr.create7bit(0x32));
        huskyLens.engage();

        waitForStart();

        while (opModeIsActive()) {
            try {
                // Example: Reading data from HuskyLens (adjust register address as needed)
                byte[] huskyLensData = huskyLens.read(0x00, 10); // Read 10 bytes from register 0x00

                // Process the data to output center coordinates
                outputCenterCoordinates(huskyLensData);

                telemetry.update();
                idle();
            } catch (Exception e) {
                telemetry.addData("Error", "An error occurred: " + e.getMessage());
                telemetry.update();
            }
        }
    }

    private void outputCenterCoordinates(byte[] data) {
        if (data.length >= 10) { // Ensure data array is the expected size
            int x = ((data[2] & 0xFF) << 8) | (data[3] & 0xFF);
            int y = ((data[4] & 0xFF) << 8) | (data[5] & 0xFF);
            int width = data[6] & 0xFF;
            int height = data[7] & 0xFF;

            int centerX = x + (width / 2);
            int centerY = y + (height / 2);

            // Output the center coordinates to telemetry
            telemetry.addData("Center X", centerX);
            telemetry.addData("Center Y", centerY);
        } else {
            telemetry.addData("Error", "Data array is too short.");
        }
    }
}