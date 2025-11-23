package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.os.Environment;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "CSV Recorder Template", group = "Utility")
public class DataOp extends LinearOpMode {
    private static final long RECORD_INTERVAL_MS = 20; // 50Hz sampling
    private FileWriter csvWriter;
    private long startTime;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // Initializare fisier CSV
            String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_data.csv";
            csvWriter = new FileWriter(filePath);

            // Header aici - coloanele pe care vrei sa le inregistrezi
            csvWriter.write(
                    "Time,Voltage\n"
            );


            telemetry.addData("Status", "Ready to record");
            telemetry.addData("File Path", filePath);
            telemetry.update();

            waitForStart();
            startTime = System.currentTimeMillis();
            long lastRecordTime = 0;

            while (opModeIsActive()) {
                long currentTime = System.currentTimeMillis();


                if (currentTime - lastRecordTime >= RECORD_INTERVAL_MS) {
                    recordData();
                    lastRecordTime = currentTime;
                }

                // Aici vine codul de control al robotului ( du-te la bla bla )

                telemetry.addData("Recording", "Active");
                telemetry.update();
            }

        } catch (IOException e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
        } finally {
            if (csvWriter != null) {
                try {
                    csvWriter.close();
                } catch (IOException e) {
                    // Ignora
                }
            }
        }
    }

    private void recordData() throws IOException {
        double timeSeconds = (System.currentTimeMillis() - startTime) / 1000.0;

        // Aici bagi datele pe care vrei sa le inregistrezi
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Aici se scriu datele in fisier - ASIGURA-TE CA ORDINEA COLOANELOR E CORECTA
        csvWriter.write(String.format(
                "%.3f,%.2f\n",
                timeSeconds, voltage
        ));

        csvWriter.flush();
    }
}