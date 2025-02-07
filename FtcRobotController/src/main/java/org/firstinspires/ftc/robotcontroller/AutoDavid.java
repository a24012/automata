package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name="IntoTheDeepAutonomous", group="Linear Opmode")
public class AutoDavid extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MID = null;
    private DcMotor MIA = null;
    private DcMotor MDD = null;
    private DcMotor MDA = null;
    private OpenCvCamera camera;

    // Constante para convertir pulgadas a "ticks"
    static final double TICKS_PER_INCH = 273.35;

    @Override
    public void runOpMode() {

        // Inicializar hardware
        MID = hardwareMap.get(DcMotor.class, "MID");
        MIA = hardwareMap.get(DcMotor.class, "MIA");
        MDD = hardwareMap.get(DcMotor.class, "MDD");
        MDA = hardwareMap.get(DcMotor.class, "MDA");

        // Establecer ZeroPowerBehavior a BRAKE para que los motores se detengan en seco
        MID.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        MIA.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        MDD.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        MDA.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Resetear los encoders
        MID.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MIA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MID.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MIA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MDA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();




        moverAdelante(24);      // Moverse hacia adelante 24 pulgadas
        moverAtras(24);         // Moverse hacia atrás 24 pulgadas
        desplazarIzquierda(24); // Moverse lateralmente a la izquierda 24 pulgadas
        desplazarDerecha(24);   // Moverse lateralmente a la derecha 24 pulgadas
        diagonalAdelanteIzquierda(24);  // Movimiento diagonal hacia adelante e izquierda 24 pulgadas
        diagonalAdelanteDerecha(24);    // Movimiento diagonal hacia adelante y derecha 24 pulgadas
        diagonalAtrasIzquierda(24);     // Movimiento diagonal hacia atrás e izquierda 24 pulgadas
        diagonalAtrasDerecha(24);       // Movimiento diagonal hacia atrás y derecha 24 pulgadas
        rotarHorario(360);              // Rotación en sentido horario 360 grados
        rotarAntihorario(360);          // Rotación en sentido antihorario 360 grados

    }

    // Métodos de movimientos
    private void moverAdelante(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(-targetPosition, -targetPosition, targetPosition, targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void moverAtras(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(targetPosition, targetPosition, -targetPosition, -targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void desplazarIzquierda(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(targetPosition, -targetPosition, targetPosition, -targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void desplazarDerecha(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(-targetPosition, targetPosition, -targetPosition, targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void diagonalAdelanteIzquierda(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(0, -targetPosition, targetPosition, 0);
        setMotorPower(1);
        esperarMotores();
    }

    private void diagonalAdelanteDerecha(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(-targetPosition, 0, 0, targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void diagonalAtrasIzquierda(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(targetPosition, 0, 0, -targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void diagonalAtrasDerecha(double distanciaInch) {
        int targetPosition = (int)(distanciaInch * TICKS_PER_INCH);
        setTargetPosition(0, targetPosition, -targetPosition, 0);
        setMotorPower(1);
        esperarMotores();
    }

    private void rotarHorario(double grados) {
        int targetPosition = (int)(grados * TICKS_PER_INCH / 360);
        setTargetPosition(-targetPosition, -targetPosition, targetPosition, targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    private void rotarAntihorario(double grados) {
        int targetPosition = (int)(grados * TICKS_PER_INCH / 360);
        setTargetPosition(targetPosition, targetPosition, targetPosition, targetPosition);
        setMotorPower(1);
        esperarMotores();
    }

    // Métodos auxiliares
    private void setTargetPosition(int midPos, int miaPos, int mddPos, int mdaPos) {
        MID.setTargetPosition(midPos);
        MIA.setTargetPosition(miaPos);
        MDD.setTargetPosition(mddPos);
        MDA.setTargetPosition(mdaPos);

        MID.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MIA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MDD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MDA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setMotorPower(double power) {
        MID.setPower(power);
        MIA.setPower(power);
        MDD.setPower(power);
        MDA.setPower(power);
    }

    private void esperarMotores() {
        while (opModeIsActive() && MID.isBusy() && MIA.isBusy() && MDD.isBusy() && MDA.isBusy()) {
            telemetry.addData("Path", "Moving to position");
            telemetry.update();
        }
        setMotorPower(0);
    }
}