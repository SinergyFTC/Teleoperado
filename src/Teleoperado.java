// Importações de bibliotecas da FTC necessárias para o funcionamento do opmode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Nome do OpMode que aparecerá na Control Hub/Driver Station
@TeleOp(name="teleopGeral")

// Classe principal que herda de LinearOpMode
public class Teleoperado extends LinearOpMode {

    // Declaração de variáveis globais (motores, servos, etc.)
    private ElapsedTime timer = new ElapsedTime(); // Timer usado para ações temporizadas
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Motores auxiliares (intake, garra, slides)
    private DcMotor corehexintake = null;
    private DcMotor corehexgarra = null;
    private DcMotor slideintake = null;
    private DcMotor slideouttake = null;

    // Servos
    private Servo servo = null;
    private CRServo motorgarra1;
    private CRServo motorgarra2;
    private Servo servoGarra = null;

    // Motores extras
    private DcMotor motor;
    private DcMotor motor2;
    private DcMotor motor3;

    // Controle de posição por encoder
    private int targetPosition = 0;
    private final int INCREMENTO = 1000; // incremento de posição para cada clique

    // Variáveis de controle da garra
    boolean garraAtiva = false;
    private long tempoAcao = 0;
    private int duracao = 0;
    private double power1 = 0;
    private double power2 = 0;

    // Variáveis para detectar clique único (debounce dos botões)
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // Mapeamento de hardware (conecta nomes do Configurator aos objetos do código)
        leftFrontDrive = hardwareMap.dcMotor.get("FL");
        leftBackDrive = hardwareMap.dcMotor.get("BL");
        rightFrontDrive = hardwareMap.dcMotor.get("FRONT RIGHT");
        rightBackDrive = hardwareMap.dcMotor.get("BR");

        // Ajuste das direções dos motores do drivetrain
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Motores auxiliares
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motorgarra1 = hardwareMap.get(CRServo.class, "MotorGarra1");
        motorgarra2 = hardwareMap.get(CRServo.class, "MotorGarra2");
        servoGarra = hardwareMap.get(Servo.class, "ServoGarra");

        // Configurações de direção e modos dos motores com encoder
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Aguarda start do Driver Station
        waitForStart();
        if (isStopRequested()) return;
        timer.reset();

        // LOOP PRINCIPAL do TeleOp
        while (opModeIsActive()) {

            // ----- CONTROLE DO DRIVER 1 (movimentação do robô) -----
            double axial = -gamepad1.left_stick_y;  // Frente/Trás
            double lateral = -gamepad1.left_stick_x; // Esquerda/Direita
            double yaw = -gamepad1.right_stick_x;    // Rotação no próprio eixo

            // Cálculo da potência dos motores (mecanum drive básico)
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalização (limitando a velocidade máxima)
            double max = Math.max(Math.abs(leftFrontPower),
                        Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftBackPower),
                        Math.abs(rightBackPower))));

            if (max > 0.3) { // limita para 30% da velocidade
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // ----- CONTROLE DO DRIVER 1 (motores com encoder) -----
            if (gamepad1.a && !lastA) {
                targetPosition += INCREMENTO; // Aumenta posição
            }
            if (gamepad1.b && !lastB) {
                targetPosition -= INCREMENTO; // Diminui posição
            }
            if (gamepad1.y && !lastY) {
                targetPosition = motor.getCurrentPosition(); // Mantém posição
            }

            // ----- CONTROLE DO DRIVER 2 (garra temporizada) -----
            if (gamepad2.x) {
                timer.reset();
                garraAtiva = true;
                duracao = 500; // tempo em ms
                power1 = 0.5;
                power2 = 0.9;
            }
            else if (gamepad2.a) {
                timer.reset();
                garraAtiva = true;
                duracao = 300;
                power1 = -1.0;
                power2 = 1.0;
            }
            else if (gamepad2.y) {
                timer.reset();
                garraAtiva = true;
                duracao = 300;
                power1 = 1.0;
                power2 = -1.0;
            }
            else if (gamepad2.b) {
                timer.reset();
                garraAtiva = true;
                duracao = 450;
                power1 = -0.9;
                power2 = -1.0;
            }

            // ----- CONTROLE DO DRIVER 1 (motor extra com triggers) -----
            if (gamepad1.right_trigger > 0.1) {
                motor3.setPower(0.5);
            }
            if (gamepad1.left_trigger > 0.1) {
                motor3.setPower(-0.5);
            }
            if (gamepad1.right_bumper) {
                motor3.setPower(0.0);
            }

            // ----- CONTROLE DO DRIVER 2 (servo da garra tipo pinça) -----
            if (gamepad2.left_trigger > 0.0) {
                servoGarra.setPosition(0); // abre a garra
            }
            if (gamepad2.right_trigger > 0.0) {
                servoGarra.setPosition(0.3); // fecha a garra
            }

            // ----- EXECUÇÃO DA GARRA TEMPORIZADA -----
            if (garraAtiva) {
                if (timer.milliseconds() < duracao) {
                    motorgarra1.setPower(power1);
                    motorgarra2.setPower(power2);
                } else {
                    motorgarra1.setPower(0);
                    motorgarra2.setPower(0);
                    garraAtiva = false; // encerra a ação
                }
            }

            // ----- EXECUÇÃO DOS MOTORES COM ENCODER -----
            motor.setTargetPosition(targetPosition);
            motor2.setTargetPosition(targetPosition);

            if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motor2.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
                motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setPower(1);    // motor principal com força total
            motor2.setPower(0.5); // motor secundário com metade da força

            // ----- EXECUÇÃO DO DRIVE -----
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // ----- TELEMETRIA -----
            telemetry.addData("Target Pos", targetPosition);
            telemetry.addData("Motor1 Pos", motor.getCurrentPosition());
            telemetry.addData("Motor2 Pos", motor2.getCurrentPosition());
            telemetry.update();

            // Atualiza os estados anteriores dos botões
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastY = gamepad1.y;
        }
    }
}
