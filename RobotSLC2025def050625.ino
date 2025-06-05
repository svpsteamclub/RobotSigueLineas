// Conexion Driver L298N
const int ENA_PIN = 6; // PWM Motor A (Right Motor)
const int ENB_PIN = 5; // PWM Motor B (Left Motor)
const int IN1_PIN = 7; // Control Motor A (Right)
const int IN2_PIN = 8; // Control Motor A (Right)
const int IN3_PIN = 9; // Control Motor B (Left)
const int IN4_PIN = 10; // Control Motor B (Left)

// Conexion Sensores TCRT5000 (Digital Output)
const int SENSOR_RIGHT_PIN = 4;  // S1 (Derecho)
const int SENSOR_CENTER_PIN = 3; // S2 (Central)
const int SENSOR_LEFT_PIN = 2;   // S3 (Izquierdo)

//Los sensores deben ajustarse de manera que cuando la linea este centrada se active el celtral, cuando la linea este entre el central y un lateral se activen el central y el lateral correspondiente, cuando la linea este sobre el lateral solo se active el lateral.

// === Constante para Sensores ===
const bool SENSOR_READING_FOR_LINE = HIGH; // los sensores que usamos se detectan la linea en High

//////////////Cambiar solo esto//////////////////////////////////////////////

/////////////////////// === Valores para control PID === //////////////////////

int baseSpeed = 200; // Velocidad base, 200 es un valor bueno (0-255)

float Kp = 30.0f;  //Controla que tanto corrige el robot, Comenzar con 30 e irlo subiendo de 10 en 10 hasta que tome bien todas las curvas del circuito.
float Ki = 0.75f;  // Suavizado fino, dejar de ultimo, no es importante
float Kd = 12.0f;  //Controla las osilaciones, hay que subirlo si se sube Kp, bajarlo si se baja Kp, sibirlo de 3 en 3, no es importante

///////////////////////////////////////////////////////////////////////////////

///////No cambiar mas nada//////////////////////////////////

//Valor maximo para la parte integral, no modificarlo.
float integralMax = 200.0f;

// === Constante para los motores ===

const int MOTOR_DEADBAND_PWM = 40; // valor minimo para la senal que mueve a los motores.

// Trim, funcion para compensar diferencias en los motores
// Mayor que 1 aumenta la velocidad del motor, menor que 1 la disminuye.
const float TRIM_RIGHT_MOTOR = 1.0f;
const float TRIM_LEFT_MOTOR = 0.95f;

// === Variables PID === No modificar
float errorPID = 0;
float previousErrorPID = 0;
float proportionalTerm = 0;
float integralAccumulator = 0;
float integralTerm = 0;
float derivativeTerm = 0;
float pidAdjustment = 0;

unsigned long currentTime = 0;
unsigned long previousLoopTime = 0;
float deltaTime_s = 0.01f; // Default delta time, will be calculated

// State for when the line is lost
enum LinePositionMemory {
    POS_MEM_NONE = 0,
    POS_MEM_LEFT_OF_LINE = 1, // Robot was to the left of the line (line was to its right)
    POS_MEM_RIGHT_OF_LINE = 2 // Robot was to the right of the line (line was to its left)
};
LinePositionMemory lastKnownLinePosition = POS_MEM_NONE;

// === Debugging ===
// 0: No debug output
// 1: Basic PID info (Error, Adjustment, PWMs)
// 2: Detailed PID info (P, I, D terms, Sensor states)
const int DEBUG_LEVEL = 1;

void setup() {
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENB_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);

    pinMode(SENSOR_RIGHT_PIN, INPUT);
    pinMode(SENSOR_CENTER_PIN, INPUT);
    pinMode(SENSOR_LEFT_PIN, INPUT);

    Serial.begin(115200);
    Serial.println("Robot Inicializado.");
    Serial.print("Sensor Avtivo: ");
    Serial.println(SENSOR_READING_FOR_LINE == HIGH ? "HIGH" : "LOW");

    previousLoopTime = millis();
}

// --- Fuciones de Motores ---
void setMotorSpeed(char motor, int speed, bool forward) {

    byte pinPWM, pinCtrl1, pinCtrl2;
    float trimFactor;

    if (motor == 'R') {
        pinPWM = ENA_PIN;
        pinCtrl1 = IN1_PIN;
        pinCtrl2 = IN2_PIN;
        trimFactor = TRIM_RIGHT_MOTOR;
    } else {
        pinPWM = ENB_PIN;
        pinCtrl1 = IN3_PIN;
        pinCtrl2 = IN4_PIN;
        trimFactor = TRIM_LEFT_MOTOR;
    }

    // Trim
    int trimmedSpeed = constrain((int)(speed * trimFactor), 0, 255);

    // Aplicar Deadband
    if (trimmedSpeed > 0 && trimmedSpeed < MOTOR_DEADBAND_PWM) {
        trimmedSpeed = MOTOR_DEADBAND_PWM;
    } else if (trimmedSpeed == 0) {
         trimmedSpeed = 0; // Ensure it stays 0 if commanded 0
    }


    if (forward) {
        digitalWrite(pinCtrl1, HIGH);
        digitalWrite(pinCtrl2, LOW);
    } else {
        digitalWrite(pinCtrl1, LOW);
        digitalWrite(pinCtrl2, HIGH);
    }
    analogWrite(pinPWM, trimmedSpeed);
}

void stopMotors() {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
    analogWrite(ENA_PIN, 0);
    analogWrite(ENB_PIN, 0);
}

void loop() {
    currentTime = millis();
    deltaTime_s = (currentTime - previousLoopTime) / 1000.0f;
    previousLoopTime = currentTime;

    if (deltaTime_s <= 0.0001f) { // previne division entre cero
        deltaTime_s = 0.01f;
    }

     // This logic inverts readings if SENSOR_READING_FOR_LINE is LOW
    int sR = (digitalRead(SENSOR_RIGHT_PIN) == SENSOR_READING_FOR_LINE) ? 1 : 0;
    int sC = (digitalRead(SENSOR_CENTER_PIN) == SENSOR_READING_FOR_LINE) ? 1 : 0;
    int sL = (digitalRead(SENSOR_LEFT_PIN) == SENSOR_READING_FOR_LINE) ? 1 : 0;

    // --- 2. Calculo Error PID ---
    // Error:
    // Positivo si la linea esta a la derecha.
    // Negative si la linea esta a la izquierda.

    if (sL == 0 && sC == 0 && sR == 0) { // 000 - Linea perdida, verifica cual fue la ultima posicion de la linea y hace que el error sea 4 o -4.
        if (lastKnownLinePosition == POS_MEM_LEFT_OF_LINE) {
            errorPID = -4.0f;
        } else if (lastKnownLinePosition == POS_MEM_RIGHT_OF_LINE) {
            errorPID = 4.0f;
        } else {
            // por defecto sera 4 si no hay valor anterior
            errorPID = 4.0f;
        }
    } else if (sL == 0 && sC == 1 && sR == 0) { // 010 - Centrado
        errorPID = 0.0f;
        lastKnownLinePosition = POS_MEM_NONE;
    } else if (sL == 1 && sC == 1 && sR == 0) { // 110 - linea un poco a la izquierda
        errorPID = -0.5f;
        lastKnownLinePosition = POS_MEM_LEFT_OF_LINE;
    } else if (sL == 1 && sC == 0 && sR == 0) { // 100 - linea a la izquierda
        errorPID = -2.0f;
        lastKnownLinePosition = POS_MEM_LEFT_OF_LINE;
    } else if (sL == 0 && sC == 1 && sR == 1) { // 011 - linea un poco a la derecha
        errorPID = 0.5f;
        lastKnownLinePosition = POS_MEM_RIGHT_OF_LINE;
    } else if (sL == 0 && sC == 0 && sR == 1) { // 001 - linea a la derecha
        errorPID = 2.0f;
        lastKnownLinePosition = POS_MEM_RIGHT_OF_LINE;
    } else if (sL == 1 && sC == 1 && sR == 1) { // 111 - todos en linea, tratar como centrado
        errorPID = 0.0f;
        lastKnownLinePosition = POS_MEM_NONE;
    } else if (sL == 1 && sC == 0 && sR == 1) { // 101 - caso extrano, mantener
        errorPID = previousErrorPID; // mantener el error previo
    }

    // --- 3. Calculo de terminos PID ---
    proportionalTerm = Kp * errorPID;

    integralAccumulator += errorPID * deltaTime_s;
    if (integralMax > 0.001f) {
        integralAccumulator = constrain(integralAccumulator, -integralMax, integralMax);
    } else {
        integralAccumulator = 0;
    }
    integralTerm = Ki * integralAccumulator;

    if (deltaTime_s > 0.0001f) {
        derivativeTerm = Kd * (errorPID - previousErrorPID) / deltaTime_s;
    } else {
        derivativeTerm = 0;
    }
    previousErrorPID = errorPID;

    pidAdjustment = proportionalTerm + integralTerm + derivativeTerm;

    // --- 4. Calculo de velocidad de Motores ---
    // pidAdjustment Positivo lo deberia mover a la derecha
    // pidAdjustment Negativo lo deberia mover a la izquierda

    float rawSpeedLeft = baseSpeed + pidAdjustment;
    float rawSpeedRight = baseSpeed - pidAdjustment;

    bool dirLeftForward = rawSpeedLeft >= 0;
    bool dirRightForward = rawSpeedRight >= 0;

    int pwmLeft = abs(round(rawSpeedLeft));
    int pwmRight = abs(round(rawSpeedRight));

    // Restringir valores de senal de motores

    pwmLeft = constrain(pwmLeft, 0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    // --- 5. Aplicar la velocidad al driver de motor ---
    setMotorSpeed('L', pwmLeft, dirLeftForward);
    setMotorSpeed('R', pwmRight, dirRightForward);

    // --- 6. Debug ---
    if (DEBUG_LEVEL > 0) {
        if (DEBUG_LEVEL >= 2) {
            Serial.print("S:["); Serial.print(sL); Serial.print(sC); Serial.print(sR); Serial.print("] ");
            Serial.print("Err:"); Serial.print(errorPID, 2);
            Serial.print(" P:"); Serial.print(proportionalTerm, 2);
            Serial.print(" I_acc:"); Serial.print(integralAccumulator, 2);
            Serial.print(" I_term:"); Serial.print(integralTerm, 2);
            Serial.print(" D:"); Serial.print(derivativeTerm, 2);
            Serial.print(" Adj:"); Serial.print(pidAdjustment, 2);
            Serial.print(" RawL:"); Serial.print(rawSpeedLeft);
            Serial.print(" RawR:"); Serial.print(rawSpeedRight);
            Serial.print(" PwmL:"); Serial.print(pwmLeft);
            Serial.print(" PwmR:"); Serial.println(pwmRight);
            Serial.print(" dt:"); Serial.println(deltaTime_s, 4);
        } else if (DEBUG_LEVEL == 1) {
            Serial.print("Err: "); Serial.print(errorPID, 2);
            Serial.print("\tAdj: "); Serial.print(pidAdjustment, 2);
            Serial.print("\tL_PWM: "); Serial.print(pwmLeft);
            Serial.print("\tR_PWM: "); Serial.println(pwmRight);
        }
    }
    
    delay(5); //un pequeno delay para que todo funcione bien, puede estar entre 5 y 10
}