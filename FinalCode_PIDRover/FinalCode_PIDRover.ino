// ── Motor A (RIGHT WHEEL) ────────────────────────────────
const int ENA = 9;  const int IN1 = 8;  const int IN2 = 7;
// ── Motor B (LEFT WHEEL) ─────────────────────────────────
const int ENB = 3;  const int IN3 = 5;  const int IN4 = 4;
// ── Ultrasonic Sensors ───────────────────────────────────
const int TRIG_FRONT = 11; const int ECHO_FRONT = 12;
const int TRIG_SF    = 6;  const int ECHO_SF    = 10;  // Side-Front
const int TRIG_SR    = 13; const int ECHO_SR    = A0;  // Side-Rear

// ── Core Parameters ──────────────────────────────────────
const float TARGET_DIST     = 32.0;  // cm from wall
const float FRONT_STOP_DIST = 35.0;  // cm — stop threshold
const int   BASE_SPEED      = 90;    // 0–255 PWM base
const int   MAX_ADJUST      = 80;    // Max PID correction added/removed

// ── Motor Trim ───────────────────────────────────────────
// Boosting the weak left motor instead of slowing the strong right
const int LEFT_TRIM = 28;

// ── PID Gains ────────────────────────────────────────────
const float Kp = 1.5;
const float Ki = 0.05;
const float Kd = 2.5;

// ── Sensor Median Filter ─────────────────────────────────
const int FILTER_SIZE = 5;
long frontBuf[FILTER_SIZE], sfBuf[FILTER_SIZE], srBuf[FILTER_SIZE];
int  frontIdx = 0, sfIdx = 0, srIdx = 0;
bool frontFull = false, sfFull = false, srFull = false;

// ── PID State ────────────────────────────────────────────
float integral   = 0.0;
float lastError  = 0.0;
unsigned long lastTime = 0;

// ── Startup kick (one-time only) ─────────────────────────
bool started = false;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_SF,    OUTPUT); pinMode(ECHO_SF,    INPUT);
  pinMode(TRIG_SR,    OUTPUT); pinMode(ECHO_SR,    INPUT);

  for (int i = 0; i < FILTER_SIZE; i++) {
    frontBuf[i] = 100;
    sfBuf[i]    = TARGET_DIST;
    srBuf[i]    = TARGET_DIST;
  }

  Serial.begin(9600);
  Serial.println("PID Wall Follower Ready");
  Serial.print("LEFT_TRIM = "); Serial.println(LEFT_TRIM);
}

// ── Raw distance read. Returns -1 on timeout/invalid. ────
long readRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration <= 0) return -1;
  long dist = duration * 0.034 / 2;
  if (dist < 1 || dist > 400) return -1;
  return dist;
}

// ── Push a valid reading into a circular buffer. ─────────
void pushReading(long val, long* buf, int &idx, bool &full) {
  if (val == -1) return;
  buf[idx] = val;
  idx = (idx + 1) % FILTER_SIZE;
  if (idx == 0) full = true;
}

// ── Compute median of a buffer. ──────────────────────────
long median(long* buf, bool full) {
  int n = full ? FILTER_SIZE : 1;
  long sorted[FILTER_SIZE];
  memcpy(sorted, buf, n * sizeof(long));
  for (int i = 1; i < n; i++) {
    long key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j+1] = sorted[j]; j--; }
    sorted[j+1] = key;
  }
  return sorted[n / 2];
}

// ── Drive both motors forward. LEFT_TRIM applied here. ───
void setMotors(int leftSpeed, int rightSpeed) {
  // Boost the weaker left motor
  leftSpeed = leftSpeed + LEFT_TRIM;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Left forward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Right forward
  analogWrite(ENB, leftSpeed);
  analogWrite(ENA, rightSpeed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  integral  = 0.0;
  lastError = 0.0;
  started   = false;
}

void loop() {
  // 1. Read sensors into buffers
  pushReading(readRaw(TRIG_FRONT, ECHO_FRONT), frontBuf, frontIdx, frontFull);
  delay(5);
  pushReading(readRaw(TRIG_SF, ECHO_SF),       sfBuf,    sfIdx,    sfFull);
  delay(5);
  pushReading(readRaw(TRIG_SR, ECHO_SR),       srBuf,    srIdx,    srFull);

  // 2. Get filtered distances
  long frontDist = median(frontBuf, frontFull);
  long sfDist    = median(sfBuf,    sfFull);
  long srDist    = median(srBuf,    srFull);

  Serial.print("F:"); Serial.print(frontDist);
  Serial.print(" SF:"); Serial.print(sfDist);
  Serial.print(" SR:"); Serial.println(srDist);

  // 3. Front obstacle check
  if (frontDist < FRONT_STOP_DIST) {
    stopMotors();
    Serial.println(">>> STOPPED: Front obstacle <<<");
    delay(200);
    return;
  }

  // 4. One-time startup kick to overcome static friction
  if (!started) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENB, constrain(255 + LEFT_TRIM, 0, 255)); // Left kick with trim
    analogWrite(ENA, 255);                                 // Right kick
    delay(300);
    started = true;
    lastTime = millis();
  }

  // 5. Compute dt
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.04;
  lastTime = now;

  // ── PID CONTROL ──────────────────────────────────────────
  float sideDist   = (sfDist + srDist) / 2.0;
  float distError  = sideDist - TARGET_DIST;
  float angleError = (float)(sfDist - srDist);
  float error      = distError + (angleError * 0.6);

  // P term
  float P = Kp * error;

  // I term (with windup clamp)
  integral += error * dt;
  integral = constrain(integral, -50.0, 50.0);
  float I = Ki * integral;

  // D term
  float D = Kd * ((error - lastError) / dt);
  lastError = error;

  float correction = P + I + D;
  correction = constrain(correction, -MAX_ADJUST, MAX_ADJUST);

  // Positive error = too far from wall → steer right → left faster, right slower
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  setMotors(leftSpeed, rightSpeed);

  Serial.print("err:"); Serial.print(error);
  Serial.print(" P:"); Serial.print(P);
  Serial.print(" I:"); Serial.print(I);
  Serial.print(" D:"); Serial.print(D);
  Serial.print(" L:"); Serial.print(leftSpeed + LEFT_TRIM); // Show actual PWM
  Serial.print(" R:"); Serial.println(rightSpeed);

  delay(40);
}