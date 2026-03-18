#include "esp32-hal-cpu.h"

#define Pin_GLED 25
#define Pin_RLED 26
#define Pin_YLED 32
#define Pin_steer 17
#define Pin_accel 16
#define Pin_empty 4

// v3.x에서는 채널 번호를 명시적으로 관리하기보다 핀에 귀속시킵니다.
int pwm_freq = 50;         // 50Hz
int pwm_resolution = 16;   // 16bit

void servoWrite(int pin, int deg) {
  // 0deg(1ms) - 90deg(1.5ms) - 180deg(2ms)
  // 16bit resolution이므로 1ms는 약 3277, 2ms는 약 6553
  int duty = (int)(deg * 18.2) + 3277;
  ledcWrite(pin, duty); // v3.x에서는 채널 대신 핀 번호를 바로 넣습니다.
}

void escWrite(int pin, int val) {
  ledcWrite(pin, val);
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  Serial.setRxBufferSize(1024);

  pinMode(Pin_GLED, OUTPUT);
  pinMode(Pin_RLED, OUTPUT);
  pinMode(Pin_YLED, OUTPUT);

  // v3.x 방식: ledcAttach(pin, freq, resolution)
  // 기존의 ledcSetup과 ledcAttachPin이 이 하나로 합쳐졌습니다.
  ledcAttach(Pin_steer, pwm_freq, pwm_resolution);
  ledcAttach(Pin_accel, pwm_freq, pwm_resolution);
  ledcAttach(Pin_empty, pwm_freq, pwm_resolution);

  // 초기 상태 설정
  servoWrite(Pin_steer, 90);
  escWrite(Pin_accel, 4915); // 중립(1.5ms)으로 초기화하는 것이 안전합니다.
  delay(2000);
}

void loop() {
  static uint8_t state = 0;
  static uint8_t payload[7];
  static uint8_t index = 0;

  // 데이터가 하나라도 들어오면 즉시 처리 (비차단 방식)
  while (Serial.available() > 0) {
    uint8_t c = Serial.read();

    switch (state) {
      case 0: // 첫 번째 헤더 대기
        if (c == 0xFF) state = 1;
        break;

      case 1: // 두 번째 헤더 대기
        if (c == 0xFE) {
          state = 2;
          index = 0;
        } else {
          state = 0; // 실패 시 초기화
        }
        break;

      case 2: // 데이터 7바이트(데이터6 + 체크섬1) 채우기
        payload[index++] = c;
        if (index >= 7) {
          // 체크섬 검증
          uint8_t checksum = 0;
          for (int i = 0; i < 6; i++) checksum += payload[i];

          if (checksum == payload[6]) {
            // 데이터 복원 및 제어 (가장 최신 데이터 반영)
            int16_t xs = (int16_t)((payload[0] << 8) | payload[1]);
            int16_t xa = (int16_t)((payload[2] << 8) | payload[3]);
            int16_t xm = (int16_t)((payload[4] << 8) | payload[5]);

            // 하드웨어 명령 하달
            servoWrite(Pin_steer, constrain(xs, 0, 180));
            escWrite(Pin_accel, constrain(xa, 3277, 6553));
            
            digitalWrite(Pin_YLED, (xm == 2));
            digitalWrite(Pin_GLED, (xm > 0));
          }
          state = 0; // 다음 패킷 준비
        }
        break;
    }
  }
}