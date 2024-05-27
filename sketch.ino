// Giray Aksakal

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// MARK: - PID CONSTANTS
#define KP 1.0
#define KI 0.05
#define KD 0.01

#define F_CPU 16000000UL
#define LCD_I2C_ADDR 0x27
#define MPU6050_ADDR 0x68

// MPU6050 register addresses
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43

// I2C functions
void i2c_init(void);
void i2c_start(uint8_t address);
void i2c_write(uint8_t data);
void i2c_stop(void);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

// LCD functions
void lcd_init(uint8_t lcd_addr);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char* str);
void lcd_clear(void);
static void lcd_send_command(uint8_t command);
static void lcd_send_data(uint8_t data);
static void lcd_write(uint8_t data, uint8_t mode);

// MPU6050 functions
void mpu6050_init(void);
void mpu6050_read_accel(int16_t* ax, int16_t* ay, int16_t* az);
void mpu6050_read_gyro(int16_t* gx, int16_t* gy, int16_t* gz);

// Servo functions
void servo_init(void);
void set_servo_position(uint8_t servo, uint8_t angle);

// Definitions for LCD control bits
#define LCD_BACKLIGHT   0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

// ---------  PID SETTINGS 2   -----------------
  double setpointX = 0;
  double inputX;
  double outputX;
  double integralX = 0;
  double lastErrorX = 0;

  double setpointY = 0;
  double inputY;
  double outputY;
  double integralY = 0;
  double lastErrorY = 0;

  double setpointZ = 0;
  double inputZ;
  double outputZ;
  double integralZ = 0;
  double lastErrorZ = 0;
  // -------------------------------------------

void updatePID(double *input, double *output, double *integral, double *lastError, double setpoint) {
    double error = setpoint - *input;
    *integral += (error);
    double derivative = (error - *lastError);

    *output = KP * error + KI * *integral + KD * derivative;
    *lastError = error;
}

int main(void) {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // Initialize I2C and MPU6050
  i2c_init();
  mpu6050_init();

  // Initialize LCD
  lcd_init(LCD_I2C_ADDR);
  lcd_set_cursor(0, 0);

  // Initialize servos
  servo_init();

  while (1) {
    // Read accelerometer and gyroscope data
    mpu6050_read_accel(&ax, &ay, &az);
    mpu6050_read_gyro(&gx, &gy, &gz);

    // MARK: - Apply PID algorithm
    inputX = gx / 131.0;
    inputY = gy / 131.0;
    inputZ = gz / 131.0;

    updatePID(&inputX, &outputX, &integralX, &lastErrorX, setpointX);
    updatePID(&inputY, &outputY, &integralY, &lastErrorY, setpointY);
    updatePID(&inputZ, &outputZ, &integralZ, &lastErrorZ, setpointZ);

    int servoPositionX = map(outputX, -280, 280, 0, 360);
    int servoPositionY = map(outputY, -280, 280, 0, 360);
    int servoPositionZ = map(outputZ, -280, 280, 0, 360);

    set_servo_position(0, servoPositionX);
    set_servo_position(1, servoPositionY);
    set_servo_position(2, servoPositionZ);
    set_servo_position(3, 360 - servoPositionZ);

    // MARK: -Display mpu data on LCD
    lcd_clear();
    lcd_set_cursor(0, 0);
    char buffer[16];
    char axbuffer[16];
    char aybuffer[16];
    char azbuffer[16];

    if(ax/1638 > 9) {
      sprintf(axbuffer, "%d.%d", ax/16380, (ax/1638)%10);
    } else if(ax/1638 < -9){
      sprintf(axbuffer, "%d.%d", ax/16380 ,(ax/1638)%10 * -1);
    } else if(ax/1638 < 0 && ax/1638 >= -9) {
      sprintf(axbuffer, "-0.%d", (ax/1638)%10 * -1);
    } else {
      sprintf(axbuffer, "0.%d", (ax/1638)%10);
    }

    if(ay/1638 > 9) {
      sprintf(aybuffer, "%d.%d", ay/16380, (ay/1638)%10);
    } else if(ay/1638 < -9){
      sprintf(aybuffer, "%d.%d", ay/16380 ,(ay/1638)%10 * -1);
    } else if(ay/1638 < 0 && ay/1638 >= -9) {
      sprintf(aybuffer, "-0.%d", (ay/1638)%10 * -1);
    } else {
      sprintf(aybuffer, "0.%d", (ay/1638)%10);
    }

    if(az/1638 > 9) {
      sprintf(azbuffer, "%d.%d", az/16380, (az/1638)%10);
    } else if(az/1638 < -9){
      sprintf(azbuffer, "%d.%d", az/16380 ,(az/1638)%10 * -1);
    } else if(az/1638 < 0 && az/1638 >= -9) {
      sprintf(azbuffer, "-0.%d", (az/1638)%10 * -1);
    } else {
      sprintf(azbuffer, "0.%d", (az/1638)%10);
    }

    lcd_print(buffer);
    lcd_set_cursor(0, 0);
    sprintf(buffer, "AX: %s", axbuffer);
    lcd_print(buffer);

    lcd_set_cursor(1, 0);
    sprintf(buffer, "AY: %s", aybuffer);
    lcd_print(buffer);

    lcd_set_cursor(2, 0);
    sprintf(buffer, "AZ: %s", azbuffer);
    lcd_print(buffer);

    lcd_set_cursor(0, 11);
    sprintf(buffer, "GX: %d", gx / 131);
    lcd_print(buffer);

    lcd_set_cursor(1, 11);
    sprintf(buffer, "GY: %d", gy / 131);
    lcd_print(buffer);

    lcd_set_cursor(2, 11);
    sprintf(buffer, "GZ: %d", gz / 131);
    lcd_print(buffer);

    _delay_ms(500);
  }

  return 0;
}

// I2C master functions
void i2c_init(void) {
  TWSR = 0x00;
  TWBR = 0x47;  // Set bit rate
  TWCR = (1 << TWEN);  // Enable TWI
}

void i2c_start(uint8_t address) {
  TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);  // Send START condition
  while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set

  TWDR = address;  // Load slave address
  TWCR = (1 << TWEN) | (1 << TWINT);  // Start transmission
  while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set
}

void i2c_write(uint8_t data) {
  TWDR = data;  // Load data to TWDR register
  TWCR = (1 << TWEN) | (1 << TWINT);  // Start transmission
  while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set
}

void i2c_stop(void) {
  TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);  // Send STOP condition
  while (TWCR & (1 << TWSTO));  // Wait until STOP condition is executed
}

uint8_t i2c_read_ack(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);  // Enable TWI and ACK
  while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set
  return TWDR;
}

uint8_t i2c_read_nack(void) {
  TWCR = (1 << TWINT) | (1 << TWEN);  // Enable TWI without ACK
  while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag set
  return TWDR;
}

// LCD I2C functions
void lcd_init(uint8_t lcd_addr) {
  _delay_ms(50);
  lcd_send_command(0x33);  
  lcd_send_command(0x32);  
  lcd_send_command(0x06);  
  lcd_send_command(0x0C);  
  lcd_send_command(0x28);  
  lcd_send_command(0x01); 
  _delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  lcd_send_command(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char* str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}

void lcd_clear(void) {
  lcd_send_command(0x01);  
  _delay_ms(2);  // Wait for the command to complete
}

static void lcd_send_command(uint8_t command) {
  lcd_write(command, 0);
}

static void lcd_send_data(uint8_t data) {
  lcd_write(data, Rs);
}

static void lcd_write(uint8_t data, uint8_t mode) {
  uint8_t high_nibble = data & 0xF0;
  uint8_t low_nibble = (data << 4) & 0xF0;

  i2c_start(LCD_I2C_ADDR << 1);
  i2c_write(high_nibble | mode | LCD_BACKLIGHT | En);
  i2c_write(high_nibble | mode | LCD_BACKLIGHT);
  i2c_write(low_nibble | mode | LCD_BACKLIGHT | En);
  i2c_write(low_nibble | mode | LCD_BACKLIGHT);
  i2c_stop();
}

// MPU6050 functions
void mpu6050_init(void) {
  i2c_start(MPU6050_ADDR << 1);  
  i2c_write(MPU6050_REG_PWR_MGMT_1);  
  i2c_write(0x00); 
  i2c_stop(); 
}

void mpu6050_read_accel(int16_t* ax, int16_t* ay, int16_t* az) {
  uint8_t data[6];
  i2c_start(MPU6050_ADDR << 1);
  i2c_write(MPU6050_REG_ACCEL_XOUT_H);
  i2c_start((MPU6050_ADDR << 1) | 1);  // Repeated start for reading

  for (int i = 0; i < 6; i++) {
    if (i == 5) {
      data[i] = i2c_read_nack();  // Read without ACK on last byte
    } else {
      data[i] = i2c_read_ack();  // Read with ACK
    }
  }
  i2c_stop();

  *ax = ((data[0] << 8 | data[1]));
  *ay = ((data[2] << 8 | data[3]));
  *az = ((data[4] << 8 | data[5]));
}

void mpu6050_read_gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t data[6];
  i2c_start(MPU6050_ADDR << 1);
  i2c_write(MPU6050_REG_GYRO_XOUT_H);
  i2c_start((MPU6050_ADDR << 1) | 1);  // Repeated start for reading

  for (int i = 0; i < 6; i++) {
    if (i == 5) {
      data[i] = i2c_read_nack();  // Read without ACK on last byte
    } else {
      data[i] = i2c_read_ack();  // Read with ACK
    }
  }
  i2c_stop();

  *gx = ((data[0] << 8 | data[1]));
  *gy = ((data[2] << 8 | data[3]));
  *gz = ((data[4] << 8 | data[5]));
}

// Servo functions
void servo_init(void) {
  // Set pins as output
  DDRB |= (1 << PB6) | (1 << PB5);
  DDRE |= (1 << PE3) | (1 << PE4);

  // Set Fast PWM mode using ICR as top value
  // 50Hz PWM frequency (16MHz / (8 * (1 + 39999)) = 50Hz)

  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  
  ICR1 = 39999;  

  TCCR3A = (1 << WGM31) | (1 << COM2A1) | (1 << COM3B1);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
  ICR3 = 39999;
}

void set_servo_position(uint8_t servo, int angle) {
  uint16_t pulse_width = (angle * 11) + 1000;  // Convert angle to pulse width (1ms to 2ms)

  switch (servo) {
    case 0: // ELEVATOR
      OCR1A = pulse_width;
      break;
    case 1: // RUDDER
      OCR1B = pulse_width;
      break;
    case 2: // LEFT FLAPERON
      OCR3A = pulse_width;
      break;
    case 3: // RIGHT FLAPERON
      OCR3B = pulse_width;
      break;
    default:
      break;
  }
}
