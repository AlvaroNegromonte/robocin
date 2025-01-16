#include "MPU6050.h"
#include <string>
#define GYRO_FIFO_ENABLE 0x70 // 0b01110000 para ativar somente o giroscopio


MPU6050::MPU6050(PinName sda, PinName scl) : i2c_(sda, scl) {
    i2c_.frequency(400000);//400khz
    
}

bool MPU6050::initialize() {

    if (this->getWhoAmI()!=0x68){
        printf("Erro: MPU not found...\n");
        return false;
    }

    this->setPowerCtl_1(0x00, 0x00, 0x00, 0x00, INT_8MHz_OSC); // Disable sleep mode
    ThisThread::sleep_for(1ms);
    char data[2];

    this->setGyroConfig(GYRO_ST_OFF, GFS_2000dps); // Gyroscope self-test trigger off.
    _gyro_scale = 16.4;
    ThisThread::sleep_for(1ms);

    return true;
}

void MPU6050::selfTest(uint8_t* TestReadings) {
  char buffer[3]; // Apenas 3 registros para os eixos X, Y e Z do giroscópio

  // Lê os registros de autoteste dos eixos do giroscópio
  buffer[0] = SingleByteRead(SELF_TEST_X_REG);
  buffer[1] = SingleByteRead(SELF_TEST_Y_REG);
  buffer[2] = SingleByteRead(SELF_TEST_Z_REG);

  // Extrai os resultados do autoteste para cada eixo do giroscópio
  TestReadings[0] = (int)buffer[0] & 0x1F; // Autoteste do giroscópio no eixo X
  TestReadings[1] = (int)buffer[1] & 0x1F; // Autoteste do giroscópio no eixo Y
  TestReadings[2] = (int)buffer[2] & 0x1F; // Autoteste do giroscópio no eixo Z
}

void MPU6050::setSampleRate(uint8_t Divider) {
    SingleByteWrite(SMPLRT_DIV_REG, Divider);
}

uint8_t MPU6050::getSampleRate(void) {
    return SingleByteRead(SMPLRT_DIV_REG);
}

void MPU6050::setFSYNCConfig(uint8_t Conf) {
    uint8_t conf_L;
    uint8_t conf_H;
    uint8_t conf_HL;

    conf_L = SingleByteRead(CONFIG_REG) & 0x07; // Lê os 3 bits menos significativos
    conf_H = Conf << 3;                         // Move a configuração FSYNC para os bits 3-5
    conf_HL = conf_H | conf_L;                  // Combina as partes alta e baixa

    SingleByteWrite(CONFIG_REG, conf_HL);       // Escreve no registro CONFIG_REG
}

uint8_t MPU6050::getFSYNCConfig(void) {
    return (int) SingleByteRead(CONFIG_REG) >> 3 & 0x07;
}

void MPU6050::setDLPFConfig(uint8_t Conf) {
    uint8_t conf_L;
    uint8_t conf_H;
    uint8_t conf_HL;

    conf_L = Conf;                                   // Configuração do DLPF (bits 0-2)
    conf_H = SingleByteRead(CONFIG_REG) & 0x38;     // Preserva os bits 3-5 do registro CONFIG_REG
    conf_HL = conf_H | conf_L;                      // Combina os bits DLPF com os bits preservados

    SingleByteWrite(CONFIG_REG, conf_HL);           // Escreve o valor combinado no registro CONFIG_REG
}

uint8_t MPU6050::getDLPFConfig(void) {
    return (int) SingleByteRead(CONFIG_REG) & 0x07; // Isola os bits 0-2
}

void MPU6050::setGyroConfig(uint8_t GyroST, uint8_t Scale) {
    SingleByteWrite(GYRO_CONFIG_REG, GyroST | Scale);
}

uint8_t MPU6050::getGyroConfig(void) {
    return (int) SingleByteRead(GYRO_CONFIG_REG);
}

void MPU6050::enableGyroFIFO() {
    // Ativa apenas os dados do giroscópio no FIFO
    setFIFO_Enable(GYRO_FIFO_ENABLE);
}
void MPU6050::readGyroRaw(int16_t* gyroReadings) {

  char GYRO_OUT_buffer[6];//armazena 6 bytes do gyro

  
  multiByteRead(GYRO_XOUT_H_REG, GYRO_OUT_buffer, 6);//i2c

  //conversao para inteiro
  gyroReadings[0] = (int) GYRO_OUT_buffer[0] << 8 | (int) GYRO_OUT_buffer[1];
  gyroReadings[1] = (int) GYRO_OUT_buffer[2] << 8 | (int) GYRO_OUT_buffer[3];
  gyroReadings[2] = (int) GYRO_OUT_buffer[4] << 8 | (int) GYRO_OUT_buffer[5];
}

//converte os valores brutos em unidade padrão
void MPU6050::readGyro(double* gyroReadings) {
  int16_t gyro[3] = {0, 0, 0};
  this->readGyroRaw(gyro);
  gyroReadings[0] = (int) gyro[0] / _gyro_scale;
  gyroReadings[1] = (int) gyro[1] / _gyro_scale;
  gyroReadings[2] = (int) gyro[2] / _gyro_scale;
}

void MPU6050::sigPathReset(uint8_t ResVal) {

  SingleByteWrite(SIGNAL_PATH_RESET_REG, ResVal);
}
void MPU6050::setUserCtl(uint8_t Settings) {

  SingleByteWrite(USER_CTRL_REG, Settings);
}
uint8_t MPU6050::getUserCtl(void) {

  return (int) SingleByteRead(USER_CTRL_REG);
}
void MPU6050::setPowerCtl_1(uint8_t DevRes,//device reset
                            uint8_t Sleep,//sleep mode (low usage)
                            uint8_t Cycle,//cycle mode (opera periodicamente)
                            uint8_t Temp,//controla temperat sensor
                            uint8_t Clock) {//select fonte de clock

  uint8_t powerSetting;

  powerSetting = DevRes | Sleep;
  powerSetting = powerSetting | Cycle;
  powerSetting = powerSetting | Temp;
  powerSetting = powerSetting | Clock;

  SingleByteWrite(PWR_MGMT_1_REG, powerSetting);//i2c
}

uint8_t MPU6050::getPowerCtl_1(void) {

  return (int) SingleByteRead(PWR_MGMT_1_REG);
}

void MPU6050::setPowerCtl_2(uint8_t Conf) {

  SingleByteWrite(PWR_MGMT_2_REG, Conf);
}

uint8_t MPU6050::getPowerCtl_2(void) {

  return (int) SingleByteRead(PWR_MGMT_2_REG);
}
uint16_t MPU6050::getFIFOCount(void) {

  uint16_t FIFOCount_HL, FIFOCount_L, FIFOCount_H;

  FIFOCount_L = SingleByteRead(FIFO_COUNTL_REG);
  FIFOCount_H = SingleByteRead(FIFO_COUNTH_REG);

  FIFOCount_HL = FIFOCount_H << 8 | FIFOCount_L;

  return FIFOCount_HL;
}

void MPU6050::FIFODataWrite(uint8_t Data) {

  SingleByteWrite(FIFO_R_W_REG, Data);
}

uint8_t MPU6050::FIFODataRead(void) {

  return (int) SingleByteRead(FIFO_R_W_REG);
}

uint8_t MPU6050::getWhoAmI(void) {

  // WhoAmI Register address.
  return SingleByteRead(WHO_AM_I_REG);
}

char MPU6050::SingleByteRead(char address) {
    // Simulação: Retorna valores fixos com base no endereço do registrador
    switch (address) {
        case WHO_AM_I_REG: return 0x68; // ID esperado do sensor
        case GYRO_XOUT_H_REG: return 0x10; // Valor alto do giroscópio no eixo X
        case GYRO_XOUT_L_REG: return 0x20; // Valor baixo do giroscópio no eixo X
        case GYRO_YOUT_H_REG: return 0x30; // Valor alto do giroscópio no eixo Y
        case GYRO_YOUT_L_REG: return 0x40; // Valor baixo do giroscópio no eixo Y
        case GYRO_ZOUT_H_REG: return 0x50; // Valor alto do giroscópio no eixo Z
        case GYRO_ZOUT_L_REG: return 0x60; // Valor baixo do giroscópio no eixo Z
        default: return 0x00; // Valor padrão
    }
}

void MPU6050::multiByteRead(char address, char* output, int size) {
    // Simulação: Preenche o buffer com valores simulados
    for (int i = 0; i < size; i++) {
        output[i] = address + i; // Valores incrementais para simulação
    }
}

uint8_t MPU6050::SingleByteWrite(char address, char data) {
    // Simulação: Sempre retorna sucesso
    return 0;
}

uint8_t MPU6050::multiByteWrite(char address, char* ptr_data, int size) {
    // Simulação: Sempre retorna sucesso
    return 0;
}

