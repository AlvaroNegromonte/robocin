#include "mbed.h"
#include "MPU6050.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <deque>
#include <cmath>

// Definição de pinos para I2C
#define MPU_SDA PB_9 // Altere conforme o seu hardware
#define MPU_SCL PB_8 // Altere conforme o seu hardware

// Caminho do arquivo CSV
#define DATASET_PATH "filtered_robot_log.csv"

// Objeto MPU6050
MPU6050 mpu(MPU_SDA, MPU_SCL);

// Função para ler dados do arquivo CSV
std::vector<std::vector<double>> readCSV(const std::string& filePath) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        printf("Erro ao abrir o arquivo CSV: %s\n", filePath.c_str());
        return data;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> row;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(std::stod(cell));
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

// Filtro de média móvel
std::deque<double> movingAverageFilter(const std::deque<double>& data, size_t windowSize) {
    std::deque<double> result;
    double sum = 0;

    // Calcula a soma inicial para a janela de dados
    for (size_t i = 0; i < windowSize && i < data.size(); ++i) {
        sum += data[i];
    }
    result.push_back(sum / windowSize);

    for (size_t i = windowSize; i < data.size(); ++i) {
        sum += data[i] - data[i - windowSize];
        result.push_back(sum / windowSize);
    }

    return result;
}

int main() {
    printf("\n=== MPU6050 Test (Simulação com CSV e Filtro de Média Móvel) ===\n");

    // Inicializa o MPU6050
    if (!mpu.initialize()) {
        printf("Erro ao inicializar o MPU6050.\n");
        return -1;
    }
    printf("MPU6050 inicializado com sucesso!\n");

    // Carrega o arquivo CSV
    std::vector<std::vector<double>> dataset = readCSV(DATASET_PATH);
    if (dataset.empty()) {
        printf("Nenhum dado encontrado no arquivo CSV.\n");
        return -1;
    }
    printf("Arquivo CSV carregado com sucesso! Total de entradas: %lu\n", dataset.size());

    // Definição do tamanho da janela para o filtro de média móvel
    const size_t windowSize = 5; // Tamanho da janela para média móvel

    // Vetor para armazenar a série temporal do giroscópio Z
    std::deque<double> gyroZData;

    // Variáveis para posição angular
    double positionZ = 0.0;
    double deltaTime = 0.01; // Intervalo de tempo entre medições (exemplo: 10ms)

    // Itera sobre os dados do CSV
    for (const auto& row : dataset) {
        if (row.size() < 1) {
            printf("Linha inválida no CSV.\n");
            continue;
        }

        // Dados brutos do giroscópio no eixo Z
        double gyroZ = row[0];

        // Adiciona os dados ao filtro de média móvel
        gyroZData.push_back(gyroZ);

        // Aplica o filtro de média móvel se houver dados suficientes
        if (gyroZData.size() > windowSize) {
            gyroZData.pop_front();
        }

        // Obtém os dados filtrados
        std::deque<double> filteredZ = movingAverageFilter(gyroZData, windowSize);

        // Exibe os dados do giroscópio no eixo Z
        printf("Giroscópio Z (deg/s): %.3lf\n", gyroZ);

        // Converte para rad/s (velocidade angular)
        double gyroZRad = gyroZ * (M_PI / 180.0);

        // Exibe a velocidade angular em rad/s
        printf("Giroscópio Z (rad/s): %.3lf\n", gyroZRad);

        // Calcula a posição angular (integração da velocidade angular)
        positionZ += gyroZRad * deltaTime;

        // Exibe a posição angular acumulada
        printf("Posição Angular Z (rad): %.3lf\n", positionZ);

        // Aguarda antes de ler a próxima linha (simula a leitura em tempo real)
        ThisThread::sleep_for(100ms);
    }

    return 0;
}
