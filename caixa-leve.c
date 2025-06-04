#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>

#define TIME_STEP 64
#define NUM_CAIXAS 20
#define NUM_SENSORES 8

// Função para inicializar sensores de proximidade
void inicializarSensores(WbDeviceTag sensores[]) {
  char nome[5];
  for (int i = 0; i < NUM_SENSORES; i++) {
    sprintf(nome, "ps%d", i);
    sensores[i] = wb_robot_get_device(nome);
    wb_distance_sensor_enable(sensores[i], TIME_STEP);
  }
}

// Função para obter leituras normalizadas dos sensores
void lerSensoresProximidade(WbDeviceTag sensores[], double leituras[]) {
  for (int i = 0; i < NUM_SENSORES; i++)
    leituras[i] = wb_distance_sensor_get_value(sensores[i]) - 60;
}

// Função para verificar se o robô está imóvel
bool estaImovel(const double *posAtual, double posAntX, double posAntZ) {
  return fabs(posAtual[0] - posAntX) < 0.001 && fabs(posAtual[2] - posAntZ) < 0.001;
}

// Verifica se alguma caixa foi movida
bool verificarCaixaMovida(WbNodeRef caixas[], double posicoesAntes[][3]) {
  for (int i = 0; i < NUM_CAIXAS; i++) {
    const double *posAtual = wb_supervisor_node_get_position(caixas[i]);
    if (fabs(posicoesAntes[i][0] - posAtual[0]) > 0.001 ||
        fabs(posicoesAntes[i][1] - posAtual[1]) > 0.001 ||
        fabs(posicoesAntes[i][2] - posAtual[2]) > 0.001)
      return true;
  }
  return false;
}

// Estratégia para sair de travamento
void estrategiaDestravamento(int direcao, double *velEsq, double *velDir) {
  switch (direcao) {
    case 0: *velEsq = 1;   *velDir = -0.4; break;
    case 1: *velEsq = -0.4; *velDir = 1;   break;
    case 2: *velEsq = 1;   *velDir = 0.4;  break;
    case 3: *velEsq = 0.4; *velDir = 1;    break;
  }
}

int main() {
  wb_robot_init();

  WbDeviceTag motorEsq = wb_robot_get_device("left wheel motor");
  WbDeviceTag motorDir = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(motorEsq, INFINITY);
  wb_motor_set_position(motorDir, INFINITY);
  wb_motor_set_velocity(motorEsq, 0);
  wb_motor_set_velocity(motorDir, 0);

  // Inicialização dos sensores
  WbDeviceTag sensores[NUM_SENSORES];
  inicializarSensores(sensores);

  // Supervisão das caixas
  WbNodeRef caixas[NUM_CAIXAS];
  char nomeCaixa[20];
  for (int i = 0; i < NUM_CAIXAS; i++) {
    sprintf(nomeCaixa, "CAIXA%02d", i + 1);
    caixas[i] = wb_supervisor_node_get_from_def(nomeCaixa);
    if (caixas[i])
      printf("Localização da %s salva\n", nomeCaixa);
    else
      printf("Erro ao acessar %s\n", nomeCaixa);
  }

  double velEsq = 1, velDir = 1;
  double posXAnt = 0, posZAnt = 0;
  int tempoImovel = 0, tentativas = 0, direcao = 0;
  bool caixaMovida = false;

  const double *posInicial = wb_supervisor_node_get_position(wb_supervisor_node_get_self());
  posXAnt = posInicial[0];
  posZAnt = posInicial[2];

  while (wb_robot_step(TIME_STEP) != -1) {
    double leituras[NUM_SENSORES];
    double posAntes[NUM_CAIXAS][3];

    lerSensoresProximidade(sensores, leituras);

    for (int i = 0; i < NUM_CAIXAS; i++) {
      const double *pos = wb_supervisor_node_get_position(caixas[i]);
      memcpy(posAntes[i], pos, 3 * sizeof(double));
    }

    const double *posAtual = wb_supervisor_node_get_position(wb_supervisor_node_get_self());

    if (!caixaMovida && estaImovel(posAtual, posXAnt, posZAnt)) {
      tempoImovel += TIME_STEP;
    } else {
      tempoImovel = 0;
      tentativas = 0;
    }

    posXAnt = posAtual[0];
    posZAnt = posAtual[2];

    if (tempoImovel >= 1100 && !caixaMovida) {
      tentativas++;
      printf("Robô travado! Ajustando... (tentativa %d de 3).\n", tentativas);
      if (tentativas >= 3) {
        direcao = (direcao + 1) % 4;
        tentativas = 0;
      }
      estrategiaDestravamento(direcao, &velEsq, &velDir);
      tempoImovel = 0;
    } else if (caixaMovida) {
      velEsq = -1;
      velDir = 1;
    } else if ((leituras[7] > 1000 || leituras[0] > 1000 || leituras[1] > 1000) &&
               (leituras[2] > 1000 || leituras[3] > 1000)) {
      velEsq = -1;
      velDir = 1;
    } else if ((leituras[7] > 1000 || leituras[0] > 1000 || leituras[6] > 1000) &&
               (leituras[4] > 1000 || leituras[5] > 1000)) {
      velEsq = 1;
      velDir = -1;
    } else if (leituras[7] > 1000 || leituras[0] > 1000) {
      velEsq = 1;
      velDir = -1;
    } else {
      velEsq = velDir = 1;
    }

    // Prevenção de marcha à ré contínua
    if (velEsq < 0 && velDir < 0) {
      if (direcao % 2 == 0) {
        velEsq = 1;
        velDir = 0.15;
      } else {
        velEsq = 0.15;
        velDir = 1;
      }
    }

    wb_motor_set_velocity(motorEsq, 6.28 * velEsq);
    wb_motor_set_velocity(motorDir, 6.28 * velDir);

    wb_robot_step(TIME_STEP);

    if (!caixaMovida && verificarCaixaMovida(caixas, posAntes)) {
      printf("Robô encontrou a caixa leve!!\n");
      caixaMovida = true;
    }
  }

  wb_robot_cleanup();
  return 0;
}

