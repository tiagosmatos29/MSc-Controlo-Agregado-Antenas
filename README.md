# Agregado de Antenas Multibanda para Comunicações Sem Fios Críticas e Seguras
> **Repositório de código – Dissertação de Mestrado em Engenharia de Eletrónica Industrial e Computadores (Universidade do Minho)**

Este repositório contém o código desenvolvido no âmbito da dissertação de Mestrado, dedicado ao controlo de fase de um agregado de antenas multibanda. O sistema permite configurar a fase dos elementos, fazer o varrimento do feixe (*beam sweeping*), estimar a direção de chegada (DoA) de uma fonte interferente e posicionar nulos para mitigar a interferência (*null steering*).

A validação experimental do código incidiu sobre o subsistema GPS L1 (agregado linear uniforme de 5 elementos, com espaçamento $\lambda/2$).

---

## 🧰 Tecnologias Utilizadas

* **C++ (Arduino)**: *firmware* de controlo dos controladores de fase por SPI. Implementa *beamforming*, *null steering* e o método de Steyskal (*independent null steering*), e inclui a calibração dos canais.
* **Python 3**: aplicação de rastreio. Faz o varrimento angular, estima a DoA, aciona o *null steering* e regista os resultados. É a ponte entre o GNU Radio e o Arduino.
* **GNU Radio + ZeroMQ**: processamento do sinal do SDR e publicação da potência recebida.
* **Bibliotecas Python**: `pyzmq` e `pyserial`.

---

## 🛠️ Hardware

* Agregado de 5 elementos; o elemento central é a referência de fase e os restantes quatro têm um controlador de fase digital PE44820 (pSemi), programado por SPI.
* Microcontrolador Arduino.
* SDR.

---

## 📁 Estrutura do Repositório

```text
Dissertacao_Codigos/
├── Firmware/             # Firmware do microcontrolador Arduino
│   └── main_control.ino  # Beamforming, null steering e método de Steyskal; LUT e offsets de calibração
│
├── GNU_Radio/            # Processamento do sinal
│   ├── rssi_monitor.grc  # Fluxograma de medição de potência (RSSI)
│   └── rssi_monitor.py   # Script gerado a partir do fluxograma
│
├── Scripts/              # Aplicação de controlo
│   └── beam_sweeping.py  # Varrimento do feixe, DoA e comunicação Arduino / SDR
│
├── Logs/                 # Registos dos ensaios
│   ├── jammer_log_20260924_160501.csv
│   └── ...
│
└── README.md
```

---

## ⚙️ Execução e Utilização

### 1️⃣ Firmware (Arduino)
1. Abrir `Firmware/main_control.ino` no Arduino IDE (ou no VS Code com a extensão do Arduino).
2. Compilar e carregar para a placa.
3. O *firmware* comunica por porta série a 115200 bauds e aguarda comandos do seu menu. Os que a aplicação em Python utiliza são a opção 2 (*beamforming*, recebe o ângulo do feixe) e a opção 6 (*null steering* independente (método de Steyskal), recebe o ângulo do feixe principal e o do nulo).
4. **Calibração:** os *offsets* de fase por canal e a tabela de conversão (LUT) estão definidos no próprio *firmware* e dependem do *hardware* utilizado. Se o circuito for diferente, é necessário voltar a medir e atualizar estes valores.

### 2️⃣ GNU Radio (SDR)
1. Abrir `GNU_Radio/rssi_monitor.grc` no GNU Radio Companion (ou executar `rssi_monitor.py`).
2. O fluxograma mede continuamente a potência do sinal (RSSI) e publica-a por ZeroMQ (`tcp://127.0.0.1:5555`, valores `float32`), que o *script* de controlo subscreve.

### 3️⃣ Aplicação de controlo (Python)
O *script* `beam_sweeping.py` na pasta `Scripts/` é o controlador central: subscreve a potência publicada pelo GNU Radio, estima a DoA da fonte interferente e envia ao Arduino os ângulos do feixe principal e do nulo, sendo o Arduino quem calcula e aplica as fases correspondentes.

**Pré-requisitos:**
Recomenda-se vivamente a criação de um ambiente virtual (`venv`) e a instalação das dependências necessárias:
```bash
pip install pyzmq pyserial
```

**Configuração:** no início de `beam_sweeping.py`, ajustar `ARDUINO_PORT` (porta série do Arduino) e, se necessário, `ZMQ_ADDRESS`.

**Execução:** arrancar primeiro o GNU Radio e depois o *script*:
```bash
cd Scripts
python beam_sweeping.py
```

Em cada ciclo, o *script* regista no ficheiro `jammer_log_<data>_<hora>.csv` o modo de rastreio, a DoA estimada, as potências antes e depois da aplicação do nulo e a atenuação obtida. Os ficheiros da pasta `Logs/` foram gerados deste modo.

---

## 👤 Autoria

**Tiago da Silva Matos** — Universidade do Minho  
**Orientação:**
* Professor Doutor Paulo Mateus Mendes
* Doutor Hugo Daniel da Costa Dinis
