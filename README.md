# Agregado de Antenas Multibanda para Comunicações Sem Fios Críticas e Seguras
> **Repositório de Código – Dissertação de Mestrado em Engenharia de Eletrónica Industrial e Computadores (Universidade do Minho)**

Este repositório contém o código-fonte desenvolvido no âmbito da dissertação de Mestrado, focado no controlo e calibração de um agregado de antenas multibanda para UAVs. O sistema permite a configuração de fase, varrimento de feixe (*beam sweeping*) e posicionamento de nulos para mitigação de interferências (*anti-jamming*).

---

## 🧰 Tecnologias Utilizadas

* **C++ (Arduino)**: *Firmware* para controlo de *hardware* e calibração de canais RF.
* **Python 3**: Automação de testes, extração de dados e controlo de sistema (fazedora de ponte entre o Arduino e o GNU Radio).
* **GNU Radio**: Processamento de sinal de Rádio Definido por Software (SDR) e medição de potência.
* **Bibliotecas Python**: `scikit-rf` (análise de parâmetros S), `matplotlib` (visualização) e `pandas`.

---

## 📁 Estrutura do Repositório

```text
Dissertacao_Codigos/
├── /Firmware             # Firmware do microcontrolador Arduino
│   └── main_control.ino  # Código de calibração e controlo de fase (LUT)
│
├── /GNU_Radio            # Fluxogramas e ficheiros gerados do GNU Radio
│   ├── rssi_monitor.grc  # Fluxograma para medição de potência de sinal (RSSI)
│   └── rssi_monitor.py   # Script Python gerado que comunica com o script de automação principal
│
├── /Scripts              # Guiões desenvolvidos no VS Code
│   └── beam_sweeping.py  # Varrimento de feixe, posicionamento de nulos e comunicação do sistema
│
├── /Logs                 # Registos de dados e resultados de testes práticos
│   ├── jammer_log_20260924_160501.csv
│   └── ...
│
└── README.md
```

---

## ⚙️ Execução e Utilização

### 1️⃣ Firmware (Arduino)
1. Abrir os ficheiros na diretoria `/Firmware` utilizando o Arduino IDE ou o VS Code (com a extensão do Arduino).
2. Compilar e carregar o *firmware* para a placa ligada.
3. O sistema aguardará por comandos de configuração de fase através de comunicação série.

### 2️⃣ GNU Radio (SDR)
1. Abrir o `rssi_monitor.grc` na pasta `/GNU_Radio` utilizando o **GNU Radio Companion** (ou executar o `rssi_monitor.py` diretamente).
2. Uma vez executado, este bloco mede continuamente a potência do sinal (RSSI) e estabelece comunicação com o *script* de automação principal para fornecer dados de potência em tempo real.

### 3️⃣ Guiões de Automação (Python)
Os *scripts* localizados na pasta `/Scripts` atuam como o controlador central. Comunicam com o Arduino (através da porta série) para definir as fases e obtêm os dados de RSSI do bloco ativo do GNU Radio para avaliar o desempenho do *beam sweeping* e do *anti-jamming*.

**Pré-requisitos:**
Recomenda-se vivamente a criação de um ambiente virtual (`venv`) e a instalação das dependências necessárias:
```bash
pip install pyserial matplotlib scikit-rf pandas
```

**Executar o teste de Beam Sweeping:**
```bash
cd Scripts
python beam_sweeping.py
```
*(Nota: Certifique-se de que a porta COM correta para o Arduino está configurada no início do ficheiro `beam_sweeping.py` e de que o GNU Radio `rssi_monitor` está ativamente a correr antes da execução).*

---

## 👤 Autor

**Tiago Matos**