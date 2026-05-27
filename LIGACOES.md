# Ligações — Ball Balancer ESP32-S3

## Hardware utilizado

| Componente | Modelo |
|---|---|
| Microcontrolador | ESP32-S3 DevKitC-1 |
| Tela resistiva | 4 fios, 187 × 141 mm |

---

## Tela resistiva → ESP32-S3

A tela resistiva de 4 fios usa dois planos resistivos sobrepostos (X e Y).
Cada plano tem dois terminais: positivo (+) e negativo (−).

| Sinal | Função | GPIO | Canal ADC | Fio (chicote) |
|---|---|---|---|---|
| X+ (XP) | Drive HIGH ao ler X / ADC ao ler Y | **GPIO 4** | ADC1_CH3 | Vermelho |
| X− (XM) | Drive LOW ao ler X | **GPIO 5** | ADC1_CH4 | Preto |
| Y+ (YP) | Drive HIGH ao ler Y / ADC ao ler X | **GPIO 6** | ADC1_CH5 | Branco |
| Y− (YM) | Drive LOW ao ler Y | **GPIO 7** | ADC1_CH6 | Verde |

> **Nota sobre o conector FPC:** a ordem dos pinos varia por fabricante.
> Se a posição reportada ficar espelhada ou com eixos trocados, ajuste os
> defines `TOUCH_X_INVERT`, `TOUCH_Y_INVERT` e `TOUCH_SWAP_XY` em
> `main/touch_screen.h`, ou rode `python tools/calibrate.py` novamente.

---

## Como o circuito funciona

```
        X+  ──────────────────────────────┐
                                          │  plano resistivo X
        X−  ──────────────────────────────┘

        Y+  ──────────────────────────────┐
                                          │  plano resistivo Y
        Y−  ──────────────────────────────┘
```

**Leitura do eixo X:**
- XP = HIGH, XM = LOW  →  cria divisor de tensão no plano X
- YP lido como ADC  →  tensão proporcional à posição X da esfera

**Leitura do eixo Y:**
- YP = HIGH, YM = LOW  →  cria divisor de tensão no plano Y
- XP lido como ADC  →  tensão proporcional à posição Y da esfera

**Detecção de toque:**
- YP = HIGH, XM = LOW, XP = input pull-down
- Se a esfera toca a tela, XP é puxado a HIGH pela camada Y
- Voto majoritário: 3 de 5 amostras precisam ser HIGH

---

## Pinout ESP32-S3 DevKitC-1 (pinos usados)

```
                    ┌─────────────────────────┐
                    │      ESP32-S3           │
                    │      DevKitC-1          │
                    │                         │
   X+  ◄── GPIO 4 ─┤ 4                     5 ├─ GPIO 5  ──► X−
   Y+  ◄── GPIO 6 ─┤ 6                     7 ├─ GPIO 7  ──► Y−
                    │                         │
                    │         USB-C           │
                    └─────────────────────────┘
```

Os quatro pinos são consecutivos (4-5-6-7), facilitando a conexão com o
conector FPC ou chicote de 4 fios.

---

## Calibração atual (`main/touch_screen.h`)

| Define | Valor |
|---|---|
| `TOUCH_X_RAW_MIN` | 0 |
| `TOUCH_X_RAW_MAX` | 1227 |
| `TOUCH_Y_RAW_MIN` | 172 |
| `TOUCH_Y_RAW_MAX` | 556 |
| `TOUCH_X_INVERT` | 0 |
| `TOUCH_Y_INVERT` | 0 |
| `TOUCH_SWAP_XY` | 0 |

Para recalibrar: `python tools/calibrate.py COM11`
O script atualiza o header e reflasha automaticamente.

---

## Parâmetros de leitura

| Define | Valor | Descrição |
|---|---|---|
| `TOUCH_AVG_SAMPLES` | 16 | Média de leituras ADC por eixo (reduz ruído) |
| `TOUCH_DETECT_SAMPLES` | 5 | Amostras para detecção de toque |
| `TOUCH_DETECT_THRESH` | 3 | Mínimo de amostras positivas (3/5) |
| `TOUCH_HOLD_LAST_MS` | 300 | ms mantendo última posição após levantar |

---

## Saída serial (115200 baud)

| Linha | Quando | Exemplo |
|---|---|---|
| `X:<mm>,Y:<mm>` | Toque ativo ou dentro do hold | `X:93.50,Y:70.20` |
| `CAL:<raw_x>,<raw_y>` | Junto com X/Y (usado pelo calibrate.py) | `CAL:614,321` |
| `NONE` | Sem toque e fora do hold | `NONE` |
