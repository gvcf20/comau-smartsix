# COMAU SmartSix — Controle Cinemático
**ELE041/EEE935 — Manipuladores Robóticos — UFMG**  
Prof. Gustavo Medeiros Freitas

---

## Descrição

Implementação do controle cinemático em malha fechada do manipulador industrial **COMAU Smart SiX** (6 GDL) utilizando MATLAB e o [Robotics Toolbox de Peter Corke](https://petercorke.com/toolboxes/robotics-toolbox/).

O efetuador é controlado para desenhar uma **Bandeira do Brasil simplificada** no plano YZ, composta por:
- Retângulo (controle de seguimento de trajetória)
- Losango (controle de seguimento de trajetória)
- Círculo inscrito no losango (seguimento circular com feedforward)

A sequência de juntas gerada é exportada para reprodução no **CoppeliaSim**.

---

## Pré-requisitos

- MATLAB R2020a ou superior
- [Robotics Toolbox for MATLAB (Peter Corke)](https://github.com/petercorke/robotics-toolbox-matlab) — instalado via `.mltbx`
- CoppeliaSim 4.3.0 — apenas para a etapa de reprodução virtual
- Arquivos de comunicação MATLAB–CoppeliaSim (disponibilizados no Moodle da disciplina)

---

## Estrutura do Repositório

```
comau-smartsix/
│
├── modelagem.m              ← Itens i–ii:    modelo DH + verificação cinemática
├── controle.m               ← Item  iv:      regulação q0 -> P0
├── retangulo.m              ← Itens vi–viii: retângulo (reg + seguimento + reg)
├── losango.m                ← Itens ix–x:   losango (reg + seguimento + reg)
├── circulo.m                ← Itens xi–xii: círculo (reg + seguimento + reg)
├── bandeira.m               ← Execução completa da bandeira (todas as fases)
├── reproJuntasCoppelia.m    ← Reprodução no CoppeliaSim via API remota
│
└── utils/
    ├── CS6bot.m             ← Parâmetros DH originais (Pena, 2013)
    ├── comau_model.m        ← Wrapper SerialLink do COMAU SmartSix
    ├── rotm2axang2.m        ← Erro de orientação eixo-ângulo (Moodle)
    ├── control_move.m       ← Núcleo do controlador cinemático
    ├── regulate.m           ← Controle de regulação
    └── track_segment.m      ← Seguimento de trajetória linear
```

> A pasta `results/` é gerada automaticamente na primeira execução e não é versionada.

---

## Como Executar

### 1. Clone o repositório

```bash
git clone https://github.com/gvcf20/comau-smartsix.git
cd comau-smartsix
```

### 2. Configure o path no MATLAB

```matlab
addpath('utils')
```

Ou clique com o botão direito na pasta `utils/` no MATLAB → *Add to Path → Selected Folders*.

---

### 3. Modelagem (Itens i–ii)

```matlab
modelagem
```

- Constrói o modelo `SerialLink` a partir dos parâmetros DH de Pena (2013)
- Plota o robô na configuração inicial `q0 = [0, 0, -90, 0, -90, 0]°`
- Imprime a matriz homogênea e compara cinemática direta manual vs. Toolbox
- Plota os eixos da base (Xb, Yb, Zb)

---

### 4. Controle de Movimentação

Cada script abaixo é **independente** — parte de `q0` internamente.

#### Regulação q0 → P0 (Item iv)
```matlab
controle
```
Leva o efetuador até o ponto de aproximação P0 = [700, 0, 650] mm mantendo a orientação desejada constante.

#### Retângulo (Itens vi–viii)
```matlab
retangulo
```
Sequência: `q0 → P0 → P1` (regulação) → `P1→P2→P3→P4→P1` (seguimento com feedforward) → `P1 → P0` (regulação).

#### Losango (Itens ix–x)
```matlab
losango
```
Mesmo padrão do retângulo, percorrendo os vértices do losango no sentido anti-horário a partir de P5.

#### Círculo (Itens xi–xii)
```matlab
circulo
```
Seguimento circular com feedforward de velocidade angular a partir de P6, inscrito no losango.

#### Bandeira completa
```matlab
bandeira
```
Executa todas as fases em sequência e gera:
- Animação 3D do robô com rastro seletivo (só desenha nos trechos de trajetória)
- Figura final no plano YZ
- `results/q_seq_bandeira.mat` — matriz `6 × n` em graus para o CoppeliaSim

---

### 5. Reprodução no CoppeliaSim

1. Abra o CoppeliaSim 4.3.0
2. Carregue o arquivo `smartsix.ttt` (disponível no Moodle)
3. Clique em **Play**
4. No MATLAB, com `q_seq` no workspace:

```matlab
reproJuntasCoppelia
```

> Se `q_seq` não estiver no workspace, o script carrega automaticamente de `results/q_seq_bandeira.mat`.

---

## Parâmetros DH — COMAU Smart SiX

Convenção DH padrão. Base rotacionada 180° em X (`Hbase`) para alinhar com o sistema de referência do fabricante (Pena, 2013).

| Junta | θ offset | d (m)   | a (m)  | α    |
|-------|----------|---------|--------|------|
| 1     | 0        | −0.450  | 0.150  | +π/2 |
| 2     | −π/2     | 0       | 0.590  | π    |
| 3     | +π/2     | 0       | 0.130  | −π/2 |
| 4     | 0        | −0.6471 | 0      | −π/2 |
| 5     | 0        | 0       | 0      | +π/2 |
| 6     | +π       | −0.095  | 0      | π    |

---

## Geometria da Bandeira

Todos os pontos de desenho estão no plano **x = 1000 mm**.  
P0 é o ponto de aproximação em **x = 700 mm**.

| Ponto | Y (mm) | Z (mm) | Descrição                  |
|-------|--------|--------|----------------------------|
| P0    | 0      | 650    | Ponto de aproximação       |
| P1    | +600   | 1000   | Canto sup. dir. retângulo  |
| P2    | +600   | 300    | Canto inf. dir. retângulo  |
| P3    | −600   | 300    | Canto inf. esq. retângulo  |
| P4    | −600   | 1000   | Canto sup. esq. retângulo  |
| P5    | −600   | 650    | Vértice esq. losango       |
| P6    | 0      | ~350   | Ponto inf. círculo         |

---

## Modelo de Controle

Controle cinemático proporcional em malha fechada via **Jacobiana Geométrica**:

```
q_{k+1} = q_k + J†(q_k) · v_d · Δt
```

onde `v_d = [Kp·ep; Ko·eo]` e `J† = Jᵀ(JJᵀ + λ²I)⁻¹`.

- **Erro de posição:** `ep = pd - p(q)`
- **Erro de orientação:** representação eixo-ângulo `eo = n̂·θ` via `rotm2axang2`
- **Seguimento:** feedforward de velocidade `v_ff = (pb - pa)/(N·dt)` para reduzir erro de rastreamento nos cantos

| Parâmetro | Valor  | Descrição                        |
|-----------|--------|----------------------------------|
| `K`       | 2.0    | Ganho proporcional               |
| `dt`      | 0.01 s | Passo de integração (Euler)      |
| `lam`     | 0.01   | Amortecimento da Jacobiana       |
| `N_reg`   | 1000   | Iterações por regulação (10 s)   |
| `N_seg`   | 500    | Iterações por segmento (5 s)     |

---
