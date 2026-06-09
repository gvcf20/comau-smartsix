# Controle do COMAU SmartSix — ELE041/EEE935
**Manipuladores Robóticos — UFMG | Prof. Gustavo Medeiros Freitas**

---

## Estrutura de Arquivos

```
comau_project/
│
├── parte1_modelagem.m          ← Item i e ii: modelo DH + plot em q0
├── parte2_controle.m           ← Itens iii-xii: controle completo
├── reproJuntasCoppelia.m       ← Envio do q_seq para o CoppeliaSim
│
├── utils/
│   ├── comau_model.m           ← Cria o SerialLink do COMAU SmartSix
│   ├── control_move.m          ← Controlador cinemático (regulação e seguimento)
│   ├── regulate.m              ← Wrapper: controle de regulação
│   ├── track_segment.m         ← Wrapper: seguimento de trajetória linear
│   └── rotm2axang2.m           ← Erro de orientação (eixo-ângulo)
│
└── results/                    ← Gerado automaticamente
    ├── q_seq.mat               ← Sequência de juntas para CoppeliaSim
    ├── caminho_YZ.png          ← Figura da bandeira no plano YZ
    └── evolucao_juntas.png     ← Evolução das 6 juntas
```

---

## Como Executar

### Parte 1 — Modelagem
```matlab
cd caminho/para/comau_project
parte1_modelagem
```
Verifica o modelo e plota o robô na configuração `q = [0, 0, -90, 0, -90, 0]°`.

### Parte 2 — Controle
```matlab
parte2_controle
```
Executa toda a sequência de movimentos e gera:
- `results/q_seq.mat` — matriz 6×n para o CoppeliaSim
- `results/caminho_YZ.png` — bandeira no plano YZ
- `results/evolucao_juntas.png` — gráfico das juntas

### CoppeliaSim
1. Abra o CoppeliaSim e carregue `smartsix.ttt`
2. Dê **Play**
3. No MATLAB (com `q_seq` no workspace):
```matlab
reproJuntasCoppelia
```

---

## Parâmetros DH — COMAU Smart SiX

| Junta | θ offset | d (m)   | a (m)  | α       |
|-------|----------|---------|--------|---------|
| 1     | 0        | -0.450  | 0.150  | +π/2    |
| 2     | -π/2     | 0       | 0.590  | π       |
| 3     | +π/2     | 0       | 0.130  | -π/2    |
| 4     | 0        | -0.6471 | 0      | -π/2    |
| 5     | 0        | 0       | 0      | +π/2    |
| 6     | +π       | -0.095  | 0      | π       |

Convenção: DH padrão. Base rotacionada 180° em X para alinhar com o fabricante.

---

## Geometria da Bandeira

| Ponto | Y (mm) | Z (mm) | Descrição              |
|-------|--------|--------|------------------------|
| P0    | 0      | 650    | Ponto de aproximação   |
| P1    | +600   | 1000   | Canto sup. direito     |
| P2    | +600   | 300    | Canto inf. direito     |
| P3    | -600   | 300    | Canto inf. esquerdo    |
| P4    | -600   | 1000   | Canto sup. esquerdo    |
| P5    | -600   | 650    | Vértice esq. losango   |
| P6    | 0      | 350    | Ponto inf. círculo     |

Todos os pontos com x = 1000 mm (exceto P0 com x = 700 mm).
