# Ferramentas Octave do `esp_angle_lut`

Esta pasta contém a biblioteca Octave que acompanha o componente embarcado
`esp_angle_lut`. Ela implementa o cálculo, a validação, a aplicação e a
serialização da LUT sem conhecer o sensor, a interface elétrica, o protocolo
USB ou o perfil de controle usado para coletar os dados.

Em outro projeto, adicione esta pasta ao caminho do Octave:

```octave
addpath ("components/esp_angle_lut/tools/octave");
```

## Relações entre as funções

```text
esp_angle_lut_calculate
├── validate_configuration                 [interna]
│   └── esp_angle_lut_validate
├── revolution_errors                      [interna]
│   └── wrap_pi                             [interna]
├── bin_median                              [interna]
├── fill_circular                           [interna]
├── circular_harmonic_smooth                [interna]
├── esp_angle_lut_validate
├── interp_cyclic                           [interna]
├── wrap_pi                                 [interna]
├── esp_angle_lut_apply
│   └── esp_angle_lut_validate
└── esp_angle_lut_pack
    ├── esp_angle_lut_int16_le_bytes
    └── esp_angle_lut_crc32_ieee
```

As funções cujo nome começa com `esp_angle_lut_` são públicas. As funções
adicionais existentes no final de `esp_angle_lut_calculate.m` são privadas
daquele arquivo e não podem ser chamadas diretamente por outros arquivos.

## `esp_angle_lut_calculate()`

```octave
calibration = esp_angle_lut_calculate (time_s, raw_angle_deg, segments, ...
                                       bin_count, full_scale_counts, ...
                                       max_abs_correction_counts)
```

**Tipo:** função pública da biblioteca.

**Chamada por:** aplicações externas e `test_esp_angle_lut.m`. Nenhuma outra
função pública da biblioteca a chama.

**Chama:** `validate_configuration()`, `revolution_errors()`, `bin_median()`,
`fill_circular()`, `circular_harmonic_smooth()`,
`esp_angle_lut_validate()`, `interp_cyclic()`, `wrap_pi()`,
`esp_angle_lut_apply()` e `esp_angle_lut_pack()`.

**Entradas:**

- `time_s`: vetor crescente com o instante de cada amostra, em segundos;
- `raw_angle_deg`: ângulo absoluto bruto correspondente, em graus e limitado
  ciclicamente a uma volta;
- `segments`: célula contendo vetores de índices. Cada vetor seleciona um
  intervalo independente de rotação estável e aproximadamente uniforme;
- `bin_count`: quantidade de pontos da LUT; padrão 256;
- `full_scale_counts`: contagens nativas por volta; padrão 16384;
- `max_abs_correction_counts`: maior correção aceita; padrão 1024 contagens.

**Saída:** estrutura `calibration`, contendo a LUT em contagens e graus, payload,
CRC, resolução, número de harmônicas, métricas de erro angular, métricas de
ondulação de velocidade e quantidade de voltas utilizadas.

**Implementação por blocos:**

1. Aplica valores padrão e valida configuração, amostras e segmentos.
2. Separa voltas alternadas de cada segmento entre treinamento e validação.
3. Calcula o erro de fase de cada amostra supondo velocidade uniforme dentro
   de cada volta completa.
4. Agrupa o erro de treinamento por posição angular usando a mediana.
5. Preenche bins ausentes circularmente.
6. Filtra o perfil periódico no domínio da frequência, inicialmente mantendo
   até oito harmônicas.
7. Quantiza a correção para contagens inteiras e reduz o número de harmônicas
   até obter um mapa angular monotônico.
8. Valida a LUT final com as mesmas regras do firmware.
9. Mede, nas voltas reservadas, o erro angular e a ondulação de velocidade antes
   e depois da correção.
10. Serializa a tabela e calcula o CRC do payload.

A função não decide quais trechos do ensaio estão estáveis. Essa escolha é
responsabilidade da aplicação, que constrói `segments`.

## `esp_angle_lut_apply()`

```octave
corrected_deg = esp_angle_lut_apply (raw_deg, correction_counts, ...
                                      full_scale_counts)
```

**Tipo:** função pública.

**Chamada por:** `esp_angle_lut_calculate()`, testes e aplicações externas.

**Chama:** `esp_angle_lut_validate()`.

**O que faz:** aplica em Octave a correção cíclica interpolada que será aplicada
pelo firmware.

**Implementação por blocos:**

1. Valida a resolução e a monotonicidade da tabela.
2. Preserva o formato original, como vetor linha, coluna ou matriz.
3. Normaliza cada ângulo para `[0, 360)`.
4. Localiza os dois bins vizinhos e calcula a fração entre eles.
5. Interpola linearmente as correções em contagens.
6. Converte a correção para graus, soma-a ao ângulo e realiza o wrap circular.

`full_scale_counts` é opcional e assume 16384 quando omitido.

## `esp_angle_lut_validate()`

```octave
[valid, report] = esp_angle_lut_validate (correction_counts, ...
                                          full_scale_counts, ...
                                          max_abs_correction_counts)
```

**Tipo:** função pública.

**Chamada por:** `esp_angle_lut_calculate()`, `esp_angle_lut_apply()`,
`validate_configuration()`, testes e adaptadores externos.

**Chama:** nenhuma função deste conjunto.

**O que faz:** reproduz no host as regras estruturais e de segurança usadas pelo
firmware antes da instalação de uma LUT.

**Implementação por blocos:**

1. Confirma que número de bins e resolução são potências de dois dentro dos
   limites do componente e que uma grandeza divide a outra.
2. Confirma que as correções são inteiras, finitas e representáveis em
   `int16`.
3. Calcula a maior correção absoluta.
4. Calcula cada passo angular corrigido, inclusive o passo do último para o
   primeiro bin.
5. Exige passos positivos e não superiores a quatro larguras nominais de bin.

`valid` é verdadeiro somente quando amplitude e monotonicidade são válidas.
`report` informa, entre outros campos, `correction_range`, `monotonic`, correção
máxima e passos corrigidos mínimo e máximo.

## `esp_angle_lut_pack()`

```octave
[payload, crc] = esp_angle_lut_pack (correction_counts)
```

**Tipo:** função pública.

**Chamada por:** `esp_angle_lut_calculate()`, testes e aplicações externas.

**Chama:** `esp_angle_lut_int16_le_bytes()` e
`esp_angle_lut_crc32_ieee()`.

**O que faz:** reúne as duas operações necessárias para transmitir uma LUT:
serializa suas correções e calcula o CRC do payload resultante.

**Saídas:** `payload` é um vetor coluna `uint8`; `crc` é um escalar `uint32`.

## `esp_angle_lut_int16_le_bytes()`

```octave
payload = esp_angle_lut_int16_le_bytes (values)
```

**Tipo:** função pública de baixo nível.

**Chamada por:** `esp_angle_lut_pack()` e aplicações externas que precisem do
formato binário diretamente.

**Chama:** nenhuma função deste conjunto.

**O que faz:** valida valores inteiros no intervalo de `int16` e emite dois
bytes por correção, primeiro o byte menos significativo. Valores negativos são
representados em complemento de dois.

## `esp_angle_lut_crc32_ieee()`

```octave
crc = esp_angle_lut_crc32_ieee (bytes)
```

**Tipo:** função pública de baixo nível.

**Chamada por:** `esp_angle_lut_pack()`, testes e aplicações externas.

**Chama:** nenhuma função deste conjunto.

**O que faz:** calcula CRC-32/IEEE refletido, com polinômio refletido
`0xEDB88320`, inicialização e XOR final `0xFFFFFFFF`. Uma tabela persistente de
256 entradas é construída somente na primeira chamada e reutilizada depois.

**Saída:** CRC escalar `uint32`, compatível com `esp_crc32_le()` no firmware.

## `esp_angle_lut_plot()`

```octave
figure_handle = esp_angle_lut_plot (calibration, font_size)
```

**Tipo:** função pública.

**Chamada por:** aplicações externas. Nenhuma função da biblioteca a chama.

**Chama:** nenhuma função deste conjunto; utiliza apenas primitivas gráficas do
Octave.

**O que faz:** cria uma figura com dois subplots. O primeiro mostra correção em
função do ângulo. O segundo compara os erros de fase antes e depois da LUT e
apresenta as métricas RMS angular e de velocidade. `font_size` é opcional e vale
12 por padrão.

## Funções internas de `esp_angle_lut_calculate.m`

### `validate_configuration()`

**Chamada por:** somente `esp_angle_lut_calculate()`.

**Chama:** `esp_angle_lut_validate()` usando uma LUT nula, para verificar a
compatibilidade entre número de bins, resolução e limite de correção antes de
processar as amostras.

### `revolution_errors()`

**Chamada por:** somente `esp_angle_lut_calculate()`, uma vez para cada segmento.

**Chama:** `wrap_pi()`.

**O que faz:** desembrulha o ângulo, determina o sentido da rotação, encontra os
cruzamentos entre voltas completas e constrói uma trajetória angular ideal com
velocidade uniforme em cada volta. Retorna fase medida, erro angular ideal menos
medido e o número da volta de cada amostra.

### `bin_median()`

**Chamada por:** somente `esp_angle_lut_calculate()`, para treinamento e
validação.

**Chama:** nenhuma função deste conjunto.

**O que faz:** distribui amostras entre bins angulares e calcula a mediana de
cada bin. A mediana reduz a influência de amostras espúrias.

### `fill_circular()`

**Chamada por:** somente `esp_angle_lut_calculate()`, para treinamento e
validação.

**Chama:** nenhuma função deste conjunto.

**O que faz:** exige cobertura mínima de um quarto dos bins e preenche lacunas
por interpolação linear. Repete posições válidas uma volta antes e depois para
que a interpolação respeite a natureza circular do ângulo.

### `circular_harmonic_smooth()`

**Chamada por:** somente `esp_angle_lut_calculate()`.

**Chama:** nenhuma função deste conjunto.

**O que faz:** calcula a FFT do erro por ângulo, preserva o nível contínuo e a
quantidade solicitada de harmônicas positivas e negativas, zera as demais e
retorna ao domínio angular pela IFFT.

### `interp_cyclic()`

**Chamada por:** somente `esp_angle_lut_calculate()`.

**Chama:** nenhuma função deste conjunto.

**O que faz:** interpola uma sequência uniformemente distribuída ao longo de
uma volta, usando o primeiro bin como sucessor circular do último.

### `wrap_pi()`

**Chamada por:** `revolution_errors()` e `esp_angle_lut_calculate()`.

**Chama:** nenhuma função deste conjunto.

**O que faz:** normaliza ângulos em radianos para o intervalo `[-pi, pi)`.

## Teste independente

`test_esp_angle_lut.m` não é uma função da biblioteca; é um script executável.
Ele testa o vetor conhecido do CRC, serialização, cálculo e aplicação em 14 e
12 bits e rejeição de uma LUT não monotônica:

```text
octave --quiet components/esp_angle_lut/tools/octave/test_esp_angle_lut.m
```

O teste usa dados sintéticos e não depende de porta serial, pacote
`instrument-control`, driver de sensor ou firmware conectado.
