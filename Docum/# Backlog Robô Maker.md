# Backlog do Projeto – Robô Maker

**Integrantes:** Vinícius Bernardino e Gabriel Pacheco

| ID  | Tarefa                                                  | Responsável        | Prioridade | Dependência | Critério de aceite                                 |
| --- | ------------------------------------------------------- | ------------------ | ---------- | ----------- | -------------------------------------------------- |
| T01 | Levantar componentes e definir arquitetura do projeto   | Vinícius + Gabriel | Alta       | —           | Componentes e arquitetura definidos                |
| T02 | Definir layout e estrutura do robô                      | Gabriel            | Alta       | T01         | Posicionamento dos componentes definido            |
| T03 | Montar chassi, motores, rodas e roda boba               | Gabriel            | Alta       | T02         | Estrutura física montada e estável                 |
| T04 | Fixar Arduino, bateria, ponte H, Bluetooth e sensor     | Gabriel            | Alta       | T03         | Componentes fixados corretamente                   |
| T05 | Montar circuito de alimentação                          | Gabriel            | Alta       | T01,T04     | Sistema alimentado corretamente                    |
| T06 | Conectar motores à ponte H e ao Arduino                 | Gabriel + Vinícius | Alta       | T05         | Motores respondem aos sinais                       |
| T07 | Testar movimentação dos motores                         | Vinícius + Gabriel | Alta       | T06         | Dois motores funcionando corretamente              |
| T08 | Programar movimentos básicos                            | Vinícius           | Alta       | T07         | Frente, ré, esquerda, direita e parada funcionando |
| T09 | Conectar e configurar sensor HC-SR04                    | Gabriel + Vinícius | Alta       | T05         | Sensor realizando leituras                         |
| T10 | Implementar detecção e parada diante de obstáculos      | Vinícius           | Alta       | T09         | Robô interrompe avanço diante de obstáculo         |
| T11 | Conectar e configurar módulo Bluetooth                  | Gabriel + Vinícius | Alta       | T05         | Comunicação Bluetooth funcionando                  |
| T12 | Implementar controle por Bluetooth                      | Vinícius           | Alta       | T08,T11     | Todos os movimentos controláveis via Bluetooth     |
| T13 | Implementar fail-safe e calibrar sensor                 | Vinícius + Gabriel | Média      | T10,T12     | Sistema entra em estado seguro e sensor calibrado  |
| T14 | Realizar testes integrados                              | Vinícius + Gabriel | Alta       | T13         | Todas as funções operam em conjunto                |
| T15 | Corrigir falhas e organizar cabeamento                  | Vinícius + Gabriel | Alta       | T14         | Sem falhas críticas e sem cabos interferindo       |
| T16 | Testar autonomia e percurso completo                    | Vinícius + Gabriel | Média      | T15         | Percurso concluído e autonomia registrada          |
| T17 | Finalizar documentação do circuito, software e montagem | Vinícius + Gabriel | Média      | T16         | Documentação concluída                             |
| T18 | Preparar demonstração final                             | Vinícius + Gabriel | Alta       | T17         | Robô pronto para apresentação                      |
