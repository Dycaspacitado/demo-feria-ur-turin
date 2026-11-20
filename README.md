# Simulación 3D de cobots GRUPRO: UR5 + Turin TCR050

Este proyecto es una aplicación web 3D, hecha con Three.js y TypeScript, con tres simulaciones de los dos cobots trabajando en coordinación:

1. **Transferencia y entrega del llavero.** El UR5 toma el llavero y se lo pasa al Turin. El Turin lo entrega en la mano del cliente.
2. **Paletizado de cubos GRUPRO.** Los dos robots apilan 6 cubos. Leída de arriba hacia abajo, la torre dice G‑R‑U‑P‑R‑O. Después la desarman en orden inverso y cada cubo vuelve a su lugar de la bandeja, así la celda queda lista para el próximo ciclo.
3. **Producción continua + visitante.** Los robots arman y desarman la torre sin parar. Llega un visitante y pulsa el botón del pedestal. Cada robot termina su cubo en curso, atiende el pedido (llavero del UR al Turin y del Turin al visitante) y vuelve al paletizado donde lo había dejado.

## Cómo abrirlo

### En la feria o en cualquier PC
Abrí **`Simulacion-GRUPRO.html`** con doble clic en Chrome o Edge. Es un único archivo de unos 1,5 MB con todo incluido: modelos, Three.js y textos. No necesita internet ni instalar nada.

### Para modificarlo
Hace falta Node.js 22.12 o superior.

```powershell
npm install
npm run dev            # http://localhost:5180
npm run verify         # ejecuta las tres simulaciones sin interfaz y comprueba colisiones, IK y resultado
npm run verify -- barrido   # pide la visita en 18 momentos distintos del ciclo y comprueba que siempre se atienda bien (tarda unos minutos)
npm run build:single   # regenera dist-single/Simulacion-GRUPRO.html
npm run models         # regenera src/models/robots.json desde los VRML de models_origen/
```

## Controles

| Control | Función |
|---|---|
| Pestañas 1 / 2 / 3 (teclas 1, 2 y 3) | Elegir la simulación |
| 🔔 Llamar a un visitante (tecla V) | Solo en la simulación 3: entra un visitante, pulsa el botón y los robots lo atienden. La primera visita llega sola a los 22 s |
| ▶ Iniciar / ⏸ Pausar (barra espaciadora) | Reproducir o pausar |
| ↺ Reiniciar (tecla R) | Volver al estado inicial |
| Velocidad | De 0,25× a 4× (solo cambia el tiempo de reproducción, no la trayectoria) |
| Repetir ciclo | Simulaciones 1 y 2: al terminar espera 3,5 s y arranca de nuevo. La simulación 3 ya es continua |
| Trayectorias del TCP | Dibuja el recorrido de la herramienta de cada robot |
| Envolventes de colisión | Muestra las esferas que se usan para medir la distancia entre robots |
| Cámaras | General, Frontal, Superior y Detalle (zona de transferencia o torre) |
| Mouse | Botón izquierdo gira, rueda hace zoom, botón derecho desplaza |

El panel derecho muestra lo siguiente:
- Para cada robot: el estado, la operación en curso, la pinza (apertura y objeto sujeto), los ángulos J1 a J6 y la posición del TCP.
- Las señales de coordinación encendidas o apagadas.
- La distancia entre robots.
- El registro de eventos.

## Modelos 3D

- **Origen.** Los archivos `UR5.cojt/ur5.jt` y `TCR050.cojt/TCR050.jt` (Siemens JT 9.5 y 10.0) se exportaron desde NX a VRML. Esos VRML están en `models_origen/` y se usan **sin modificar la geometría**: 17 piezas del UR5 y 22 del TCR050.
- **Conversión.** `tools/build_models.py` asigna cada pieza a su eslabón y lleva el Turin a la posición de pie. En NX estaba montado invertido. También calcula las esferas envolventes para la detección de colisiones.
- **Cinemática del UR5.** Usa la tabla DH oficial del UR5 de la serie CB (d1 = 89,159, a2 = 425, a3 = 392,25, d4 = 109,15, d5 = 94,65 y d6 = 82,3 mm). Está verificada contra los centros de las tapas en la malla, con diferencias menores a 1 mm.
- **Cinemática del TCR050.** Los ejes se midieron sobre la propia malla, en los centros de los anillos y tapas de cada articulación. Brazo de 439,5 mm y antebrazo de 353 mm.
- **Pinzas.** Son modelos propios de pinza paralela de dos dedos, con carrera de 0 a 86 mm, porque no venían en los archivos. Hay una por robot, con su color.

## Cómo funciona

- **Programas.** Cada robot ejecuta su propio programa de pasos, como lo haría su controlador: MoveJ, MoveL, pinza, esperas y señales. Los programas están en `src/sim/scenarioKeychain.ts`, `src/sim/scenarioTower.ts` y `src/sim/scenarioService.ts`.
- **Sincronización.** Los robots se coordinan **solo con señales digitales**, como se cablearían entre los dos controladores:
  - En la simulación 1 hay un handshake de transferencia: `UR_EN_TRANSFER` → `TURIN_SUJETA` → `UR_FUERA`.
  - En la simulación 2 hay exclusión de la zona de la torre con un protocolo de bit alternado (`UR_FIN_TURNO` / `TURIN_FIN_TURNO`) más las señales `*_EN_ZONA`. Cada robot invierte su bit 6 veces por ciclo, así que al terminar las señales quedan igual que al empezar y el ciclo se puede repetir sin fin.
  - En la simulación 3 el botón activa la señal `PEDIDO`. Cada robot la consulta solo en sus **puntos seguros**: con la pinza vacía, fuera de la torre y cuando **no** es su turno en la torre. Si es su turno, coloca un cubo más y atiende el pedido en el punto seguro siguiente. Así el otro robot, que quizás ya tiene un cubo en la pinza, siempre puede terminar su operación y nunca quedan esperándose mutuamente.
  - En la transferencia de la simulación 3 el Turin solo se acerca cuando el UR ya tomó el llavero (`UR_TOMO`) y avisa con `TURIN_LISTO`. Cada servicio arranca y termina en Home, así la trayectoria es siempre la misma, sin importar en qué punto del ciclo llegó el pedido. Al terminar, el Turin baja `PEDIDO` y los dos retoman el paletizado.
- **Cinemática inversa.** Es numérica y usa mínimos cuadrados amortiguados sobre el producto de exponenciales (`src/kin/robot.ts`). Cada destino se resuelve desde la posición actual, así que no hay saltos de configuración. La pose Home se elige con el codo arriba y la muñeca del lado del propio robot.
- **Movimientos.** Usan un perfil de mínimo tirón (quíntico), con velocidad y aceleración nulas al inicio y al final. MoveL interpola en línea recta con orientación por SLERP. Los tramos de aproximación, toma y retiro son lineales.
- **Agarre.** El objeto queda unido a la pinza solo si al cerrar está realmente entre las mordazas. Si no, se registra un error de agarre. Durante la transferencia los dos robots sujetan el llavero en zonas distintas: el UR por arriba, del lado del aro, y el Turin en horizontal, por el otro extremo.
- **Colisiones.** Se verifican durante toda la simulación:
  - Robot contra robot, con esferas envolventes de la malla original más la pinza.
  - Brazos contra la mesa, el soporte del llavero, las bandejas y la base de la torre.
  - Autocolisión de muñeca y pinza contra hombro y brazo. Si las envolventes se tocan, se confirma con los vértices reales de la malla.
  - Dedos contra la mesa, los soportes y los objetos.
  - Cubo transportado contra los otros cubos y contra la otra pinza.

  Cualquier contacto aparece en el registro y enciende la luz roja de la baliza.
- **Verificación automática** (`npm run verify`):

  | Simulación | Ciclo | Distancia mínima entre robots | Resultado |
  |---|---|---|---|
  | Llavero | 35 s | 13 mm (durante la transferencia, con envolventes) | El cliente recibe el llavero |
  | Torre | 97 s | 104 mm | GRUPRO armada con 0,03 mm de desalineación; los 6 cubos vuelven a su lugar de la bandeja |
  | Producción + visitante | 330 s de prueba | 13 mm (durante la transferencia) | 3 torres, 2 ciclos completos y 2 llaveros entregados: la visita automática y otra pedida a los 150 s |

  En las tres simulaciones no hubo colisiones, errores de IK ni saltos articulares. En el barrido de 18 momentos distintos de pedido (de 26 s a 230 s), todas las visitas se atendieron sin errores.

## Limitaciones y supuestos

- **Ejes del TCR050.** No se encontró la tabla DH del TCR050 (los manuales disponibles son de la serie TKB), así que los ejes se obtuvieron de la geometría. Los límites articulares son supuestos: ±360°, salvo J3 = ±165°.
- **Física.** No se usa un motor de física. El agarre es cinemático y las colisiones se verifican geométricamente, que es lo adecuado para mostrar trayectorias planificadas.
- **Cliente.** Es un maniquí simplificado con un brazo articulado por IK de dos segmentos.
- **Colores.** Los colores del Turin se tomaron de la captura de NX, porque el VRML del Turin no los traía.

## Estructura

```
models_origen/   VRML exportados desde NX (UR5 y TCR050)
tools/           vrml_parse.py, build_models.py (conversión a src/models/robots.json)
src/kin/         álgebra, cinemática POE, IK numérica
src/sim/         motor de pasos, pinza, colisiones, escenarios (programas de cada robot)
src/view/        escena Three.js (solo lee el estado de la simulación)
src/ui/          interfaz
scripts/verify.ts  verificación automática sin interfaz
```

## Publicación web (Cloudflare Pages)

Está publicada en **https://demo-feria-cobots-grupro.pages.dev**. Es un proyecto de Cloudflare Pages, gratis y solo con archivos estáticos.

Para actualizarla después de un cambio:

```powershell
npm run build:single
Copy-Item Simulacion-GRUPRO.html deploy-worker\public\index.html
cd deploy-worker; npx wrangler pages deploy public --project-name demo-feria-cobots-grupro --branch main
```

No hace falta `--force`: solo se usó una vez para crear el proyecto en Pages en lugar de en Workers.

Antes estuvo publicada como Worker en `demo-feria-cobots-grupro.eldenwynn-cloud.workers.dev`. El Worker se conserva, pero su página está desactivada (`"workers_dev": false` en `deploy-worker/wrangler.jsonc`). Para reactivarla, se pone `true` y se corre `npx wrangler deploy`.
