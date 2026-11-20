# Modelos 3D originales

Estos son los archivos de partida de la simulación, tal como se recibieron. No se modificaron.

| Carpeta / archivo | Qué es |
|---|---|
| `NX_Process_Simulate/UR5.cojt/` | Robot Universal Robots UR5 (serie CB): paquete `.cojt` de Siemens Process Simulate. Contiene la geometría `ur5.jt` (JT 9.5), los nombres de los eslabones (`kin_graph.xml`) y los parámetros de movimiento (`motionparameters.e`). |
| `NX_Process_Simulate/TCR050.cojt/` | Cobot Turin TCR050 en el mismo formato: geometría `TCR050.jt` (JT 10.0) y nombres de los eslabones. |
| `STEP/TCR050_Turin3D.stp` | Exportación del TCR050 a STEP desde NX 12 (22 sólidos, sin colores). La del UR5 a STEP salió vacía, porque su JT solo trae facetas, así que no se incluye. |
| `referencia_colores_NX.png` | Captura de NX usada como referencia de colores, sobre todo para el Turin, cuyo VRML no trae colores. |

La simulación usa las exportaciones VRML de esos mismos modelos, hechas desde NX. Están en `../models_origen/` (`UR5_NX.wrl` y `TCR050_NX.wrl`), y `tools/build_models.py` las convierte en `src/models/robots.json`.
