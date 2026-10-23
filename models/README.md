# Modelos de Fase 1

Se necesitan `pose_landmarker_lite.task` y `hand_landmarker.task` de MediaPipe.
Se usa la versión 1 de cada bundle float16, con inferencia CPU.

`scripts/download_models.py` muestra las URL sin descargar. Con `--download`,
descarga solo estos dos archivos por HTTPS desde `storage.googleapis.com`, con
timeout, límite de 40 MiB por archivo y publicación al terminar la descarga.
No extrae ni ejecuta contenido. No sobrescribe modelos existentes.
El `.sha256` registra el archivo obtenido; no es una firma ni una suma verificada
contra un catálogo independiente. Los modelos y sus sumas se ignoran en Git.

El usuario también puede colocar manualmente los bundles oficiales aquí o usar
`--models-dir /ruta`. La aplicación nunca descarga modelos al arrancar.

Referencias oficiales y condiciones de los modelos:

- [Pose Landmarker y modelo Lite](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker#models).
- [Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker#models).
