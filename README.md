# Maple X3 Receiver v0.2

Firmware para XTEINK X3 que convierte el lector en un espejo interactivo de Maple v7.5.

## Funcionamiento

Al arrancar crea una red Wi-Fi abierta llamada `Maple-X3` con IP `192.168.4.1`.
Maple se sincroniza contra `http://192.168.4.1/api/v1`.

El X3 recibe el estado completo de Maple, lo guarda en la microSD y permite:

- marcar/desmarcar tareas;
- marcar/desmarcar hábitos del día;
- marcar/desmarcar pasos de proyectos;
- marcar/desmarcar sets de ejercicios;
- subir/bajar el peso (kg) de cada ejercicio;
- leer las libretas de Escribir en modo solo lectura.

La estructura sigue siendo propiedad de Maple: en el X3 no se crean, borran, renombran o reordenan elementos y no se modifican reps ni número de sets.

## Botones

- Izquierda/Derecha: cambiar pestaña; en el detalle de un ejercicio cambian el peso cuando `PESO` está seleccionado.
- Arriba/Abajo: mover selección.
- Confirmar: abrir o marcar/desmarcar.
- Atrás: volver al nivel anterior; desde raíz vuelve a Hoy.

## API

- `GET /api/v1/status`
- `GET /api/v1/changes`
- `POST /api/v1/state`

Los cambios locales pendientes se guardan en `/.maple/changes.json` y se eliminan únicamente cuando Maple los confirma con `X-Maple-Ack-Through`.

## Compilar

El workflow de GitHub Actions genera `Maple-X3-Receiver-v0.2.bin` usando FreeInk SDK fijado a un commit conocido.
