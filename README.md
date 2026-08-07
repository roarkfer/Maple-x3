# Maple X3 v1.0

Port nativo para XTEINK X3 de la webapp Maple.

## Qué incluye
- Tareas: crear, editar, marcar, reordenar.
- Hábitos: crear, editar, marcar hoy, reordenar (bitmap local de 31 días; la vista anual web no está en esta primera build).
- Ejercicios: carpetas, ejercicios, sets/reps/kg y reordenamiento.
- Persistencia en NVS.
- Teclado en pantalla con botones físicos.
- Refresco e-ink y suspensión con pulsación larga de Power.

## Controles
- Izquierda/Derecha: cambiar pestaña.
- Arriba/Abajo: mover selección.
- OK: abrir/alternar/editar según el modo.
- Power corto: entrar/salir de edición.
- Izquierda/Derecha en edición: reordenar.
- Back: volver/salir de edición.
- Power largo: dormir.

## Compilar
Este proyecto usa FreeInk SDK como submódulo.

```bash
git submodule update --init --recursive
pio run -e xteink_x3
```

El binario sale en:
`.pio/build/xteink_x3/firmware.bin`

## Flashear
Usa el flasher de CrossPoint y selecciona **Custom .bin**, o por CLI:

```bash
esptool.py --chip esp32c3 --port /dev/cu.usbmodemXXXX --baud 921600 write_flash 0x10000 .pio/build/xteink_x3/firmware.bin
```

IMPORTANTE: esta es una build de desarrollo. Conserva una copia del firmware oficial/FlowE antes de flashear.
